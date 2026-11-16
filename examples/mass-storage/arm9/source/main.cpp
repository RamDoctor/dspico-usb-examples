#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nds/disc_io.h>
#include <nds/arm9/cache.h>
#include <fat.h>
#include <libtwl/gfx/gfx.h>
#include <libtwl/gfx/gfxBackground.h>
#include <libtwl/gfx/gfxStatus.h>
#include <libtwl/mem/memVram.h>
#include <libtwl/mem/memExtern.h>
#include <libtwl/rtos/rtosIrq.h>
#include <libtwl/rtos/rtosThread.h>
#include <libtwl/rtos/rtosEvent.h>
#include <libtwl/ipc/ipcSync.h>
#include <libtwl/ipc/ipcFifoSystem.h>
#include "ipcChannels.h"
#include "usbStatusIpc.h"
#include "dldiIpc.h"
#include "picoLoader7.h"
#include "font8x8.h"

extern u8 gDldiStub[];

#define REG_KEYINPUT    (*(vu16*)0x04000130)
#define KEY_START_BIT   (1u << 3)
#define REG_POWERCNT    (*(vu16*)0x04000304)
#define BG_PAL_MAIN     ((vu16*)0x05000000)
#define BG_PAL_SUB      ((vu16*)0x05000400)

#define RGB15(r, g, b)  ((u16)(((r) & 31) | (((g) & 31) << 5) | (((b) & 31) << 10)))

// VRAM layout per engine (within a single 128 KB bank: VRAM A for Main, VRAM C for Sub)
// 0x0000..0x1FFF : BG0 4bpp tiles (charBase 0)
// 0x1000..0x17FF : BG0 32x32 tilemap (mapBase 2 -> offset 0x1000)
// 0x4000..0x1BFFF: BG3 256x192 16bpp bitmap (mapBase 1 -> offset 0x4000)
static constexpr u32 BG_BITMAP_OFFSET_BYTES = 0x4000;
static constexpr u32 BG_MAP_OFFSET_U16      = 0x1000 / 2;
static constexpr u32 BG_IMAGE_BYTES         = 256 * 192 * 2;

enum TextPal : u16
{
    PAL_WHITE  = 0,
    PAL_GREEN  = 1,
    PAL_YELLOW = 2,
    PAL_RED    = 3,
    PAL_CYAN   = 4,
    PAL_GRAY   = 5
};

static rtos_event_t sVblankEvent;

// USB status updated by ARM7 via IPC_CHANNEL_SOUND
static volatile u32 sUsbStatusFlags = 0;
static volatile u32 sReadHoldFrames = 0;
static volatile u32 sWriteHoldFrames = 0;
static volatile u32 sReadKB = 0;
static volatile u32 sWriteKB = 0;
static volatile bool sLoaderDisconnected = false;

// Preloaded Pico Loader binaries for returning to fat:/_picoboot.nds
static u8* sPicoLoader9Buf = nullptr;
static u32 sPicoLoader9Size = 0;
static u8* sPicoLoader7Buf = nullptr;
static u32 sPicoLoader7Size = 0;

static char sThemeName[64] = "material";

// -----------------------------------------------------------------------------
// DISC_INTERFACE wrapper using dldiIpc (runs BEFORE TinyUSB starts on ARM7)
// -----------------------------------------------------------------------------
static bool discStartup() { return true; }
static bool discIsInserted() { return true; }

static bool discReadSectors(sec_t sector, sec_t numSectors, void* buffer)
{
    dldi_readSectors(buffer, sector, numSectors);
    return true;
}

static bool discWriteSectors(sec_t sector, sec_t numSectors, const void* buffer)
{
    dldi_writeSectors(buffer, sector, numSectors);
    return true;
}

static bool discClearStatus() { return true; }
static bool discShutdown() { return true; }

static const DISC_INTERFACE sDldiIpcDisc = {
    0x44494C44, // "DLDI"
    FEATURE_MEDIUM_CANREAD | FEATURE_MEDIUM_CANWRITE,
    discStartup,
    discIsInserted,
    discReadSectors,
    discWriteSectors,
    discClearStatus,
    discShutdown
};

// -----------------------------------------------------------------------------
// Interrupts & IPC handlers
// -----------------------------------------------------------------------------
static void vblankIrq(u32 irqMask)
{
    (void)irqMask;
    rtos_signalEvent(&sVblankEvent);
}

static void usbStatusIpcHandler(u32 channel, u32 data, void* arg)
{
    (void)channel;
    (void)arg;
    u32 tag = data >> USB_IPC_TAG_SHIFT;
    u32 val = data & USB_IPC_VAL_MASK;

    if (tag == USB_IPC_TAG_STATUS)
    {
        sUsbStatusFlags = val;
        if (val & USB_STATUS_READING)
            sReadHoldFrames = 18;
        if (val & USB_STATUS_WRITING)
            sWriteHoldFrames = 24;
    }
    else if (tag == USB_IPC_TAG_READ_KB)
    {
        sReadKB = val;
        sReadHoldFrames = 18;
    }
    else if (tag == USB_IPC_TAG_WRITE_KB)
    {
        sWriteKB = val;
        sWriteHoldFrames = 24;
    }
}

static void loaderIpcHandler(u32 channel, u32 data, void* arg)
{
    (void)channel;
    (void)arg;
    if (data == LOADER_RESP_DISCONNECTED)
    {
        sLoaderDisconnected = true;
    }
}

// -----------------------------------------------------------------------------
// Tiny file & JSON helpers
// -----------------------------------------------------------------------------
static u8* readWholeFile(const char* path, u32* outSize)
{
    FILE* f = fopen(path, "rb");
    if (!f)
        return nullptr;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    if (sz <= 0)
    {
        fclose(f);
        return nullptr;
    }
    fseek(f, 0, SEEK_SET);

    u32 allocSize = ((u32)sz + 3u) & ~3u;
    u8* buf = (u8*)malloc(allocSize);
    if (!buf)
    {
        fclose(f);
        return nullptr;
    }
    memset(buf, 0, allocSize);

    size_t readBytes = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (readBytes != (size_t)sz)
    {
        free(buf);
        return nullptr;
    }

    if (outSize)
        *outSize = (u32)sz;
    return buf;
}

static const char* skipWs(const char* p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
        p++;
    return p;
}

static const char* findJsonKeyValue(const char* json, const char* key)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    size_t patLen = strlen(pattern);

    const char* p = json;
    while ((p = strstr(p, pattern)) != nullptr)
    {
        const char* after = skipWs(p + patLen);
        if (*after == ':')
            return skipWs(after + 1);
        p += patLen;
    }
    return nullptr;
}

static bool jsonGetString(const char* json, const char* key, char* out, size_t outSize)
{
    const char* v = findJsonKeyValue(json, key);
    if (!v || *v != '"')
        return false;
    v++;
    size_t i = 0;
    while (*v && *v != '"' && i + 1 < outSize)
    {
        if (*v == '\\' && v[1])
            v++;
        out[i++] = *v++;
    }
    out[i] = '\0';
    return i > 0;
}

static bool jsonGetBool(const char* json, const char* key, bool fallback)
{
    const char* v = findJsonKeyValue(json, key);
    if (!v)
        return fallback;
    if (strncmp(v, "true", 4) == 0)
        return true;
    if (strncmp(v, "false", 5) == 0)
        return false;
    return fallback;
}

static int jsonGetInt(const char* json, const char* key, int fallback)
{
    const char* v = findJsonKeyValue(json, key);
    if (!v)
        return fallback;
    return atoi(v);
}

// -----------------------------------------------------------------------------
// Graphics / Text Console / Theme Background
// -----------------------------------------------------------------------------
static void wordCopyToVram(vu16* dst, const u16* src, u32 byteCount)
{
    u32 words = byteCount / 4;
    vu32* dst32 = (vu32*)dst;
    const u32* src32 = (const u32*)src;
    for (u32 i = 0; i < words; i++)
        dst32[i] = src32[i];
}

static void uploadFontTiles(vu16* tileBase)
{
    // Tile 0: fully transparent (index 0)
    for (u32 i = 0; i < 16; i++)
        tileBase[i] = 0x0000;

    // Tile 1: solid dark panel background (index 7)
    for (u32 i = 0; i < 16; i++)
        tileBase[16 + i] = 0x7777;

    // Tiles 2..96: ASCII 0x20..0x7E over dark panel (index 7 bg, index 1 fg)
    for (u32 c = 0; c < 95; c++)
    {
        vu16* dst = tileBase + (2 + c) * 16;
        for (u32 row = 0; row < 8; row++)
        {
            u8 bits = kFont8x8[c][row];
            u32 row4bpp = 0;
            for (u32 col = 0; col < 8; col++)
            {
                u32 pix = (bits & (0x80u >> col)) ? 1u : 7u;
                row4bpp |= (pix << (col * 4));
            }
            dst[row * 2 + 0] = (u16)(row4bpp & 0xFFFFu);
            dst[row * 2 + 1] = (u16)(row4bpp >> 16);
        }
    }
}

static void initTextPalettes(vu16* pal)
{
    const u16 fgColors[6] = {
        RGB15(31, 31, 31), // PAL_WHITE
        RGB15(10, 31, 12), // PAL_GREEN
        RGB15(31, 26, 6),  // PAL_YELLOW
        RGB15(31, 9, 9),   // PAL_RED
        RGB15(12, 27, 31), // PAL_CYAN
        RGB15(20, 21, 24)  // PAL_GRAY
    };

    for (u32 p = 0; p < 6; p++)
    {
        vu16* row = pal + p * 16;
        row[0] = 0;
        row[1] = fgColors[p];
        row[7] = RGB15(2, 3, 5); // dark readable panel backing
    }
}

static void clearTextLayer(vu16* map)
{
    for (u32 i = 0; i < 32 * 24; i++)
        map[i] = 0; // Tile 0 = transparent (shows theme background)
}

static void drawBox(vu16* map, int x, int y, int w, int h)
{
    for (int ry = y; ry < y + h; ry++)
    {
        for (int rx = x; rx < x + w; rx++)
        {
            map[ry * 32 + rx] = 1; // Tile 1 = dark panel fill
        }
    }

    // Subtle border using ASCII characters
    for (int rx = x + 1; rx < x + w - 1; rx++)
    {
        map[y * 32 + rx]           = (PAL_CYAN << 12) | (u16)(('-' - 0x20) + 2);
        map[(y + h - 1) * 32 + rx] = (PAL_CYAN << 12) | (u16)(('-' - 0x20) + 2);
    }
    for (int ry = y + 1; ry < y + h - 1; ry++)
    {
        map[ry * 32 + x]           = (PAL_CYAN << 12) | (u16)(('|' - 0x20) + 2);
        map[ry * 32 + (x + w - 1)] = (PAL_CYAN << 12) | (u16)(('|' - 0x20) + 2);
    }
    map[y * 32 + x]                   = (PAL_CYAN << 12) | (u16)(('+' - 0x20) + 2);
    map[y * 32 + (x + w - 1)]         = (PAL_CYAN << 12) | (u16)(('+' - 0x20) + 2);
    map[(y + h - 1) * 32 + x]         = (PAL_CYAN << 12) | (u16)(('+' - 0x20) + 2);
    map[(y + h - 1) * 32 + (x + w - 1)] = (PAL_CYAN << 12) | (u16)(('+' - 0x20) + 2);
}

static void drawText(vu16* map, int x, int y, TextPal pal, const char* text)
{
    int cx = x;
    for (const char* p = text; *p && cx < 32; p++, cx++)
    {
        u8 ch = (u8)*p;
        if (ch < 0x20 || ch > 0x7E)
            ch = '?';
        map[y * 32 + cx] = (u16)((pal << 12) | ((ch - 0x20) + 2));
    }
}

static void drawTextPadded(vu16* map, int x, int y, int width, TextPal pal, const char* text)
{
    char buf[33];
    int len = (int)strlen(text);
    if (width > 32) width = 32;
    for (int i = 0; i < width; i++)
        buf[i] = (i < len) ? text[i] : ' ';
    buf[width] = '\0';
    drawText(map, x, y, pal, buf);
}

static void fillSolidBackground(vu16* bmpVram, u16 color15)
{
    u32 pair = ((u32)(color15 | 0x8000u) << 16) | (u32)(color15 | 0x8000u);
    vu32* dst32 = (vu32*)bmpVram;
    u32 words = BG_IMAGE_BYTES / 4;
    for (u32 i = 0; i < words; i++)
        dst32[i] = pair;
}

static bool loadBinBackground(const char* path, vu16* bmpVram)
{
    u32 sz = 0;
    u8* data = readWholeFile(path, &sz);
    if (!data || sz < BG_IMAGE_BYTES)
    {
        free(data);
        return false;
    }

    u16* px = (u16*)data;
    u32 count = BG_IMAGE_BYTES / 2;
    for (u32 i = 0; i < count; i++)
        px[i] |= 0x8000u;

    wordCopyToVram(bmpVram, px, BG_IMAGE_BYTES);
    free(data);
    return true;
}

static void loadActivePicoTheme(vu16* topBmp, vu16* subBmp)
{
    u16 fallbackTop = RGB15(3, 3, 5);
    u16 fallbackSub = RGB15(2, 2, 4);

    char* settingsJson = (char*)readWholeFile("fat:/_pico/settings.json", nullptr);
    if (settingsJson)
    {
        jsonGetString(settingsJson, "theme", sThemeName, sizeof(sThemeName));
        free(settingsJson);
    }

    char path[160];
    snprintf(path, sizeof(path), "fat:/_pico/themes/%s/theme.json", sThemeName);
    char* themeJson = (char*)readWholeFile(path, nullptr);
    if (themeJson)
    {
        bool dark = jsonGetBool(themeJson, "darkTheme", true);
        const char* prim = findJsonKeyValue(themeJson, "primaryColor");
        int r = 160, g = 180, b = 220;
        if (prim)
        {
            r = jsonGetInt(prim, "r", r);
            g = jsonGetInt(prim, "g", g);
            b = jsonGetInt(prim, "b", b);
        }
        if (dark)
        {
            fallbackTop = RGB15(2 + r / 64, 2 + g / 64, 3 + b / 64);
            fallbackSub = RGB15(1 + r / 80, 1 + g / 80, 2 + b / 80);
        }
        else
        {
            fallbackTop = RGB15(26 + r / 64, 26 + g / 64, 27 + b / 64);
            fallbackSub = RGB15(25 + r / 64, 25 + g / 64, 26 + b / 64);
        }
        free(themeJson);
    }

    snprintf(path, sizeof(path), "fat:/_pico/themes/%s/topbg.bin", sThemeName);
    if (!loadBinBackground(path, topBmp))
        fillSolidBackground(topBmp, fallbackTop);

    snprintf(path, sizeof(path), "fat:/_pico/themes/%s/bottombg.bin", sThemeName);
    if (!loadBinBackground(path, subBmp))
        fillSolidBackground(subBmp, fallbackSub);
}

static void initVideo()
{
    REG_POWERCNT = (1u << 0) | (1u << 1) | (1u << 9) | (1u << 15); // LCD + 2D A + 2D B + Main on Top
    REG_MASTER_BRIGHT = 0;
    REG_MASTER_BRIGHT_SUB = 0;

    mem_setVramAMapping(MEM_VRAM_AB_MAIN_BG_00000);
    mem_setVramCMapping(MEM_VRAM_C_SUB_BG_00000);

    // Mode 5: BG0 (text 4bpp) + BG3 (16bpp direct bitmap)
    REG_DISPCNT     = 5u | (1u << 8) | (1u << 11) | (1u << 16);
    REG_DISPCNT_SUB = 5u | (1u << 8) | (1u << 11) | (1u << 16);

    REG_BG0CNT     = 0x0200; // priority 0, charBase 0, mapBase 2 (offset 0x1000)
    REG_BG0CNT_SUB = 0x0200;
    REG_BG0HOFS = 0; REG_BG0VOFS = 0;
    REG_BG0HOFS_SUB = 0; REG_BG0VOFS_SUB = 0;

    REG_BG3CNT     = 0x4185; // priority 1, 256x256, 16-bit bitmap, mapBase 1 (offset 0x4000)
    REG_BG3CNT_SUB = 0x4185;
    gfx_setBg3Affine(256, 0, 0, 256, 0, 0);
    gfx_setSubBg3Affine(256, 0, 0, 256, 0, 0);

    uploadFontTiles(GFX_BG_MAIN);
    uploadFontTiles(GFX_BG_SUB);
    initTextPalettes(BG_PAL_MAIN);
    initTextPalettes(BG_PAL_SUB);

    clearTextLayer(GFX_BG_MAIN + BG_MAP_OFFSET_U16);
    clearTextLayer(GFX_BG_SUB + BG_MAP_OFFSET_U16);
}

// -----------------------------------------------------------------------------
// Return to fat:/_picoboot.nds via Pico Loader
// -----------------------------------------------------------------------------
static void returnToPicoLauncher()
{
    vu16* subMap = GFX_BG_SUB + BG_MAP_OFFSET_U16;
    drawTextPadded(subMap, 2, 16, 28, PAL_YELLOW, "Disconnecting USB...");

    // Wait briefly if a write just finished so host buffers settle
    while (sWriteHoldFrames > 0)
    {
        sWriteHoldFrames--;
        rtos_waitEvent(&sVblankEvent, true, true);
    }

    sLoaderDisconnected = false;
    ipc_sendFifoMessage(IPC_CHANNEL_LOADER, LOADER_CMD_DISCONNECT_USB);
    while (!sLoaderDisconnected)
    {
        rtos_waitEvent(&sVblankEvent, true, true);
    }

    // Blank screens cleanly before overwriting VRAM with Pico Loader
    REG_DISPCNT = 0;
    REG_DISPCNT_SUB = 0;

    mem_setVramAMapping(MEM_VRAM_AB_LCDC);
    mem_setVramBMapping(MEM_VRAM_AB_LCDC);
    mem_setVramCMapping(MEM_VRAM_C_LCDC);
    mem_setVramDMapping(MEM_VRAM_D_LCDC);

    // Prepare picoLoader7 header in main RAM first (VRAM does not allow byte writes!)
    pload_header7_t* hdr = (pload_header7_t*)sPicoLoader7Buf;
    hdr->dldiDriver = gDldiStub;
    hdr->bootDrive = PLOAD_BOOT_DRIVE_DLDI;
    memset(&hdr->loadParams, 0, sizeof(hdr->loadParams));
    strncpy(hdr->loadParams.romPath, "fat:/_picoboot.nds", sizeof(hdr->loadParams.romPath) - 1);
    if (hdr->apiVersion >= 2)
        memset(&hdr->v2, 0, sizeof(hdr->v2));
    if (hdr->apiVersion >= 3)
        hdr->v3.cheats = nullptr;

    // Word-copy both loaders into LCDC VRAM
    wordCopyToVram((vu16*)0x06800000, (const u16*)sPicoLoader9Buf, (sPicoLoader9Size + 3u) & ~3u);
    wordCopyToVram((vu16*)0x06840000, (const u16*)sPicoLoader7Buf, (sPicoLoader7Size + 3u) & ~3u);

    DC_FlushRange(gDldiStub, 16 * 1024);
    DC_FlushAll();
    IC_InvalidateAll();

    // Disable ARM9 IRQs & DMA
    *(vu32*)0x04000208 = 0;   // REG_IME
    *(vu32*)0x04000210 = 0;   // REG_IE
    *(vu32*)0x04000214 = ~0u; // REG_IF
    *(vu32*)0x040000BC = 0;   // DMA0CNT
    *(vu32*)0x040000C8 = 0;   // DMA1CNT
    *(vu32*)0x040000D4 = 0;   // DMA2CNT
    *(vu32*)0x040000E0 = 0;   // DMA3CNT

    // Map VRAM C and D to ARM7 (appears at 0x06000000 on ARM7)
    mem_setVramCMapping(MEM_VRAM_C_ARM7_00000);
    mem_setVramDMapping(MEM_VRAM_D_ARM7_20000);

    ipc_sendFifoMessage(IPC_CHANNEL_LOADER, LOADER_CMD_BOOT_PICOLOADER);
    while (ipc_getArm7SyncBits() != 5);

    ((void (*)(void))0x06800000)();
    while (true);
}

// -----------------------------------------------------------------------------
// Main UI & Loop
// -----------------------------------------------------------------------------
static void drawStaticHud(bool picoLoaderReady)
{
    vu16* topMap = GFX_BG_MAIN + BG_MAP_OFFSET_U16;
    vu16* subMap = GFX_BG_SUB + BG_MAP_OFFSET_U16;

    // Top screen panel
    drawBox(topMap, 1, 2, 30, 12);
    drawText(topMap, 3, 3, PAL_CYAN, "DSpico USB SD Reader");

    char line[32];
    snprintf(line, sizeof(line), "Theme: %.20s", sThemeName);
    drawText(topMap, 3, 5, PAL_GRAY, line);

    // Bottom screen panel
    drawBox(subMap, 1, 11, 30, 9);
    drawText(subMap, 3, 12, PAL_CYAN, "Controls & Tips");
    drawText(subMap, 3, 14, PAL_WHITE, "Connect USB-C to host.");
    drawText(subMap, 3, 15, PAL_GRAY,  "Eject drive before exit.");

    if (picoLoaderReady)
        drawText(subMap, 3, 18, PAL_GREEN, "[START] Return to Menu");
    else
        drawText(subMap, 3, 18, PAL_RED,   "picoLoader bins missing!");
}

static void updateDynamicHud(u32 frame)
{
    vu16* topMap = GFX_BG_MAIN + BG_MAP_OFFSET_U16;
    const u32 flags = sUsbStatusFlags;
    const bool mounted   = (flags & USB_STATUS_MOUNTED) != 0;
    const bool suspended = (flags & USB_STATUS_SUSPENDED) != 0;
    const bool connected = (flags & USB_STATUS_CONNECTED) != 0;
    const bool reading   = sReadHoldFrames > 0;
    const bool writing   = sWriteHoldFrames > 0;

    if (mounted && !suspended)
        drawTextPadded(topMap, 3, 7, 26, PAL_GREEN, "USB: CONNECTED");
    else if (suspended)
        drawTextPadded(topMap, 3, 7, 26, PAL_YELLOW, "USB: SUSPENDED");
    else if (connected)
        drawTextPadded(topMap, 3, 7, 26, PAL_YELLOW, "USB: ENUMERATING...");
    else
        drawTextPadded(topMap, 3, 7, 26, PAL_GRAY, "USB: WAITING FOR HOST");

    static const char spinner[4] = {'|', '/', '-', '\\'};
    char actLine[32];
    if (writing)
    {
        snprintf(actLine, sizeof(actLine), "SD: WRITING! DO NOT UNPLUG %c", spinner[(frame >> 2) & 3]);
        drawTextPadded(topMap, 3, 9, 26, PAL_RED, actLine);
    }
    else if (reading)
    {
        snprintf(actLine, sizeof(actLine), "SD: READING... %c", spinner[(frame >> 2) & 3]);
        drawTextPadded(topMap, 3, 9, 26, PAL_YELLOW, actLine);
    }
    else
    {
        drawTextPadded(topMap, 3, 9, 26, PAL_WHITE, "SD: IDLE");
    }

    char stats[32];
    snprintf(stats, sizeof(stats), "Read:  %lu KB", (unsigned long)sReadKB);
    drawTextPadded(topMap, 3, 11, 26, PAL_CYAN, stats);
    snprintf(stats, sizeof(stats), "Write: %lu KB", (unsigned long)sWriteKB);
    drawTextPadded(topMap, 3, 12, 26, PAL_CYAN, stats);
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    mem_setDsCartridgeCpu(EXMEMCNT_SLOT1_CPU_ARM7);

    rtos_initIrq();
    rtos_startMainThread();
    ipc_initFifoSystem();
    rtos_createEvent(&sVblankEvent);

    initVideo();

    while (ipc_getArm7SyncBits() != 7);

    bool dldiOk = dldi_init();
    bool picoLoaderReady = false;

    if (dldiOk && fatMountSimple("fat", &sDldiIpcDisc))
    {
        loadActivePicoTheme(
            GFX_BG_MAIN + (BG_BITMAP_OFFSET_BYTES / 2),
            GFX_BG_SUB  + (BG_BITMAP_OFFSET_BYTES / 2)
        );

        sPicoLoader9Buf = readWholeFile("fat:/_pico/picoLoader9.bin", &sPicoLoader9Size);
        sPicoLoader7Buf = readWholeFile("fat:/_pico/picoLoader7.bin", &sPicoLoader7Size);
        picoLoaderReady = (sPicoLoader9Buf != nullptr && sPicoLoader7Buf != nullptr);

        // Always unmount FAT before TinyUSB takes raw control of the SD card!
        fatUnmount("fat");
    }
    else
    {
        fillSolidBackground(GFX_BG_MAIN + (BG_BITMAP_OFFSET_BYTES / 2), RGB15(10, 2, 2));
        fillSolidBackground(GFX_BG_SUB  + (BG_BITMAP_OFFSET_BYTES / 2), RGB15(8, 2, 2));
    }

    drawStaticHud(picoLoaderReady);

    ipc_setChannelHandler(IPC_CHANNEL_SOUND, usbStatusIpcHandler, nullptr);
    ipc_setChannelHandler(IPC_CHANNEL_LOADER, loaderIpcHandler, nullptr);

    // Signal ARM7 to initialize TinyUSB Mass Storage
    ipc_setArm9SyncBits(6);

    rtos_setIrqFunc(RTOS_IRQ_VBLANK, vblankIrq);
    rtos_enableIrqMask(RTOS_IRQ_VBLANK);
    gfx_setVBlankIrqEnabled(true);

    // Wait until START is released on boot so holding START doesn't immediately exit
    bool startPreviouslyPressed = ((REG_KEYINPUT & KEY_START_BIT) == 0);
    u32 frame = 0;

    while (true)
    {
        rtos_waitEvent(&sVblankEvent, true, true);
        frame++;

        if (sReadHoldFrames > 0)
            sReadHoldFrames--;
        if (sWriteHoldFrames > 0)
            sWriteHoldFrames--;

        updateDynamicHud(frame);

        bool startDown = ((REG_KEYINPUT & KEY_START_BIT) == 0);
        if (startDown && !startPreviouslyPressed && picoLoaderReady)
        {
            returnToPicoLauncher();
        }
        startPreviouslyPressed = startDown;
    }

    return 0;
}
