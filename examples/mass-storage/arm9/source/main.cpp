#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/statvfs.h>
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
#include "dldiIpcCommand.h"
#include "usbStatusIpc.h"
#include "dldiIpc.h"
#include "picoLoader7.h"
#include "font8x8.h"

extern u8 gDldiStub[];

#define REG_KEYINPUT    (*(vu16*)0x04000130)
#define KEY_SELECT_BIT  (1u << 2)
#define KEY_START_BIT   (1u << 3)
#define REG_POWERCNT    (*(vu16*)0x04000304)
#define BG_PAL_MAIN     ((vu16*)0x05000000)
#define BG_PAL_SUB      ((vu16*)0x05000400)

#define RGB15(r, g, b)  ((u16)(((r) & 31) | (((g) & 31) << 5) | (((b) & 31) << 10)))

static constexpr u32 BG_BITMAP_OFFSET_BYTES = 0x4000;
static constexpr u32 BG_MAP_OFFSET_U16      = 0x1000 / 2;
static constexpr u32 BG_IMAGE_BYTES         = 256 * 192 * 2;
static constexpr u32 TILE_PROGRESS_BASE     = 97;

enum TextPal : u16
{
    PAL_TEXT   = 0,
    PAL_GREEN  = 1,
    PAL_YELLOW = 2,
    PAL_RED    = 3,
    PAL_ACCENT = 4,
    PAL_MUTED  = 5
};

struct StorageStats
{
    bool valid = false;
    u64 totalBytes = 0;
    u64 freeBytes = 0;
    u64 usedBytes = 0;
    u32 usedPercent = 0;
};

static rtos_event_t sVblankEvent;

// USB & RTC status updated by ARM7 via IPC_CHANNEL_SOUND
static volatile u32 sUsbStatusFlags = 0;
static volatile u32 sReadHoldFrames = 0;
static volatile u32 sWriteHoldFrames = 0;
static volatile u32 sReadKB = 0;
static volatile u32 sWriteKB = 0;
static volatile u32 sPackedTime = 0;
static volatile bool sHasTime = false;
static volatile bool sLoaderDisconnected = false;
static volatile bool sDriveSwitchDone = false;
static bool sScreensOff = false;

static StorageStats sDriveStats[2]; // 0 = DSpico SD, 1 = 3DS/DSi SD
static bool sDsiSdAvailable = false;
static int sDisplayedDrive = 0;
static bool sSwitchingUi = false;

// Preloaded Pico Loader binaries for returning to fat:/_picoboot.nds
static u8* sPicoLoader9Buf = nullptr;
static u32 sPicoLoader9Size = 0;
static u8* sPicoLoader7Buf = nullptr;
static u32 sPicoLoader7Size = 0;

// Active theme styling (loaded from theme.json)
static char sThemeName[64] = "material";
static bool sDarkTheme = true;
static int sPrimaryR = 100;
static int sPrimaryG = 200;
static int sPrimaryB = 240;

// -----------------------------------------------------------------------------
// DISC_INTERFACE wrappers (run BEFORE TinyUSB starts on ARM7)
// -----------------------------------------------------------------------------
static bool discStartup() { return true; }
static bool discIsInserted() { return true; }
static bool discClearStatus() { return true; }
static bool discShutdown() { return true; }

static bool dspicoReadSectors(sec_t sector, sec_t numSectors, void* buffer)
{
    dldi_readSectors(buffer, sector, numSectors);
    return true;
}

static bool dspicoWriteSectors(sec_t sector, sec_t numSectors, const void* buffer)
{
    dldi_writeSectors(buffer, sector, numSectors);
    return true;
}

static const DISC_INTERFACE sDldiIpcDisc = {
    0x44494C44, // "DLDI"
    FEATURE_MEDIUM_CANREAD | FEATURE_MEDIUM_CANWRITE,
    discStartup,
    discIsInserted,
    dspicoReadSectors,
    dspicoWriteSectors,
    discClearStatus,
    discShutdown
};

// Boot-only DSi/3DS SD IPC wrapper for measuring storage stats with statvfs
static rtos_event_t sDsiSdEvent;
static volatile u32 sDsiSdResp = 0;
alignas(32) static dldi_ipc_cmd_t sDsiSdCmd;
alignas(32) static u8 sDsiSdTemp[512];

static void dsiSdIpcHandler(u32 channel, u32 data, void* arg)
{
    (void)channel;
    (void)arg;
    sDsiSdResp = data;
    rtos_signalEvent(&sDsiSdEvent);
}

static bool dsiSdSendCmd(u32 cmd, void* buf, u32 sector, u32 count)
{
    sDsiSdCmd.cmd = cmd;
    sDsiSdCmd.buffer = buf;
    sDsiSdCmd.sector = sector;
    sDsiSdCmd.count = count;
    rtos_clearEvent(&sDsiSdEvent);
    DC_FlushRange(&sDsiSdCmd, sizeof(sDsiSdCmd));
    ipc_sendFifoMessage(IPC_CHANNEL_DSI_SD, (u32)&sDsiSdCmd >> 2);
    rtos_waitEvent(&sDsiSdEvent, false, true);
    return sDsiSdResp != 0;
}

static bool dsiSdStartup()
{
    return dsiSdSendCmd(DLDI_IPC_CMD_SETUP, nullptr, 0, 0);
}

static bool dsiSdReadSectors(sec_t sector, sec_t numSectors, void* buffer)
{
    if (numSectors == 0)
        return true;
    if (((u32)buffer & 0x1Fu) == 0)
    {
        DC_InvalidateRange(buffer, 512u * numSectors);
        return dsiSdSendCmd(DLDI_IPC_CMD_READ_SECTORS, buffer, sector, numSectors);
    }
    for (u32 i = 0; i < numSectors; i++)
    {
        DC_InvalidateRange(sDsiSdTemp, 512);
        if (!dsiSdSendCmd(DLDI_IPC_CMD_READ_SECTORS, sDsiSdTemp, sector + i, 1))
            return false;
        memcpy((u8*)buffer + 512u * i, sDsiSdTemp, 512);
    }
    return true;
}

static bool dsiSdWriteSectors(sec_t sector, sec_t numSectors, const void* buffer)
{
    (void)sector;
    (void)numSectors;
    (void)buffer;
    return false; // read-only at boot for statvfs
}

static const DISC_INTERFACE sDsiSdIpcDisc = {
    0x44534953, // "DSIS"
    FEATURE_MEDIUM_CANREAD,
    dsiSdStartup,
    discIsInserted,
    dsiSdReadSectors,
    dsiSdWriteSectors,
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
        if (val > 0)
            sReadHoldFrames = 18;
    }
    else if (tag == USB_IPC_TAG_WRITE_KB)
    {
        sWriteKB = val;
        if (val > 0)
            sWriteHoldFrames = 24;
    }
    else if (tag == USB_IPC_TAG_TIME)
    {
        sPackedTime = val;
        sHasTime = true;
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
    else if (data == LOADER_RESP_DRIVE_SWITCHED)
    {
        sDriveSwitchDone = true;
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

static int clamp255(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return v;
}

static int clamp31(int v)
{
    if (v < 0) return 0;
    if (v > 31) return 31;
    return v;
}

static int hexVal(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static void parsePrimaryColor(const char* themeJson)
{
    const char* prim = findJsonKeyValue(themeJson, "primaryColor");
    if (!prim)
        return;

    if (*prim == '"')
    {
        prim++;
        if (*prim == '#')
            prim++;
        int d[6];
        for (int i = 0; i < 6; i++)
        {
            d[i] = hexVal(prim[i]);
            if (d[i] < 0)
                return;
        }
        sPrimaryR = (d[0] << 4) | d[1];
        sPrimaryG = (d[2] << 4) | d[3];
        sPrimaryB = (d[4] << 4) | d[5];
        return;
    }

    if (*prim == '{')
    {
        sPrimaryR = clamp255(jsonGetInt(prim, "r", sPrimaryR));
        sPrimaryG = clamp255(jsonGetInt(prim, "g", sPrimaryG));
        sPrimaryB = clamp255(jsonGetInt(prim, "b", sPrimaryB));
    }
}

// -----------------------------------------------------------------------------
// Graphics / Text Console / Progress Bar / Theme Background
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
    for (u32 i = 0; i < 16; i++)
        tileBase[i] = 0x0000;

    for (u32 i = 0; i < 16; i++)
        tileBase[16 + i] = 0x7777;

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

    for (u32 fillPx = 0; fillPx <= 8; fillPx++)
    {
        vu16* dst = tileBase + (TILE_PROGRESS_BASE + fillPx) * 16;
        for (u32 row = 0; row < 8; row++)
        {
            u32 row4bpp = 0;
            for (u32 col = 0; col < 8; col++)
            {
                u32 pix;
                if (row == 0 || row == 7)
                    pix = 2u;
                else if (col < fillPx)
                    pix = 1u;
                else
                    pix = 3u;
                row4bpp |= (pix << (col * 4));
            }
            dst[row * 2 + 0] = (u16)(row4bpp & 0xFFFFu);
            dst[row * 2 + 1] = (u16)(row4bpp >> 16);
        }
    }
}

static void initTextPalettes(vu16* pal)
{
    const int r5 = sPrimaryR >> 3;
    const int g5 = sPrimaryG >> 3;
    const int b5 = sPrimaryB >> 3;

    u16 panelBg;
    u16 barTrack;
    u16 accentColor;
    u16 fgColors[6];

    if (sDarkTheme)
    {
        panelBg  = RGB15(1 + r5 / 10, 1 + g5 / 10, 1 + b5 / 10);
        barTrack = RGB15(4 + r5 / 8,  4 + g5 / 8,  4 + b5 / 8);

        int ar = clamp31(r5 + (31 - r5) / 4);
        int ag = clamp31(g5 + (31 - g5) / 4);
        int ab = clamp31(b5 + (31 - b5) / 4);
        accentColor = RGB15(ar, ag, ab);

        fgColors[PAL_TEXT]   = RGB15(31, 31, 31);
        fgColors[PAL_GREEN]  = RGB15(10, 31, 12);
        fgColors[PAL_YELLOW] = RGB15(31, 26, 6);
        fgColors[PAL_RED]    = RGB15(31, 9, 9);
        fgColors[PAL_ACCENT] = accentColor;
        fgColors[PAL_MUTED]  = RGB15(20, 21, 24);
    }
    else
    {
        panelBg  = RGB15(clamp31(27 + r5 / 8),  clamp31(27 + g5 / 8),  clamp31(27 + b5 / 8));
        barTrack = RGB15(clamp31(22 + r5 / 10), clamp31(22 + g5 / 10), clamp31(22 + b5 / 10));

        int ar = r5, ag = g5, ab = b5;
        int lum = ar * 2 + ag * 5 + ab;
        if (lum > 140)
        {
            ar = (ar * 140) / lum;
            ag = (ag * 140) / lum;
            ab = (ab * 140) / lum;
        }
        accentColor = RGB15(clamp31(ar), clamp31(ag), clamp31(ab));

        fgColors[PAL_TEXT]   = RGB15(4, 4, 6);
        fgColors[PAL_GREEN]  = RGB15(2, 19, 4);
        fgColors[PAL_YELLOW] = RGB15(23, 13, 0);
        fgColors[PAL_RED]    = RGB15(25, 3, 3);
        fgColors[PAL_ACCENT] = accentColor;
        fgColors[PAL_MUTED]  = RGB15(12, 12, 15);
    }

    for (u32 p = 0; p < 6; p++)
    {
        vu16* row = pal + p * 16;
        row[0] = 0;
        row[1] = fgColors[p];
        row[2] = accentColor;
        row[3] = barTrack;
        row[7] = panelBg;
    }
}

static void clearTextLayer(vu16* map)
{
    for (u32 i = 0; i < 32 * 24; i++)
        map[i] = 0;
}

static void drawBox(vu16* map, int x, int y, int w, int h)
{
    for (int ry = y; ry < y + h; ry++)
    {
        for (int rx = x; rx < x + w; rx++)
        {
            map[ry * 32 + rx] = 1;
        }
    }

    for (int rx = x + 1; rx < x + w - 1; rx++)
    {
        map[y * 32 + rx]           = (PAL_ACCENT << 12) | (u16)(('-' - 0x20) + 2);
        map[(y + h - 1) * 32 + rx] = (PAL_ACCENT << 12) | (u16)(('-' - 0x20) + 2);
    }
    for (int ry = y + 1; ry < y + h - 1; ry++)
    {
        map[ry * 32 + x]           = (PAL_ACCENT << 12) | (u16)(('|' - 0x20) + 2);
        map[ry * 32 + (x + w - 1)] = (PAL_ACCENT << 12) | (u16)(('|' - 0x20) + 2);
    }
    map[y * 32 + x]                     = (PAL_ACCENT << 12) | (u16)(('+' - 0x20) + 2);
    map[y * 32 + (x + w - 1)]           = (PAL_ACCENT << 12) | (u16)(('+' - 0x20) + 2);
    map[(y + h - 1) * 32 + x]           = (PAL_ACCENT << 12) | (u16)(('+' - 0x20) + 2);
    map[(y + h - 1) * 32 + (x + w - 1)] = (PAL_ACCENT << 12) | (u16)(('+' - 0x20) + 2);
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

static void drawTextCentered(vu16* map, int x, int y, int width, TextPal pal, const char* text)
{
    char buf[33];
    if (width > 32) width = 32;
    int len = (int)strlen(text);
    if (len > width) len = width;
    int padLeft = (width - len) / 2;

    for (int i = 0; i < width; i++)
        buf[i] = ' ';
    for (int i = 0; i < len; i++)
        buf[padLeft + i] = text[i];
    buf[width] = '\0';
    drawText(map, x, y, pal, buf);
}

static void drawProgressBar(vu16* map, int x, int y, int widthTiles, u64 used, u64 total, TextPal pal)
{
    u32 totalPx = (u32)widthTiles * 8u;
    u32 filledPx = (total > 0) ? (u32)((used * (u64)totalPx) / total) : 0u;
    if (filledPx > totalPx)
        filledPx = totalPx;

    for (int i = 0; i < widthTiles; i++)
    {
        u32 startPx = (u32)i * 8u;
        u32 segFill = 0;
        if (filledPx >= startPx + 8u)
            segFill = 8u;
        else if (filledPx > startPx)
            segFill = filledPx - startPx;

        map[y * 32 + x + i] = (u16)((pal << 12) | (TILE_PROGRESS_BASE + segFill));
    }
}

static void formatByteSize(u64 bytes, char* out, size_t outSize)
{
    const u64 gib = 1024ULL * 1024ULL * 1024ULL;
    const u64 mib = 1024ULL * 1024ULL;
    if (bytes >= gib)
    {
        u32 tenths = (u32)((bytes * 10ULL) / gib);
        snprintf(out, outSize, "%lu.%luGB", (unsigned long)(tenths / 10u), (unsigned long)(tenths % 10u));
    }
    else
    {
        u32 mb = (u32)(bytes / mib);
        snprintf(out, outSize, "%luMB", (unsigned long)mb);
    }
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
        sDarkTheme = jsonGetBool(themeJson, "darkTheme", true);
        parsePrimaryColor(themeJson);

        if (sDarkTheme)
        {
            fallbackTop = RGB15(2 + sPrimaryR / 64, 2 + sPrimaryG / 64, 3 + sPrimaryB / 64);
            fallbackSub = RGB15(1 + sPrimaryR / 80, 1 + sPrimaryG / 80, 2 + sPrimaryB / 80);
        }
        else
        {
            fallbackTop = RGB15(26 + sPrimaryR / 64, 26 + sPrimaryG / 64, 27 + sPrimaryB / 64);
            fallbackSub = RGB15(25 + sPrimaryR / 64, 25 + sPrimaryG / 64, 26 + sPrimaryB / 64);
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

static void measureVolumeStats(const char* mountPoint, StorageStats& outStats)
{
    struct statvfs st;
    if (statvfs(mountPoint, &st) == 0 && st.f_blocks > 0)
    {
        u64 blockSize = (u64)(st.f_frsize ? st.f_frsize : st.f_bsize);
        outStats.totalBytes = (u64)st.f_blocks * blockSize;
        outStats.freeBytes  = (u64)st.f_bfree  * blockSize;
        if (outStats.freeBytes > outStats.totalBytes)
            outStats.freeBytes = outStats.totalBytes;
        outStats.usedBytes = outStats.totalBytes - outStats.freeBytes;
        outStats.usedPercent = (outStats.totalBytes > 0)
            ? (u32)((outStats.usedBytes * 100ULL) / outStats.totalBytes)
            : 0u;
        if (outStats.usedPercent > 100u)
            outStats.usedPercent = 100u;
        outStats.valid = true;
    }
}

static void initVideo()
{
    REG_POWERCNT = (1u << 0) | (1u << 1) | (1u << 9) | (1u << 15);
    REG_MASTER_BRIGHT = 0;
    REG_MASTER_BRIGHT_SUB = 0;

    mem_setVramAMapping(MEM_VRAM_AB_MAIN_BG_00000);
    mem_setVramCMapping(MEM_VRAM_C_SUB_BG_00000);

    REG_DISPCNT     = 5u | (1u << 8) | (1u << 11) | (1u << 16);
    REG_DISPCNT_SUB = 5u | (1u << 8) | (1u << 11) | (1u << 16);

    REG_BG0CNT     = 0x0200;
    REG_BG0CNT_SUB = 0x0200;
    REG_BG0HOFS = 0; REG_BG0VOFS = 0;
    REG_BG0HOFS_SUB = 0; REG_BG0VOFS_SUB = 0;

    REG_BG3CNT     = 0x4185;
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
    REG_MASTER_BRIGHT = 0;
    REG_MASTER_BRIGHT_SUB = 0;

    vu16* subMap = GFX_BG_SUB + BG_MAP_OFFSET_U16;
    drawTextPadded(subMap, 3, 17, 26, PAL_YELLOW, "Disconnecting USB...");

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

    REG_DISPCNT = 0;
    REG_DISPCNT_SUB = 0;

    mem_setVramAMapping(MEM_VRAM_AB_LCDC);
    mem_setVramBMapping(MEM_VRAM_AB_LCDC);
    mem_setVramCMapping(MEM_VRAM_C_LCDC);
    mem_setVramDMapping(MEM_VRAM_D_LCDC);

    pload_header7_t* hdr = (pload_header7_t*)sPicoLoader7Buf;
    hdr->dldiDriver = gDldiStub;
    hdr->bootDrive = PLOAD_BOOT_DRIVE_DLDI;
    memset(&hdr->loadParams, 0, sizeof(hdr->loadParams));
    strncpy(hdr->loadParams.romPath, "fat:/_picoboot.nds", sizeof(hdr->loadParams.romPath) - 1);
    if (hdr->apiVersion >= 2)
        memset(&hdr->v2, 0, sizeof(hdr->v2));
    if (hdr->apiVersion >= 3)
        hdr->v3.cheats = nullptr;

    wordCopyToVram((vu16*)0x06800000, (const u16*)sPicoLoader9Buf, (sPicoLoader9Size + 3u) & ~3u);
    wordCopyToVram((vu16*)0x06840000, (const u16*)sPicoLoader7Buf, (sPicoLoader7Size + 3u) & ~3u);

    DC_FlushRange(gDldiStub, 16 * 1024);
    DC_FlushAll();
    IC_InvalidateAll();

    *(vu32*)0x04000208 = 0;
    *(vu32*)0x04000210 = 0;
    *(vu32*)0x04000214 = ~0u;
    *(vu32*)0x040000BC = 0;
    *(vu32*)0x040000C8 = 0;
    *(vu32*)0x040000D4 = 0;
    *(vu32*)0x040000E0 = 0;

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
static void drawStoragePanel(int driveIndex)
{
    vu16* topMap = GFX_BG_MAIN + BG_MAP_OFFSET_U16;
    const StorageStats& st = sDriveStats[driveIndex];

    if (st.valid)
    {
        char line[32];
        char freeStr[12], totalStr[12], usedStr[12];
        formatByteSize(st.freeBytes,  freeStr,  sizeof(freeStr));
        formatByteSize(st.totalBytes, totalStr, sizeof(totalStr));
        formatByteSize(st.usedBytes,  usedStr,  sizeof(usedStr));

        TextPal barPal = PAL_GREEN;
        if (st.usedPercent >= 90u)
            barPal = PAL_RED;
        else if (st.usedPercent >= 75u)
            barPal = PAL_YELLOW;

        snprintf(line, sizeof(line), "Free: %s / %s", freeStr, totalStr);
        drawTextPadded(topMap, 3, 18, 26, PAL_TEXT, line);

        drawProgressBar(topMap, 3, 19, 26, st.usedBytes, st.totalBytes, barPal);

        snprintf(line, sizeof(line), "Used: %-7s   %3lu%% Full", usedStr, (unsigned long)st.usedPercent);
        drawTextPadded(topMap, 3, 20, 26, barPal, line);
    }
    else
    {
        drawTextPadded(topMap, 3, 18, 26, PAL_MUTED, "SD Storage: Unavailable");
        drawProgressBar(topMap, 3, 19, 26, 0, 1, PAL_MUTED);
        drawTextPadded(topMap, 3, 20, 26, PAL_MUTED, "");
    }
}

static void drawStaticHud(bool picoLoaderReady)
{
    vu16* topMap = GFX_BG_MAIN + BG_MAP_OFFSET_U16;
    vu16* subMap = GFX_BG_SUB + BG_MAP_OFFSET_U16;

    // Upper top-screen panel (rows 4..11)
    drawBox(topMap, 1, 4, 30, 8);

    // Lower top-screen panel (rows 15..21)
    drawBox(topMap, 2, 15, 28, 7);
    drawStoragePanel(0);

    // Bottom screen panel
    drawBox(subMap, 1, 11, 30, 9);
    drawText(subMap, 3, 12, PAL_ACCENT, "Controls & Tips");
    drawText(subMap, 3, 14, PAL_TEXT,   "Connect USB-C to host.");
    drawText(subMap, 3, 15, PAL_MUTED,  "Tap screen: Toggle display");

    if (sDsiSdAvailable)
        drawText(subMap, 3, 16, PAL_ACCENT, "[SELECT] Swap DSpico/3DS SD");
    else
        drawText(subMap, 3, 16, PAL_MUTED,  "3DS SD locked (DS mode)");

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
    const bool switching = (flags & USB_STATUS_SWITCHING) != 0 || sSwitchingUi;
    const int activeDrv  = (flags & USB_STATUS_DRIVE_DSI_SD) ? 1 : 0;
    const bool reading   = sReadHoldFrames > 0;
    const bool writing   = sWriteHoldFrames > 0;

    if (activeDrv != sDisplayedDrive)
    {
        sDisplayedDrive = activeDrv;
        drawStoragePanel(sDisplayedDrive);
    }

    if (switching)
        drawTextCentered(topMap, 2, 5, 28, PAL_YELLOW, "USB: SWITCHING DRIVE...");
    else if (mounted && !suspended)
        drawTextCentered(topMap, 2, 5, 28, PAL_GREEN, "USB: CONNECTED");
    else if (suspended)
        drawTextCentered(topMap, 2, 5, 28, PAL_YELLOW, "USB: SUSPENDED");
    else if (connected)
        drawTextCentered(topMap, 2, 5, 28, PAL_YELLOW, "USB: ENUMERATING...");
    else
        drawTextCentered(topMap, 2, 5, 28, PAL_MUTED, "USB: WAITING FOR HOST");

    drawTextCentered(topMap, 2, 6, 28, PAL_ACCENT,
                     (activeDrv == 1) ? "Drive: 3DS / DSi SD" : "Drive: DSpico SD");

    static const char spinner[4] = {'|', '/', '-', '\\'};
    char actLine[32];
    if (writing)
    {
        snprintf(actLine, sizeof(actLine), "SD: WRITING! DO NOT UNPLUG %c", spinner[(frame >> 2) & 3]);
        drawTextCentered(topMap, 2, 7, 28, PAL_RED, actLine);
    }
    else if (reading)
    {
        snprintf(actLine, sizeof(actLine), "SD: READING... %c", spinner[(frame >> 2) & 3]);
        drawTextCentered(topMap, 2, 7, 28, PAL_YELLOW, actLine);
    }
    else
    {
        drawTextCentered(topMap, 2, 7, 28, PAL_TEXT, "SD: IDLE");
    }

    char stats[32];
    snprintf(stats, sizeof(stats), "Read: %lu KB", (unsigned long)sReadKB);
    drawTextCentered(topMap, 2, 9, 28, PAL_ACCENT, stats);
    snprintf(stats, sizeof(stats), "Write: %lu KB", (unsigned long)sWriteKB);
    drawTextCentered(topMap, 2, 10, 28, PAL_ACCENT, stats);

    // Live device clock inside row 16 of the lower box
    char clockLine[32];
    if (sHasTime)
    {
        u32 packed = sPackedTime;
        u32 hour24 = (packed >> 16) & 0x1Fu;
        u32 min    = (packed >> 8)  & 0x3Fu;
        u32 sec    = packed         & 0x3Fu;
        bool isPm  = (hour24 >= 12u);
        u32 hour12 = hour24 % 12u;
        if (hour12 == 0u) hour12 = 12u;

        snprintf(clockLine, sizeof(clockLine), "Time: %02lu:%02lu:%02lu %s",
                 (unsigned long)hour12,
                 (unsigned long)min,
                 (unsigned long)sec,
                 isPm ? "PM" : "AM");
        drawTextPadded(topMap, 3, 16, 26, PAL_ACCENT, clockLine);
    }
    else
    {
        drawTextPadded(topMap, 3, 16, 26, PAL_MUTED, "Time: --:--:-- --");
    }
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
    rtos_createEvent(&sDsiSdEvent);

    ipc_setChannelHandler(IPC_CHANNEL_DSI_SD, dsiSdIpcHandler, nullptr);

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

        initTextPalettes(BG_PAL_MAIN);
        initTextPalettes(BG_PAL_SUB);

        measureVolumeStats("fat:/", sDriveStats[0]);

        sPicoLoader9Buf = readWholeFile("fat:/_pico/picoLoader9.bin", &sPicoLoader9Size);
        sPicoLoader7Buf = readWholeFile("fat:/_pico/picoLoader7.bin", &sPicoLoader7Size);
        picoLoaderReady = (sPicoLoader9Buf != nullptr && sPicoLoader7Buf != nullptr);

        fatUnmount("fat");
    }
    else
    {
        fillSolidBackground(GFX_BG_MAIN + (BG_BITMAP_OFFSET_BYTES / 2), RGB15(10, 2, 2));
        fillSolidBackground(GFX_BG_SUB  + (BG_BITMAP_OFFSET_BYTES / 2), RGB15(8, 2, 2));
    }

    // Also measure 3DS/DSi SD storage stats at boot if accessible in DSi mode
    if (fatMountSimple("sd", &sDsiSdIpcDisc))
    {
        sDsiSdAvailable = true;
        measureVolumeStats("sd:/", sDriveStats[1]);
        fatUnmount("sd");
    }

    drawStaticHud(picoLoaderReady);

    ipc_setChannelHandler(IPC_CHANNEL_SOUND, usbStatusIpcHandler, nullptr);
    ipc_setChannelHandler(IPC_CHANNEL_LOADER, loaderIpcHandler, nullptr);

    // Signal ARM7 to initialize TinyUSB Mass Storage
    ipc_setArm9SyncBits(6);

    rtos_setIrqFunc(RTOS_IRQ_VBLANK, vblankIrq);
    rtos_enableIrqMask(RTOS_IRQ_VBLANK);
    gfx_setVBlankIrqEnabled(true);

    bool startPreviouslyPressed  = ((REG_KEYINPUT & KEY_START_BIT) == 0);
    bool selectPreviouslyPressed = ((REG_KEYINPUT & KEY_SELECT_BIT) == 0);
    u32 frame = 0;

    while (true)
    {
        rtos_waitEvent(&sVblankEvent, true, true);
        frame++;

        if (sReadHoldFrames > 0)
            sReadHoldFrames--;
        if (sWriteHoldFrames > 0)
            sWriteHoldFrames--;

        bool screensOff = (sUsbStatusFlags & USB_STATUS_SCREENS_OFF) != 0;
        if (screensOff != sScreensOff)
        {
            sScreensOff = screensOff;
            u16 bright = screensOff ? ((2u << 14) | 16u) : 0u;
            REG_MASTER_BRIGHT = bright;
            REG_MASTER_BRIGHT_SUB = bright;
        }

        if (sDriveSwitchDone)
        {
            sDriveSwitchDone = false;
            sSwitchingUi = false;
        }

        updateDynamicHud(frame);

        bool selectDown = ((REG_KEYINPUT & KEY_SELECT_BIT) == 0);
        if (selectDown && !selectPreviouslyPressed && sDsiSdAvailable && !sSwitchingUi)
        {
            if (sWriteHoldFrames == 0)
            {
                sSwitchingUi = true;
                sDriveSwitchDone = false;
                u32 cmd = (sDisplayedDrive == 0) ? LOADER_CMD_SWITCH_TO_DSI_SD : LOADER_CMD_SWITCH_TO_DSPICO;
                ipc_sendFifoMessage(IPC_CHANNEL_LOADER, cmd);
            }
        }
        selectPreviouslyPressed = selectDown;

        bool startDown = ((REG_KEYINPUT & KEY_START_BIT) == 0);
        if (startDown && !startPreviouslyPressed && picoLoaderReady && !sSwitchingUi)
        {
            returnToPicoLauncher();
        }
        startPreviouslyPressed = startDown;
    }

    return 0;
}
