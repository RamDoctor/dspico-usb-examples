#include "common.h"
#include <nds/disc_io.h>
#include <nds/touch.h>
#include <nds/arm7/touch.h>
#include <libtwl/rtos/rtosIrq.h>
#include <libtwl/rtos/rtosThread.h>
#include <libtwl/rtos/rtosEvent.h>
#include <libtwl/sound/soundChannel.h>
#include <libtwl/timer/timer.h>
#include <libtwl/sound/sound.h>
#include <libtwl/ipc/ipcSync.h>
#include <libtwl/ipc/ipcFifoSystem.h>
#include <libtwl/sys/sysPower.h>
#include <libtwl/sio/sioRtc.h>
#include <libtwl/sio/sio.h>
#include <libtwl/gfx/gfxStatus.h>
#include <libtwl/mem/memSwap.h>
#include <libtwl/i2c/i2cMcu.h>
#include <libtwl/spi/spiPmic.h>
#include "ipc/ThreadIpcService.h"
#include "ipcServices/DldiIpcService.h"
#include "dldiIpcCommand.h"
#include "ipcChannels.h"
#include "usbStatusIpc.h"
#include "mmc/sdmmc.h"
#include "mmc/tmio.h"
#include "ExitMode.h"
#include "Arm7State.h"
#include "tusb.h"
#include "usb_descriptors.h"

static DldiIpcService sDldiIpcService;

rtos_mutex_t gCardMutex;

static rtos_event_t sVBlankEvent;
static ExitMode sExitMode;
static Arm7State sState;
static volatile u8 sMcuIrqFlag = false;

static volatile u8 sReadPulse = false;
static volatile u8 sWritePulse = false;
static volatile u32 sReadSectors = 0;
static volatile u32 sWriteSectors = 0;
static u32 sLastSentStatus = 0xFFFFFFFFu;
static bool sUsbStarted = false;

static u32 sRtcStatus1 = 0;
static u32 sLastSentTime = 0xFFFFFFFFu;
static int sRtcPollFrames = 0;

static bool sScreensEnabled = true;
static int sTouchCooldown = 0;
static int sTouchUpFrames = 0;

static volatile bool sDisconnectRequested = false;
static int sDisconnectWaitFrames = 0;

static bool sDsiSdAvailable = false;
static u32 sDspicoBlockCount = 0;
static u32 sDsiSdBlockCount = 0;
static volatile u8 sActiveDrive = 0;    // 0 = DSpico SD, 1 = 3DS/DSi SD
static volatile u8 sRequestedDrive = 0;
static volatile bool sSwitchRequested = false;
static bool sSwitchingDrive = false;
static int sSwitchWaitFrames = 0;

static u8 sSector0Buffer[512] alignas(4);

static rtos_thread_t sUsbThread;
static u32 sUsbThreadStack[512];

extern FN_MEDIUM_READSECTORS _DLDI_readSectors_ptr;
extern FN_MEDIUM_WRITESECTORS _DLDI_writeSectors_ptr;

// Exposes DSi/3DS internal SD to ARM9 at boot for statvfs before USB starts
class DsiSdBootIpcService : public ThreadIpcService
{
    u32 _threadStack[128];

public:
    DsiSdBootIpcService()
        : ThreadIpcService(IPC_CHANNEL_DSI_SD, 6, _threadStack, sizeof(_threadStack)) { }

    void HandleMessage(u32 data) override
    {
        auto cmd = reinterpret_cast<const dldi_ipc_cmd_t*>(data << 2);
        bool ok = false;
        switch (cmd->cmd)
        {
            case DLDI_IPC_CMD_SETUP:
                ok = sDsiSdAvailable;
                break;

            case DLDI_IPC_CMD_READ_SECTORS:
                if (sDsiSdAvailable)
                    ok = (pico_SDMMC_readSectors(SDMMC_DEV_CARD, cmd->sector, cmd->buffer, cmd->count) == SDMMC_ERR_NONE);
                break;

            case DLDI_IPC_CMD_WRITE_SECTORS:
                if (sDsiSdAvailable)
                    ok = (pico_SDMMC_writeSectors(SDMMC_DEV_CARD, cmd->sector, cmd->buffer, cmd->count) == SDMMC_ERR_NONE);
                break;
        }
        SendResponseMessage(ok ? 1u : 0u);
    }
};

static DsiSdBootIpcService sDsiSdBootIpcService;

static void vblankIrq(u32 irqMask)
{
    (void)irqMask;
    rtos_signalEvent(&sVBlankEvent);
}

static void mcuIrq(u32 irq2Mask)
{
    (void)irq2Mask;
    sMcuIrqFlag = true;
}

static void loaderIpcHandler(u32 channel, u32 data, void* arg)
{
    (void)channel;
    (void)arg;
    if (data == LOADER_CMD_DISCONNECT_USB)
    {
        sDisconnectRequested = true;
    }
    else if (data == LOADER_CMD_BOOT_PICOLOADER)
    {
        sExitMode = ExitMode::PicoLoader;
        sState = Arm7State::ExitRequested;
    }
    else if (data == LOADER_CMD_SWITCH_TO_DSPICO)
    {
        sRequestedDrive = 0;
        sSwitchRequested = true;
    }
    else if (data == LOADER_CMD_SWITCH_TO_DSI_SD)
    {
        if (sDsiSdAvailable)
        {
            sRequestedDrive = 1;
            sSwitchRequested = true;
        }
    }
}

static void checkMcuIrq(void)
{
    if (isDSiMode())
    {
        if (mem_swapByte(false, &sMcuIrqFlag))
        {
            u32 irqMask = mcu_getIrqMask();
            if (irqMask & MCU_IRQ_RESET)
            {
                sExitMode = ExitMode::Reset;
                sState = Arm7State::ExitRequested;
            }
            else if (irqMask & MCU_IRQ_POWER_OFF)
            {
                sExitMode = ExitMode::PowerOff;
                sState = Arm7State::ExitRequested;
            }
        }
    }
}

static void checkTouchToggle()
{
    if (!sUsbStarted)
        return;

    if (sTouchCooldown > 0)
        sTouchCooldown--;

    bool penDown = touchPenDown();
    if (penDown)
    {
        touchPosition pos;
        touchReadXY(&pos);

        if (sTouchCooldown == 0 && sTouchUpFrames >= 3)
        {
            sScreensEnabled = !sScreensEnabled;
            sTouchCooldown = 18;
        }
        sTouchUpFrames = 0;
    }
    else
    {
        if (sTouchUpFrames < 10)
            sTouchUpFrames++;
    }
}

static void initializeVBlankIrq()
{
    rtos_createEvent(&sVBlankEvent);
    rtos_setIrqFunc(RTOS_IRQ_VBLANK, vblankIrq);
    rtos_enableIrqMask(RTOS_IRQ_VBLANK);
    gfx_setVBlankIrqEnabled(true);
}

static void usbThreadMain(void* arg)
{
    (void)arg;
    while (true)
    {
        tud_task();
    }
}

// Based on https://github.com/asiekierka/nrio-usb-disk/blob/main/source/msc.c msc_find_block_count by Asie
static u32 findDspicoBlockCount()
{
    rtos_lockMutex(&gCardMutex);
    _DLDI_readSectors_ptr(0, 1, sSector0Buffer);
    rtos_unlockMutex(&gCardMutex);

    u16 footer = *(u16*)(sSector0Buffer + 510);
    if (footer == 0xAA55)
    {
        u8 bootOpcode = sSector0Buffer[0];
        if (bootOpcode == 0xEB || bootOpcode == 0xE9 || bootOpcode == 0xE8)
        {
            if (!memcmp(sSector0Buffer + 54, "FAT", 3) || !memcmp(sSector0Buffer + 82, "FAT32   ", 8))
            {
                u32 totalSectors = *(u32*)(sSector0Buffer + 32);
                if (totalSectors < 0x10000)
                {
                    totalSectors = sSector0Buffer[19] | (sSector0Buffer[20] << 8);
                }
                return totalSectors;
            }
        }

        u32 blockCount = 0;
        for (u32 tableEntry = 0x1BE; tableEntry < 0x1FE; tableEntry += 16)
        {
            u32 pStart = *(u16*)(sSector0Buffer + tableEntry + 8) | (*(u16*)(sSector0Buffer + tableEntry + 10) << 16);
            u32 pCount = *(u16*)(sSector0Buffer + tableEntry + 12) | (*(u16*)(sSector0Buffer + tableEntry + 14) << 16);
            u32 pEnd = pStart + pCount;
            if (pEnd > blockCount)
            {
                blockCount = pEnd;
            }
        }

        return blockCount;
    }

    return 0;
}

static inline u32 bcdToBin(u8 bcd)
{
    return (u32)(((bcd >> 4) * 10) + (bcd & 0x0Fu));
}

static void sendRtcTimeUpdate(bool force)
{
    if (!force && ++sRtcPollFrames < 15)
        return;
    sRtcPollFrames = 0;

    rtc_time_t t;
    rtc_readTime(&t);

    u32 hour = bcdToBin((u8)t.hour);
    u32 min  = bcdToBin(t.minute);
    u32 sec  = bcdToBin(t.second);

    if (!(sRtcStatus1 & RTC_STATUS1_24HOUR))
    {
        hour = (hour % 12u) + ((t.amPm == RTC_PM) ? 12u : 0u);
    }

    u32 packed = ((hour & 0x1Fu) << 16) | ((min & 0x3Fu) << 8) | (sec & 0x3Fu);
    if (force || packed != sLastSentTime)
    {
        sLastSentTime = packed;
        ipc_sendFifoMessage(IPC_CHANNEL_SOUND, (USB_IPC_TAG_TIME << USB_IPC_TAG_SHIFT) | (packed & USB_IPC_VAL_MASK));
    }
}

static void initializeArm7()
{
    rtos_initIrq();
    rtos_startMainThread();
    ipc_initFifoSystem();

    rtos_createMutex(&gCardMutex);

    dmaFillWords(0, (void*)0x04000400, 0x100);

    pmic_setAmplifierEnable(true);
    sys_setSoundPower(true);

    readUserSettings();
    pmic_setPowerLedBlink(PMIC_CONTROL_POWER_LED_BLINK_NONE);

    sio_setGpioSiIrq(false);
    sio_setGpioMode(RCNT0_L_MODE_GPIO);

    touchInit();
    rtc_init();
    sRtcStatus1 = rtc_readStatus1();

    initializeVBlankIrq();

    if (isDSiMode())
    {
        rtos_setIrq2Func(RTOS_IRQ2_MCU, mcuIrq);
        rtos_enableIrq2Mask(RTOS_IRQ2_MCU);

        TMIO_init();
        if (pico_SDMMC_init(SDMMC_DEV_CARD) == SDMMC_ERR_NONE)
        {
            sDsiSdBlockCount = pico_SDMMC_getSectors(SDMMC_DEV_CARD);
            sDsiSdAvailable = (sDsiSdBlockCount > 0);
        }
    }

    sDldiIpcService.Start();
    sDsiSdBootIpcService.Start();
    ipc_setChannelHandler(IPC_CHANNEL_LOADER, loaderIpcHandler, nullptr);

    snd_setMasterVolume(127);
    snd_setMasterEnable(true);

    ipc_setArm7SyncBits(7);

    while (ipc_getArm9SyncBits() != 6)
    {
        rtos_waitEvent(&sVBlankEvent, true, true);
    }

    sDspicoBlockCount = findDspicoBlockCount();

    tusb_rhport_init_t dev_init =
    {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    tusb_init(0, &dev_init);
    sUsbStarted = true;

    sendRtcTimeUpdate(true);

    rtos_createThread(&sUsbThread, 3, usbThreadMain, NULL, sUsbThreadStack, sizeof(sUsbThreadStack));
    rtos_wakeupThread(&sUsbThread);
}

static void sendUsbStatusUpdate()
{
    if (!sUsbStarted)
        return;

    bool didRead = mem_swapByte(false, &sReadPulse);
    bool didWrite = mem_swapByte(false, &sWritePulse);

    u32 status = 0;
    if (tud_mounted())
        status |= USB_STATUS_MOUNTED;
    if (tud_suspended())
        status |= USB_STATUS_SUSPENDED;
    if (tud_connected())
        status |= USB_STATUS_CONNECTED;
    if (didRead)
        status |= USB_STATUS_READING;
    if (didWrite)
        status |= USB_STATUS_WRITING;
    if (!sScreensEnabled)
        status |= USB_STATUS_SCREENS_OFF;
    if (sActiveDrive == 1)
        status |= USB_STATUS_DRIVE_DSI_SD;
    if (sSwitchingDrive)
        status |= USB_STATUS_SWITCHING;

    if (status != sLastSentStatus)
    {
        sLastSentStatus = status;
        ipc_sendFifoMessage(IPC_CHANNEL_SOUND, (USB_IPC_TAG_STATUS << USB_IPC_TAG_SHIFT) | (status & USB_IPC_VAL_MASK));
    }

    if (didRead)
    {
        u32 readKb = (sReadSectors >> 1) & USB_IPC_VAL_MASK;
        ipc_sendFifoMessage(IPC_CHANNEL_SOUND, (USB_IPC_TAG_READ_KB << USB_IPC_TAG_SHIFT) | readKb);
    }

    if (didWrite)
    {
        u32 writeKb = (sWriteSectors >> 1) & USB_IPC_VAL_MASK;
        ipc_sendFifoMessage(IPC_CHANNEL_SOUND, (USB_IPC_TAG_WRITE_KB << USB_IPC_TAG_SHIFT) | writeKb);
    }

    sendRtcTimeUpdate(false);
}

static void updateArm7IdleState()
{
    checkMcuIrq();
    checkTouchToggle();
    sendUsbStatusUpdate();

    if (sSwitchRequested && !sSwitchingDrive)
    {
        sSwitchRequested = false;
        sSwitchingDrive = true;
        if (sUsbStarted)
            tud_disconnect();
        sSwitchWaitFrames = 30;
    }

    if (sSwitchingDrive && sSwitchWaitFrames > 0)
    {
        if (--sSwitchWaitFrames == 0)
        {
            sActiveDrive = sRequestedDrive;
            sReadSectors = 0;
            sWriteSectors = 0;
            sReadPulse = false;
            sWritePulse = false;
            sSwitchingDrive = false;
            if (sUsbStarted)
                tud_connect();
            ipc_sendFifoMessage(IPC_CHANNEL_SOUND, (USB_IPC_TAG_READ_KB << USB_IPC_TAG_SHIFT) | 0u);
            ipc_sendFifoMessage(IPC_CHANNEL_SOUND, (USB_IPC_TAG_WRITE_KB << USB_IPC_TAG_SHIFT) | 0u);
            ipc_sendFifoMessage(IPC_CHANNEL_LOADER, LOADER_RESP_DRIVE_SWITCHED);
        }
    }

    if (sDisconnectRequested)
    {
        sDisconnectRequested = false;
        sScreensEnabled = true;
        if (sUsbStarted)
            tud_disconnect();
        snd_setMasterVolume(0);
        sDisconnectWaitFrames = 15;
    }

    if (sDisconnectWaitFrames > 0)
    {
        if (--sDisconnectWaitFrames == 0)
        {
            ipc_sendFifoMessage(IPC_CHANNEL_LOADER, LOADER_RESP_DISCONNECTED);
        }
    }

    if (sState == Arm7State::ExitRequested)
    {
        snd_setMasterVolume(0);
    }
}

static void performExit(ExitMode exitMode)
{
    switch (exitMode)
    {
        case ExitMode::Reset:
        {
            mcu_setWarmBootFlag(true);
            mcu_hardReset();
            break;
        }
        case ExitMode::PowerOff:
        {
            pmic_shutdown();
            break;
        }
        case ExitMode::PicoLoader:
        {
            if (isDSiMode())
            {
                if (sDsiSdAvailable)
                    pico_SDMMC_deinit(SDMMC_DEV_CARD);
                TMIO_deinit();
            }

            snd_setMasterVolume(0);
            snd_setMasterEnable(false);
            sys_setSoundPower(false);
            dmaFillWords(0, (void*)0x04000400, 0x100);

            *(vu32*)0x04000208 = 0;
            *(vu32*)0x04000210 = 0;
            *(vu32*)0x04000214 = ~0u;
            if (isDSiMode())
            {
                *(vu32*)0x04000218 = 0;
                *(vu32*)0x0400021C = ~0u;
            }

            *(vu32*)0x040000BC = 0;
            *(vu32*)0x040000C8 = 0;
            *(vu32*)0x040000D4 = 0;
            *(vu32*)0x040000E0 = 0;

            void* entryPoint = *(void* const*)0x06000000;
            ipc_setArm7SyncBits(5);
            ((void (*)(void))entryPoint)();
            break;
        }
    }

    while (true);
}

static void updateArm7ExitRequestedState()
{
    performExit(sExitMode);
}

static void updateArm7()
{
    switch (sState)
    {
        case Arm7State::Idle:
        {
            updateArm7IdleState();
            break;
        }
        case Arm7State::ExitRequested:
        {
            updateArm7ExitRequestedState();
            break;
        }
    }
}

int main()
{
    sState = Arm7State::Idle;
    initializeArm7();

    while (true)
    {
        rtos_waitEvent(&sVBlankEvent, true, true);
        updateArm7();
    }

    return 0;
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;

    const char vid[] = "DSpico";
    const char* pid = (sActiveDrive == 1) ? "3DS/DSi SD" : "Mass Storage";
    const char rev[] = "1.0";

    memset(vendor_id, ' ', 8);
    memset(product_id, ' ', 16);
    memset(product_rev, ' ', 4);
    memcpy(vendor_id  , vid, strlen(vid));
    memcpy(product_id , pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    if (sSwitchingDrive)
    {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return false;
    }
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
    (void) lun;

    *block_count = (sActiveDrive == 1) ? sDsiSdBlockCount : sDspicoBlockCount;
    *block_size  = 512;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void) lun;
    (void) power_condition;
    (void) start;
    (void) load_eject;
    return true;
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
    (void) lun;

    if (sSwitchingDrive || offset != 0 || (bufsize % 512) != 0)
    {
        return -1;
    }

    u32 count = bufsize / 512;
    sReadPulse = true;
    sReadSectors += count;

    if (sActiveDrive == 1)
    {
        if (!sDsiSdAvailable)
            return -1;
        if (pico_SDMMC_readSectors(SDMMC_DEV_CARD, lba, buffer, count) != SDMMC_ERR_NONE)
            return -1;
    }
    else
    {
        rtos_lockMutex(&gCardMutex);
        _DLDI_readSectors_ptr(lba, count, buffer);
        rtos_unlockMutex(&gCardMutex);
    }

    return (int32_t)bufsize;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void) lun;
    return true;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
    (void) lun;

    if (sSwitchingDrive || offset != 0 || (bufsize % 512) != 0)
    {
        return -1;
    }

    u32 count = bufsize / 512;
    sWritePulse = true;
    sWriteSectors += count;

    if (sActiveDrive == 1)
    {
        if (!sDsiSdAvailable)
            return -1;
        if (pico_SDMMC_writeSectors(SDMMC_DEV_CARD, lba, buffer, count) != SDMMC_ERR_NONE)
            return -1;
    }
    else
    {
        rtos_lockMutex(&gCardMutex);
        _DLDI_writeSectors_ptr(lba, count, buffer);
        rtos_unlockMutex(&gCardMutex);
    }

    return (int32_t)bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize)
{
    (void)scsi_cmd;
    (void)buffer;
    (void)bufsize;

    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}
