#pragma once

// Sent from ARM7 -> ARM9 on IPC_CHANNEL_SOUND (26-bit payload max)
#define USB_IPC_TAG_SHIFT       24
#define USB_IPC_TAG_STATUS      0
#define USB_IPC_TAG_READ_KB     1
#define USB_IPC_TAG_WRITE_KB    2
#define USB_IPC_TAG_TIME        3
#define USB_IPC_VAL_MASK        0x00FFFFFFu

#define USB_STATUS_MOUNTED      (1u << 0)
#define USB_STATUS_SUSPENDED    (1u << 1)
#define USB_STATUS_CONNECTED    (1u << 2)
#define USB_STATUS_READING      (1u << 3)
#define USB_STATUS_WRITING      (1u << 4)
#define USB_STATUS_SCREENS_OFF  (1u << 5)
#define USB_STATUS_DRIVE_DSI_SD (1u << 6)
#define USB_STATUS_SWITCHING    (1u << 7)

// Sent on IPC_CHANNEL_LOADER
#define LOADER_CMD_DISCONNECT_USB   1u
#define LOADER_CMD_BOOT_PICOLOADER  2u
#define LOADER_CMD_SWITCH_TO_DSPICO 3u
#define LOADER_CMD_SWITCH_TO_DSI_SD 4u
#define LOADER_RESP_DISCONNECTED    1u
#define LOADER_RESP_DRIVE_SWITCHED  2u
