#pragma once
#include <nds/ndstypes.h>

#define PLOAD_API_VERSION 3

typedef enum
{
    PLOAD_BOOT_DRIVE_DLDI = 0,
    PLOAD_BOOT_DRIVE_DSI_SD = 1,
    PLOAD_BOOT_DRIVE_AGB_SEMIHOSTING = 2,
    PLOAD_BOOT_DRIVE_MULTIBOOT_FLAG = 1u << 15
} PicoLoaderBootDrive;

typedef struct
{
    char romPath[256];
    char savePath[256];
    u32 argumentsLength;
    char arguments[256];
} pload_params_t;

typedef struct
{
    char launcherPath[256];
} pload_header7_v2_t;

typedef struct
{
    u32 a;
    u32 b;
} pload_cheat_opcode_t;

typedef struct
{
    u32 length;
    pload_cheat_opcode_t opcodes[1];
} pload_cheat_t;

typedef struct
{
    u32 length;
    u32 numberOfCheats;
    pload_cheat_t firstCheat;
} pload_cheats_t;

typedef struct
{
    const pload_cheats_t* cheats;
} pload_header7_v3_t;

typedef struct
{
    void* const entryPoint;
    void* dldiDriver;
    u16 bootDrive;
    const u16 apiVersion;
    pload_params_t loadParams;
    pload_header7_v2_t v2;
    pload_header7_v3_t v3;
} pload_header7_t;
