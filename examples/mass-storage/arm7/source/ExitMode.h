#pragma once

/// @brief Enum representing the exit mode of launcher.
enum class ExitMode
{
    /// @brief Reset the system (DSi mode only).
    Reset,
    /// @brief Power off the system.
    PowerOff,
    /// @brief Boot Pico Loader in VRAM.
    PicoLoader
};
