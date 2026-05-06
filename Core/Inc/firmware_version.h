/***************************************************************************
 * firmware_version.h
 * created by Sebastian Forenza 2026
 *
 * Single source of truth for the firmware version string.
 * Format: V<MAJOR>.<MAIN>.<V2>
 *   MAJOR — manual bump (architectural milestones)
 *   MAIN  — incremented on every push to the `main` branch
 *   V2    — incremented on every push to the `V2` branch
 *
 * Bump rules and authority live in CLAUDE.md (section "Firmware Version").
 ***************************************************************************/

#ifndef FIRMWARE_VERSION_H
#define FIRMWARE_VERSION_H

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MAIN     11
#define FW_VERSION_V2       25

#define FW_STRINGIFY_(x)    #x
#define FW_STRINGIFY(x)     FW_STRINGIFY_(x)

#define FW_VERSION_STRING   "V" FW_STRINGIFY(FW_VERSION_MAJOR) "." \
                                FW_STRINGIFY(FW_VERSION_MAIN)  "." \
                                FW_STRINGIFY(FW_VERSION_V2)

#endif /* FIRMWARE_VERSION_H */
