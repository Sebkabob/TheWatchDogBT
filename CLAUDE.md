# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ONLY EDIT CODE WITHIN THE USER EDITABLE SECTIONS!!!

## Project Overview

WatchDogBT is embedded firmware for a Bluetooth Low Energy asset-tracking/alarm device built on the **STM32WB05KZV6TR** (Cortex-M0+, STM32WB0 family). The device uses a LIS2DUX12 accelerometer with an on-chip Machine Learning Core (MLC) for motion classification, a BQ25186 battery charger (PG/STAT lines on PB4/PA11), a BQ27427 fuel gauge (I2C, see `Drivers/BQ27427/`), an M24C08 EEPROM (see `Drivers/M24C08/`), RGB LEDs, and a magnetic buzzer.

## Build System

This is an **STM32CubeIDE** project. There is no command-line Makefile. Build and flash using STM32CubeIDE (`WatchDogBT.launch` / `WatchDogBT.cfg`). The `.ioc` file is the CubeMX configuration — regenerating from it overwrites everything outside the `/* USER CODE BEGIN/END */` blocks (those are preserved by CubeMX).

## Commenting Style

All user-created `.c` / `.h` files follow a uniform comment style. Apply it to any new file you author and respect it when editing existing ones.

### File header

Every user-created source file begins with:

```c
/***************************************************************************
 * filename.c
 * created by Sebastian Forenza 2026
 *
 * <One-line summary of what this file does>
 *
 * <Optional: pin map, state-machine sketch, hardware quirks — only if
 *  genuinely useful. Keep it tight, no paragraphs, no change-logs, no
 *  TODOs. That stuff belongs in git, not the header.>
 ***************************************************************************/
```

For CubeMX-generated files (`lockservice_app.{c,h}`, etc.) the ST license header is preserved verbatim and the user header is **not** added on top — clean only inside the `USER CODE` blocks.

### Function header

Non-trivial functions get a banner block directly above them:

```c
/***************************************************************************
 * FUNCTION_NAME — short tagline describing what it does
 *   <Optional 1–4 lines of detail: parameters, return values, side
 *    effects, edge cases, hardware behaviour — only if not obvious from
 *    the code. Don't restate the signature.>
 ***************************************************************************/
```

Required for: anything exposed in a header, ISRs, state-transition functions, hardware-access wrappers, anything with non-obvious behaviour. Skip for: tiny one-liners, simple getters/setters, obvious wrappers. If the description runs >5 lines, the function probably needs to be split.

### Inline comments — cut aggressively

Delete:

- Restating the code: `i++; // increment i`
- Obvious operations: `GPIO_SetPin(LED); // turn on LED`
- Section banners between every function (the function header itself is enough separation)
- Decorative `=====` / `*****` dividers
- Commented-out code (unless there's a clear reason to keep it, in which case add a one-line note)
- Stale TODOs / FIXMEs

Keep inline `//` comments only when they explain **why** — silicon errata, ordering requirements, units on magic numbers, hardware quirks, non-obvious workarounds. Wrap at ~80 columns.

### What not to touch

- Don't change code logic, only comments.
- Don't reformat code, rename variables, or restructure functions during a comment pass.
- Preserve any license headers / third-party attribution exactly as-is.

## Architecture

### State Machine (`Core/Src/state_machine.c`)

The central control loop. `StateMachine_Run()` is called every iteration of `main()`. States:

```
DISCONNECTED_IDLE → (BLE connect) → CONNECTED_IDLE
CONNECTED_IDLE    → (armed)       → STABILIZING
STABILIZING       → (3s still)    → LOCKED
LOCKED            → (motion)      → ALARM_ACTIVE
ALARM_ACTIVE      → (melody done + no motion) → LOCKED
```

`StateMachine_ChangeState()` automatically sets/clears the ARMED bit when entering STABILIZING/LOCKED/ALARM_ACTIVE vs. any other state, then pushes a status notification.

The `deviceState` byte packs all user-configurable settings:

| Bits | Field | Values |
|------|-------|--------|
| 0 | ARMED | 0/1 |
| 2:1 | ALARM_TYPE | 0=none, 1=calm, 2=normal, 3=loud |
| 4:3 | SENSITIVITY | 0=low, 1=medium, 2=high |
| 5 | LIGHTS | 0/1 |
| 6 | LOGGING | 0/1 |
| 7 | SILENCE | 0/1 |

`deviceInfo` bit 0 is `HIGH_PERF`: when set, BLE status updates run at 50 Hz (20 ms) instead of 2 Hz (500 ms).

A short **motion-grace window** is started on every BLE connect (`StateMachine_StartMotionGrace(2000)`): RestoreAll reloads the UCF, INT1 glitches, and the user is invariably handling the device while pairing — without the grace window every connect would fire the alarm. While active, `LOCKED` drains MLC/FSM events and suppresses transitions to `ALARM_ACTIVE`.

### BLE Layer (`STM32_BLE/App/`)

Custom **LockService** (16-bit UUID `0x183E`) GATT service with three characteristics:
- `APPTOWD` (write) — iOS → device commands. First byte of the inner payload (after the 4-byte loyalty token) = opcode; remaining bytes are payload. Settings writes (no opcode match) also carry a 6-byte trailing timestamp consumed by `UpdateBootTimeFromiOS()`.
- `DEVICESTATUS` (notify) — **16-byte** payload built by `LOCKSERVICE_Devicestatus_SendNotification()`. Pushed at 50 Hz when `HIGH_PERF` is set, 2 Hz otherwise (gated by `PowerMgmt_IsLowPower()`). Bytes 14..15 carry the low 2 bytes of the BD address (LE) — used by iOS as the user-visible "WatchDog #" identifier.
- `BATTERYDIAG` (notify) — 51-byte packed gauge telemetry payload (v11), attached dynamically at init in `lockservice.c`. Sent every ~1 s from `main()`'s battery tick.

iOS opcodes (`lockservice_app.h`) — these run **after** the 4-byte loyalty token has been stripped:

| Opcode | Name | Payload |
|--------|------|---------|
| `0xF0` | `CMD_REQUEST_LOG_COUNT` | none — kicks off log transfer |
| `0xF1` | `CMD_REQUEST_EVENT` | `uint16_t` BE index of event to fetch |
| `0xF2` | `CMD_CLEAR_LOG` | none |
| `0xF3` | `CMD_ACK_EVENT` | none |
| `0xFA` | `CMD_FIND_MY_DEVICE` | byte[1] bit 0 = start |
| `0xFB` | `CMD_RESET_DEVICE` | none — calls `NVIC_SystemReset()` |
| `0xFC` | `CMD_DRAIN_MODE` | byte[1] bit 0: 1=start drain, 0=stop |

Anything not matching an opcode is interpreted as a settings write: `cmd_data[0] → deviceState`, `cmd_data[1] → deviceInfo`, then a forced status notification.

`app_ble.c` handles GAP/GATT stack init and connection events; `lockservice.c` is the auto-generated GATT server (with hand-added BATTERYDIAG inside USER CODE blocks); `lockservice_app.c` contains all application logic for processing writes and sending notifications. `g_bd_address[6]` is published in `app_ble.c` and `bd_address_override` in `main.c` controls whether the code-defined BD address overwrites EEPROM at boot.

Advertising uses `HCI_ADV_FILTER_ACCEPT_LIST_CONNECT` — only bonded devices can connect.

On connect, `PowerMgmt_RestoreAll()` is called, a 2-second motion-grace window is armed, and any pending logged events trigger an unsolicited `LOCKSERVICE_SendEventCount()` so iOS knows to drain them.

### Accelerometer (`Core/Src/accelerometer.c`, `Core/Src/lis2dux12_app.c`)

LIS2DUX12 communicates over I2C1 (`hi2c1`). A pre-built UCF asset is loaded at `LIS2DUX12_Init()` to configure the MLC and FSM. INT1 is on **PB15** (EXTI rising edge) — the ISR only sets `motion_detected_flag`; all I2C reads happen in the main loop. MLC states: `STATIONARY_UPRIGHT`, `STATIONARY_NOT_UPRIGHT`, `IN_MOTION`, `SHAKEN`. FSM detects discrete impact and freefall events.

In armed low-power mode the accel sleep configuration depends on `SENSITIVITY`:

- **HIGH** — `LIS2DUX12_ConfigArmedSleep()` keeps the UCF loaded across sleep. MLC is still classifying when the MCU wakes, so the first read is immediately valid (no UCF reload, no accumulation window). Costs a few extra µA.
- **MEDIUM** — `LIS2DUX12_EnterMediumLowPowerWakeup()` wipes MLC/FSM and runs a 3 Hz ULP wake-only mode (~1.7 µA). Pairs with a raw-accel "significant motion" probe so big motion fires the alarm without waiting for a UCF reload.
- **LOW** — `LIS2DUX12_EnterUltraLowPowerWakeup()` wipes MLC/FSM and runs 1.6 Hz ULP wake-only (~1.5 µA). Always waits for MLC after wake.

On DEEPSTOP wakeup `HAL_PWR_WKUPx_Callback()` is used instead of `HAL_GPIO_EXTI_Callback()`.

### Power Management (`Core/Src/power_management.c`)

Two low-power modes, both entered when disconnected (no `stayAwakeFlag`) and PB5 (DEBUG_GPIO) is LOW:

- **`PowerMgmt_EnterLowPower_Idle()`** — used in `DISCONNECTED_IDLE`. Kills I2C bus power entirely. No motion wakeup; PB4 (cable) and PB5 (debug) can wake.
- **`PowerMgmt_EnterLowPower_Armed()`** — used in `LOCKED`. Uses `Gate_I2C_KeepPower()` (NOT `Gate_I2C()`) to keep the accelerometer powered and in its chosen wake-up mode. PB15 (accel INT), PB4 (cable), and PB5 (debug) remain as wakeup sources.

Two restore paths:

- **`PowerMgmt_RestoreAll()`** — full restore (BLE connect, cable plug). Reinitialises I2C, TIM2 (LEDs), TIM16 (buzzer), GPIO outputs, all interrupts, then runs `LIS2DUX12_Init()` (UCF reload) and `BATTERY_Init()`.
- **`PowerMgmt_RestoreForMotion()`** — lean wake on accel motion. Skips TIM2/LED restore, skips `BATTERY_Init`, and skips `LIS2DUX12_Init` when the chip kept MLC alive across sleep (HIGH sensitivity). Net wake latency is dominated by the I2C/TIM16 reinit (~ a few ms).

`PowerMgmt_IsLowPower()` gates sensor polling throughout the state machine.

### Motion Logger (`Core/Src/motion_logger.c`)

Ring buffer of up to **`MAX_MOTION_EVENTS` (169)** `MotionEvent_t` records (HAL tick + `MotionType_t`), mirrored to EEPROM so it survives DEEPSTOP / power loss. Boot time is synced from iOS via a BLE write; `MotionLogger_TickToDateTime()` converts ticks to calendar time for log transfers.

### Deferred motion alert (in `state_machine.c::State_Locked_Loop`)

While `LOCKED`, ordinary IN_MOTION/SHAKEN alerts are **deferred**: the alarm itself fires immediately, but the BLE alert + log entry are held until the MLC settles back to STATIONARY. A 3 s safety timeout flushes any pending motion that never settled. Impact and freefall (FSM) are always sent immediately.

(This was the entry point used by `door_detector.c` in earlier revisions — that module has since been removed; the deferral is now just used as a debounce so a brief jolt isn't double-logged.)

### Battery (`Core/Src/battery.c`, `Drivers/BQ27427/`)

Wraps the BQ27427 fuel gauge. Cached state is updated once a second from `main()` (`BATTERY_UpdateState()`); `BATTERY_GetSOC()` / `BATTERY_IsFullCached()` etc. read from that cache so the state machine never blocks on I2C. A long list of diagnostic getters (flags, control_status, temperature, qmax/RES learned bits, design capacity, taper rate, …) feeds the 51-byte BATTERYDIAG notification. `s_init_fail_stage` is a one-shot diagnostic that pinpoints which BATTERY_Init() step failed after the CC-Gain self-heal RESET.

### Drain Mode (`STM32_BLE/App/lockservice_app.c`)

Diagnostic high-load mode for fuel-gauge characterisation: white LED at full brightness + continuous `DRAIN_TONE_FREQUENCY_HZ` (60 Hz) buzzer tone. Started/stopped via `CMD_DRAIN_MODE`, auto-stops at `DRAIN_AUTO_STOP_SOC` (5 %). `Drain_Tick()` runs every loop iteration in `main()` and re-asserts outputs so other subsystems can't override it while active.

### `wd_system.{c,h}`

`wd_system.c` is currently empty. `wd_system.h` contains a draft of system-level types and macros that **nothing in the firmware currently includes** — the live state machine is in `state_machine.{c,h}`. The header is kept as a sketch for future system-level glue. Heads-up: it defines `GET_ALARM_TYPE` / `GET_SENSITIVITY` / `SENSITIVITY_LOW/MEDIUM/HIGH` macros that **collide** with same-named macros in `state_machine.h` but with different bit layouts. Do not include `wd_system.h` from production code without resolving the conflict.

## Critical Hardware Constraints

**UART / PA9:** `MX_USART1_UART_Init()` must NOT be called in production. Calling UART init reconfigures PA9 as AF push-pull and wastes ~100 µA. Only enable UART explicitly for debug sessions with a cable.

**I2C bus power:** The I2C bus VDD is switched via `I2C_POWER_Pin` (PA10). Always drive PA10 HIGH before initialising `hi2c1`. In armed LP, use `Gate_I2C_KeepPower()` — cutting power destroys the wake-up configuration loaded into the accelerometer.

**Buzzer:** `BUZZ_Pin` (PB0) drives TIM16_CH1 hardware PWM into an N-channel MOSFET. Frequency set via ARR, 50% duty via CCR. PB0 is held LOW as a regular GPIO when idle and only switched to AF while a tone is playing — keeps mains-frequency noise from coupling through the gate. TIM16 is buzzer-only; TIM2 is LED-only (CH2/CH3/CH4).

**EEPROM power:** Controlled separately by `EEPROM_POW_Pin` (PB6). Off by default; use `PowerMgmt_EEPROM_PowerOn/Off()` to bracket access.

**Debug GPIO:** `DEBUG_GPIO_Pin` (PB5) is EXTI rising-edge with pulldown. While HIGH, device stays awake and won't enter low power — allows debugger attachment after DEEPSTOP wake. Also registered as a PWR wakeup pin (`LL_PWR_WAKEUP_PB5`, polarity HIGH).

**Cable plug (PB4):** Falling edge = cable plugged in. Triggers `CablePlug_IRQCallback()` which sets `cablePlugFlag` + `stayAwakeFlag`. Also registered as a PWR wakeup pin (`LL_PWR_WAKEUP_PB4`, polarity LOW). After unplug, the device stays awake for `CABLE_UNPLUG_AWAKE_MS` (5 s) before being allowed back to deep sleep.

**DEEPSTOP wakeups bypass EXTI:** On STM32WB0x, wake from DEEPSTOP routes through the PWR controller, NOT EXTI. `HAL_PWR_WKUPx_Callback()` in `accelerometer.c` is the override: it sets `motion_detected_flag` for PB15 wakeups and calls `CablePlug_IRQCallback()` for PB4 wakeups.

**LED clamp after timer reinit:** After `MX_TIM2_Reinit()`, CCRs default to 0 — which (active-low) drives all LEDs full ON for the tens of ms it takes the rest of `RestoreAll()` to finish. `PowerMgmt_RestoreAll()` calls `LED_Off()` immediately after `MX_TIM2_Reinit()` to clamp the CCRs to 999 and prevent the flash.

**Safe boot:** If the cable is plugged in at boot, `main()` plays a descending tone and busy-waits in a `while (IS_CABLE_PLUGGED())` loop before initialising BLE. This is a recovery hatch for bricked-firmware reflashing.

**Boot stayAwakeFlag clear:** Just before entering the main loop, `main()` runs `NVIC_ClearPendingIRQ(GPIOB_IRQn)` and forces `stayAwakeFlag = 0` so a stray IRQ during init can't pin the device awake forever.

## Loyalty token (application-layer ownership)

There is no BLE-layer pairing or bonding. Any phone may connect (the GATT characteristics are all `BLE_GATT_SRV_PERM_NONE`), but only the owner's commands are processed. Ownership is enforced by a 4-byte token stored at EEPROM offset `0x10` (inside the reserved device-info region). See `STM32_BLE/App/loyalty.{c,h}` and `FW_LOYALTY_TOKEN_PROMPT.md` for full design.

The 6-byte EEPROM record is `[status, t0..t3, crc8]`. CRC8 (poly 0x07, init 0x00) covers status + token. Legacy records (pre-CRC, status `0xA7` with `crc=0x00`) are migrated in place once at boot. If the EEPROM read at boot fails the store goes UNHEALTHY and the dispatcher refuses **all** loyalty operations (including CLAIM) — otherwise a transient I2C glitch could let any phone hijack a claimed device.

Wire format on `APPTOWD` writes (from iOS):

| First byte | Meaning | Payload |
|------------|---------|---------|
| `0xC1` | `CMD_CLAIM_DEVICE` (first claim) | 4-byte token |
| `0xC2` | `CMD_VERIFY_OWNER` (reconnect) | 4-byte token |
| `0xC0` | `CMD_UNBOND_DEVICE` (user unpair) | 4-byte token |
| anything else | Existing opcodes (`0xF0`, settings, etc.) | Prefixed with 4-byte token: `[t0, t1, t2, t3, opcode, ...]` |

`CLAIM` is **idempotent** when the incoming token matches the already-stored one — the legitimate owner with stale local state (app reinstall, BondManager cleared) gets an immediate `RESP_CLAIM_OK` instead of a reject.

Firmware responses on `DEVICESTATUS` notify (2 bytes each):

| Bytes | Meaning |
|-------|---------|
| `[0xE7, 0x01]` | `RESP_CLAIM_OK` — token persisted |
| `[0xE9, 0x01]` | `RESP_VERIFY_OK` — token matches |
| `[0xE8, 0x01]` | `RESP_REJECT` — followed by disconnect ~50 ms later |
| `[0xE4, 0x01]` | `RESP_UNPAIR_ACK` — EEPROM wiped, followed by disconnect |

The dispatcher in `lockservice_app.c::LOCKSERVICE_Notification()` validates the token before forwarding any regular-command payload to the existing handlers. Payload offsets inside regular handlers are unchanged — the token is stripped before the inner switch runs.

**Recovery hatch:** holding the charging cable plugged in for **30 consecutive seconds** during the safe-boot busy-wait in `main.c` calls `Loyalty_Wipe()` and plays a distinct ascending tone. After unplug, any phone can claim the device again. This exists for users whose owner phone is lost or the app's local token has been deleted.

## Key Pin Assignments

| Signal | Pin | Notes |
|--------|-----|-------|
| LED1 (red) | PB3 | TIM2 CH4, HW PWM, active-LOW |
| LED2 (green) | PB2 | TIM2 CH3, HW PWM, active-LOW |
| LED3 (blue) | PB7 | TIM2 CH2, HW PWM, active-LOW |
| Buzzer | PB0 | TIM16 CH1, HW PWM, N-ch MOSFET (idle = GPIO LOW) |
| DEBUG_GPIO | PB5 | EXTI rising, pulldown, hold-awake |
| ACCEL_INT | PB15 | EXTI rising, MLC state change |
| BQ251_PG (cable) | PB4 | EXTI falling + PWR wakeup, LOW = cable in |
| STAT (charging) | PA11 | GPIO input, LOW = charging |
| I2C_POWER | PA10 | HIGH = I2C bus on |
| EEPROM_POW | PB6 | HIGH = EEPROM on |
| GPOUT | PA8 | General-purpose output |

## User-created source files

These are the files you should edit / clean up. Vendor (`Drivers/`, `Middlewares/`) and CubeMX-generated infrastructure (`main.c`, `app_entry.c`, `stm32wb0x_*`, `system_*`, `syscalls.c`, `sysmem.c`) are off-limits unless explicitly requested.

**`Core/Src` + `Core/Inc`:** `accelerometer`, `battery`, `lights`, `lis2dux12_app`, `motion_logger`, `power_management`, `sound`, `state_machine`, `wd_system`

**`STM32_BLE/App` (CubeMX-generated, edit only inside `USER CODE` blocks):** `lockservice_app`, plus the fully-user-authored `loyalty.{c,h}`
