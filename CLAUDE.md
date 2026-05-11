# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ONLY EDIT CODE WITHIN THE USER EDITABLE SECTIONS!!!

## Firmware Version

**Current: V1.12.2**  (last reconciled at commit `b59e8aa`)

Format: `V<MAJOR>.<MAIN>.<V2>` — single source of truth lives in `Core/Inc/firmware_version.h` (`FW_VERSION_MAJOR/MAIN/V2`, plus `FW_VERSION_STRING`). This line in CLAUDE.md and the macros in the header **must stay in sync**.

### Bump rules

| Field | When to bump | Cascading reset |
|-------|--------------|-----------------|
| `MAJOR` | Manual only — Sebastian explicitly asks ("bump major", architectural milestone). | **Reset both `MAIN` and `V2` to `0`.** |
| `MAIN`  | +1 for every commit that lands on the `main` branch. | **Reset `V2` to `0`.** |
| `V2`    | +1 for every commit that lands on the `V2` branch. | None. |

The cascading reset is non-negotiable — never carry an old `V2` (or `MAIN`) value forward across a higher-field bump. Worked examples:

- At `V1.0.7`, a `MAJOR` bump → `V2.0.0` (not `V2.0.7`).
- At `V1.3.5`, a `MAIN` bump → `V1.4.0` (not `V1.4.5`).
- At `V1.3.5`, a `V2` bump → `V1.3.6`.
- At `V1.3.5`, two `main` commits then one `V2` commit → `V1.5.1` (MAIN→4 resets V2→0; MAIN→5 resets V2→0; V2→1).

### Counting from scratch (the canonical recompute)

`MAIN` and `V2` are not free-running counters tied to a "reconciled SHA" — they are derived **from full history** every time. Always recompute from the full repo, not from a delta. The reconciled-SHA marker is just a convenience receipt of the last successful run; never trust it as the only input.

Canonical recompute:

```
MAIN = number of first-parent commits on `main`
       (i.e. `git rev-list --count --first-parent main`)

V2   = number of commits on `V2` not reachable from `main`
       (i.e. `git rev-list --count main..V2`)
```

This makes the rule self-correcting: if a `main` commit ever lands while you weren't looking, the next recompute notices it, picks up the new `MAIN`, and resets `V2` automatically. Merge commits and rebases do not double-count because `--first-parent main` follows only the merges/commits that actually landed on `main`, and `main..V2` excludes anything already on `main`.

`MAJOR` is never derived from history — it only changes when Sebastian says so.

### Claude — do this every session, before any other work

1. Run `git rev-list --count --first-parent main` → that's the new `MAIN`.
2. Run `git rev-list --count main..V2` → that's the new `V2`.
3. Compare against `firmware_version.h`. If different: update `FW_VERSION_MAIN` / `FW_VERSION_V2`, update the "Current: V…" line and the reconciled-sha (set it to current `git rev-parse --short HEAD`) in lockstep. Commit message style: `version bump to Vx.y.z`.
4. If both fields already match: do nothing, don't touch the files.

### When the user asks you to commit/push

Make your edits, stage them, then **before** committing run the recompute above against `HEAD` *as it would be after your commit lands* — i.e. add 1 to whichever counter the current branch feeds (`main` → `MAIN+1, V2=0`; `V2` → `V2+1`). Bake the bumped version into the same commit. Don't bump for amends, rebases, or local-only WIP commits unless the user says so. When in doubt about which branch a commit will land on, ask.

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
STABILIZING       → (15s elapsed) → CONNECTED_IDLE
LOCKED            → (motion)      → ALARM_ACTIVE
ALARM_ACTIVE      → (alarm_duration_seconds elapsed with no motion) → LOCKED
```

`StateMachine_ChangeState()` automatically sets/clears the ARMED bit when entering STABILIZING/LOCKED/ALARM_ACTIVE vs. any other state, then pushes a status notification.

STABILIZING is bounded by `STABILIZE_TIMEOUT_MS` (15 s). If the device never settles into 3 s of stillness within that window the loop bails to `CONNECTED_IDLE`, which clears ARMED and pushes a status update so iOS sees the device fall back to unlocked. This makes a permanent stuck blue pulse impossible.

The `deviceState` byte packs all user-configurable settings:

| Bits | Field | Values |
|------|-------|--------|
| 0 | ARMED | 0/1 |
| 2:1 | ALARM_TYPE | 0=none, 1=calm, 2=normal, 3=loud |
| 4:3 | SENSITIVITY | 0=low, 1=medium, 2=high |
| 5 | LIGHTS | 0/1 |
| 6 | LOGGING | 0/1 |
| 7 | SILENCE | 0/1 |

`deviceInfo` bit layout:
- Bit 0 `HIGH_PERF`: when set, BLE status updates run at 50 Hz (20 ms) instead of 2 Hz (500 ms).
- Bit 1 `alarmDisabled`: when set, `StateMachine_ChangeState(STATE_ALARM_ACTIVE)` is gated off — alarm LED never runs, motion logging + BLE motion alerts still fire. EEPROM-persisted via `sound.c`. **Buzzer is hard-gated at the TIM16 chokepoint** (`BUZZER_SetFrequency`): every non-zero frequency request is forced to zero while the flag is set, so find-my, drain mode, one-shot tones, and any other path are also silent. `AlarmDisabled_Set(true)` calls `BUZZER_Stop()` before persisting so any in-flight tone is killed immediately. Safe-boot tones (recovery hatch in `main()`) run before `AlarmDisabled_Init()` and are therefore unaffected by the persisted flag.
- Bit 2 `disconnectSoundDisabled`: when set, `SOUND_Disconnected()` (the three-tone descending chime fired on `LOCKSERVICE_DISCON_HANDLE_EVT`) returns immediately. EEPROM-persisted via `sound.c`. Independent of `alarmDisabled` — alarm tones, find-my, drain mode, and connection chimes are unaffected. The flag is checked inside `SOUND_Disconnected()` itself so callers don't have to gate.
- Bits 3..7 reserved (masked to 0 on receive and on echo).

A short **motion-grace window** is started on every BLE connect (`StateMachine_StartMotionGrace(2000)`): RestoreAll reloads the UCF, INT1 glitches, and the user is invariably handling the device while pairing — without the grace window every connect would fire the alarm. While active, `LOCKED` drains MLC/FSM events and suppresses transitions to `ALARM_ACTIVE`.

### BLE Layer (`STM32_BLE/App/`)

Custom **LockService** (16-bit UUID `0x183E`) GATT service with three characteristics:
- `APPTOWD` (write) — iOS → device commands. First byte of the inner payload (after the 4-byte loyalty token) = opcode; remaining bytes are payload. Settings writes (no opcode match) also carry a 6-byte trailing timestamp consumed by `UpdateBootTimeFromiOS()`.
- `DEVICESTATUS` (notify) — **19-byte** payload built by `LOCKSERVICE_Devicestatus_SendNotification()`. Pushed at 50 Hz when `HIGH_PERF` is set, 2 Hz otherwise (gated by `PowerMgmt_IsLowPower()`). Bytes 14..15 carry the low 2 bytes of the BD address (LE) — used by iOS as the user-visible "WatchDog #" identifier. Bytes 16..18 are the firmware version triplet (`FW_VERSION_MAJOR`, `FW_VERSION_MAIN`, `FW_VERSION_V2`) from `firmware_version.h`. Shorter framed responses on the same characteristic (motion alert, log-count, event-data, loyalty acks, …) are unchanged.
- `BATTERYDIAG` (notify) — **on-demand** TLV diagnostic dump, attached dynamically at init in `lockservice.c` (UUID/handle unchanged for backwards compatibility, but the payload schema is now sectioned, not the old 51-byte gauge struct). Triggered by `CMD_REQUEST_DIAG` (0xF4); the firmware no longer auto-pushes this characteristic. Wire format: `[format_version, section_count, (section_id, section_len, payload)*]` with sections SYSTEM, BATTERY, BLE, SENSOR, POWER, STORAGE. Buffer is sized for 220 bytes to allow future fields to be appended inside sections without requiring iOS to rediscover the service. The full per-section layout lives in `FW_DIAGNOSTICS_PROMPT.md` and is the source of truth for the iOS parser.

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
| `0xF4` | `CMD_REQUEST_DIAG` | optional byte[1] = section bitmask (default 0xFF). Triggers one TLV notification on `BATTERYDIAG`. |

Anything not matching an opcode is interpreted as a settings write: `cmd_data[0] → deviceState`, `cmd_data[1] → deviceInfo` (bit 0 HIGH_PERF + bit 1 → `AlarmDisabled_Set()` + bit 2 → `DisconnectSoundDisabled_Set()`, upper bits masked), optional `cmd_data[2] → alarm_duration_seconds` (clamped to 0..30, EEPROM-persisted via `sound.c`), optional `cmd_data[3] → led_brightness` (clamped to 1..255, EEPROM-persisted via `lights.c`), then a forced status notification. The settings core can be 1, 2, 3, or 4 bytes; the dispatcher computes its length as `cmd_length - 6` when the trailing 6-byte timestamp is present (`cmd_length >= 7`). Status LED calls (armed/stabilizing/alarm/find-my/connected-rainbow) multiply by `LedBrightness_Get()`; the charging-status path and the drain-mode diagnostic bypass the scalar. The DEVICESTATUS byte 13 echoes `(deviceInfo & 0x01) | (AlarmDisabled_Get() ? 0x02 : 0) | (DisconnectSoundDisabled_Get() ? 0x04 : 0)` so iOS sees the persisted alarmDisabled and disconnectSoundDisabled state across boots.

`app_ble.c` handles GAP/GATT stack init and connection events; `lockservice.c` is the auto-generated GATT server (with hand-added BATTERYDIAG inside USER CODE blocks); `lockservice_app.c` contains all application logic for processing writes and sending notifications. `g_bd_address[6]` is published in `app_ble.c` and `bd_address_override` in `main.c` controls whether the code-defined BD address overwrites EEPROM at boot.

Advertising uses `HCI_ADV_FILTER_ACCEPT_LIST_CONNECT` — only bonded devices can connect.

On connect, `PowerMgmt_RestoreAll()` is called and a 2-second motion-grace window is armed. The firmware no longer pushes an unsolicited `LOCKSERVICE_SendEventCount()` on connect — that fired before the loyalty handshake completed and leaked the pending-event count to any phone the radio accept-list let in. iOS now drives the drain itself by calling `requestMotionLogCount()` inside `onLoyaltyVerifiedHook`, 0.5 s after `RESP_CLAIM_OK` / `RESP_VERIFY_OK`. Log delivery is therefore gated by application-layer ownership verification.

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

Ring buffer of up to **`MAX_MOTION_EVENTS` (169)** `MotionEvent_t` records (4-byte `epoch_seconds_2000` + `MotionType_t`), mirrored to EEPROM so it survives DEEPSTOP / power loss. Calendar time is captured **at log time** via `MotionLogger_NowSeconds2000()` and frozen in the slot; the wire-side reader (`MotionLogger_EpochSecondsToDateTime`) is a pure decomposition with no anchor dependency.

**Why store calendar instead of HAL ticks:** the prior design stored `HAL_GetTick()` per event and converted to calendar at read time using whichever anchor was current then. Three failure modes: (1) every `MotionLogger_SetBootTime` call from iOS (sendSettings, every UI toggle, every Motion Logs view open) reset `boot_tick_ms = HAL_GetTick()`, so all previously-logged events with smaller ticks underflowed the `uint32_t` subtraction and produced ~49-day-future garbage; (2) after a reset, `HAL_GetTick` restarts at 0 while the reloaded EEPROM anchor still held the previous boot's `boot_tick_ms`, so any new event underflowed instantly; (3) SysTick is suspended in DEEPSTOP, so events logged after the device next slept were stamped with the anchor's calendar value + a few ms of wake time — minutes of real elapsed wall-clock disappeared. Storing seconds-since-2000 captured at log time means nothing downstream — anchor moves, reboots, drift — can disturb an event once it's written.

**Monotonic clock source:** `MotionLogger_NowSeconds2000` does NOT use `HAL_GetTick()`. The radio-timer's 64-bit counter via `HAL_RADIO_TIMER_GetCurrentSysTime() / 409600` is the monotonic reference. It's clocked from LSI and keeps counting through DEEPSTOP — but only if the DEEPSTOP mode is `PWR_DEEPSTOP_WITH_SLOW_CLOCK_ON`. `Projects/Common/BLE/Interfaces/stm32_lpm_if.c::PWR_EnterOffMode` is set to `_ON`; flipping it to `_OFF` (the CubeMX default) saves ~100 nA but kills every low-power timer on the chip, including this counter. Don't flip it back without reading the comment block in that file.

**Time anchor:** the iOS-sync anchor (`boot_time`: calendar + `boot_monotonic_secs`) is RAM-only authority for the current session. iOS pushes a fresh anchor inside `sendSettings()` right after `RESP_CLAIM_OK` / `RESP_VERIFY_OK`. `MotionLogger_SetBootTime` still mirrors the bytes to EEPROM at `0x00..0x0A` (magic `0xB7`) — and `state_machine.c` checkpoints again on entry to `STATE_LOCKED` — but `MotionLogger_Init` deliberately does **not** consume the persisted bytes at boot. They're forensic data. Events logged after a reset and before iOS reconnects get the unknown-time sentinel (`epoch_seconds_2000 = 0`), which the wire-side encoder emits as `(0,1,1,0,0,0)` and the iOS parser maps to `nil` ("Unknown time"). Without an RTC, this is the honest answer.

**EEPROM magic byte:** bumped to `0xA6` (was `0xA5`) when the per-slot semantics changed. Old EEPROMs load as empty; legacy events couldn't be displayed correctly anyway.

**Deferred EEPROM during `ALARM_ACTIVE`:** an M24C08 page commit pins the main loop for ~15–25 ms (`HAL_Delay(2)` for power-on + `i2c_wait` polling). TIM16 keeps playing the last-set buzzer frequency through the stall, which is audible as the alarm tone hitching on a single pitch. `StateMachine_ChangeState` calls `MotionLogger_SetDeferEEPROM(1)` on entry to `ALARM_ACTIVE` and `FlushPending` + `SetDeferEEPROM(0)` on exit (after `BUZZER_Stop`). While deferred, events still hit the RAM ring immediately — only the EEPROM mirror lags. Tradeoff: a hard reset *during* the alarm loses deferred events that hadn't been flushed yet.

**Other I2C work paused during `ALARM_ACTIVE`** for the same reason: `BATTERY_UpdateState()` in `main.c` (the 1 Hz fuel-gauge poll does ~10 I2C transactions = 20–30 ms blocking) and the live `LIS2DUX12_ReadAcceleration()` inside `LOCKSERVICE_Devicestatus_SendNotification`. The battery cache and live accel bytes go stale during the alarm; iOS uses the motion-alert path for the live channel anyway. Both resume the moment we leave `ALARM_ACTIVE`.

### Deferred motion alert (in `state_machine.c::State_Locked_Loop`)

While `LOCKED`, ordinary IN_MOTION/SHAKEN alerts are **deferred**: the alarm itself fires immediately, but the BLE alert + log entry are held until the MLC settles back to STATIONARY. A 3 s safety timeout flushes any pending motion that never settled. Impact and freefall (FSM) are always sent immediately.

**One log per wake.** The LP-wake path used to log `MOTION_TYPE_IN_MOTION` unconditionally up-front and *then* log a second event for whatever classifier (FSM impact/freefall, or motion_pending → settle) fired below — so every wake landed 2-3 entries in the ring. The current code tracks `wake_handled`: the classifier branches that produce a real log (FSM immediate, or motion_pending which logs later on settle) mark it true, and the catch-all fallback `MotionLogger_LogEvent(MOTION_TYPE_IN_MOTION)` only runs when nothing else qualified. Brief blips that the MLC missed still surface, but every wake produces exactly one ring entry.

**Alarm-loop log gating.** `State_Alarm_Active_Loop`'s INT-triggered `MotionLogger_LogEvent` is gated on a qualifying classification (`mlc_out == IN_MOTION/SHAKEN || impact || freefall`). A stationary→stationary INT (no FSM event) used to log a spurious `MOTION_TYPE_IN_MOTION` because the default `motionType` was set above the gate; the log is now inside the qualifying branch.

### Battery (`Core/Src/battery.c`, `Drivers/BQ27427/`)

Wraps the BQ27427 fuel gauge. Cached state is updated once a second from `main()` (`BATTERY_UpdateState()`); `BATTERY_GetSOC()` / `BATTERY_IsFullCached()` etc. read from that cache so the state machine never blocks on I2C. A long list of diagnostic getters (flags, control_status, temperature, qmax/RES learned bits, design capacity, taper rate, …) feeds the BATTERY section of the on-demand diagnostic dump. `s_init_fail_stage` is a one-shot diagnostic that pinpoints which BATTERY_Init() step failed after the CC-Gain self-heal RESET.

### Diagnostics (`STM32_BLE/App/lockservice_app.c::LOCKSERVICE_SendDiagnostic`)

On-demand, sectioned, TLV-framed dump of every cross-cutting health signal the firmware can surface, emitted on the `BATTERYDIAG` characteristic when iOS sends `CMD_REQUEST_DIAG`. There is no auto-push — the device is silent on this characteristic until asked. Sections (per `FW_DIAGNOSTICS_PROMPT.md`):

| ID | Name | Sourced from |
|----|------|--------------|
| 0x01 | SYSTEM | `power_management.c` (uptime/boot/reset cause), `firmware_version.h`, `Loyalty_StoreUnhealthy`, `BATTERY_GetInitCompleted` |
| 0x02 | BATTERY | All `BATTERY_Get*` accessors — schema-compatible with the legacy 51-byte v11 BatteryDiagnostic struct |
| 0x03 | BLE | TODO counters in `lockservice_app.c` (RSSI, conn count, last disconnect reason, MTU) — currently zero/sentinel |
| 0x04 | SENSOR | `lis2dux12_app_get_cached_mlc_state` + TODO transition / INT1 / log counters |
| 0x05 | POWER | `PowerMgmt_IsLowPower` + TODO wake-source counters and time-in-LP accumulator |
| 0x06 | STORAGE | `MotionLogger_GetEventCount`, loyalty health/claimed flags + TODO peripheral fail counters |

Reset cause is captured by `PowerMgmt_CaptureResetCause()` from `RCC->CSR` as the very first thing in `main()`, packed into a single byte (bits 0..4: PAD/POR/SFT/WDG/LOCKUP), then `__HAL_RCC_CLEAR_RESET_FLAGS()` is called so the next boot's flags are clean. Boot count is a uint32 LE persisted at EEPROM `0x20` (inside the existing reserved 0x000..0x03F device-info block — does not collide with motion log or loyalty); incremented once at boot by `PowerMgmt_BootCount_Init()`.

Each section payload reserves trailing bytes for future fields. iOS reads exactly `section_len` bytes per section and ignores trailing reserved bytes, which is how new fields can be appended later without breaking the app.

### Drain Mode (`STM32_BLE/App/lockservice_app.c`)

Diagnostic high-load mode for fuel-gauge characterisation: white LED at full brightness + continuous `DRAIN_TONE_FREQUENCY_HZ` (60 Hz) buzzer tone. Started/stopped via `CMD_DRAIN_MODE`, auto-stops at `DRAIN_AUTO_STOP_SOC` (5 %). `Drain_Tick()` runs every loop iteration in `main()` and re-asserts outputs so other subsystems can't override it while active.

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

**Auto-disarm on owner authentication:** every successful CLAIM/VERIFY (both `RESP_CLAIM_OK` and `RESP_VERIFY_OK` paths) calls `Unlock_OnOwnerAuthenticated()` which clears the `ARMED` bit in `deviceState`. The state-loops (`State_Locked_Loop`, `State_Alarm_Active_Loop`) already gate on `!ARMED` to tear down (buzzer stop, LED reset) and transition to `CONNECTED_IDLE` with a forced status notify, so the helper is just one bit-clear. The disarm is intentionally gated on loyalty verify, not the raw BLE connect event — connecting alone doesn't prove ownership.

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

**`Core/Src` + `Core/Inc`:** `accelerometer`, `battery`, `lights` (RGB driver + persisted user brightness), `lis2dux12_app`, `motion_logger`, `power_management`, `sound` (buzzer driver + persisted alarm-duration, alarm-disabled, and disconnect-sound-disabled), `state_machine`

**`STM32_BLE/App` (CubeMX-generated, edit only inside `USER CODE` blocks):** `lockservice_app`, plus the fully-user-authored `loyalty.{c,h}`
