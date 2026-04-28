# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ONLY EDIT CODE WITHIN THE USER EDITABLE SECTIONS!!!

## Project Overview

WatchDogBT is embedded firmware for a Bluetooth Low Energy asset-tracking/alarm device built on the **STM32WB05KZV6TR** (Cortex-M0+, STM32WB0 family). The device uses a LIS2DUX12 accelerometer with an on-chip Machine Learning Core (MLC) for motion classification, a BQ25186 battery charger (PG/STAT lines on PB4/PA11), a BQ27427 fuel gauge (I2C, see `Drivers/BQ27427/`), an M24C08 EEPROM (see `Drivers/M24C08/`), RGB LEDs, and a magnetic buzzer.

## Build System

This is an **STM32CubeIDE** project. There is no command-line Makefile. Build and flash using STM32CubeIDE (`WatchDogBT.launch` / `WatchDogBT.cfg`). The `.ioc` file is the CubeMX configuration — regenerating from it overwrites the `/* USER CODE BEGIN/END */` blocks (those are preserved by CubeMX).

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

`STATE_SLEEP` exists in the enum but its loop (`State_Sleep_Loop`) is currently a placeholder — no transitions land there in production. `StateMachine_ChangeState()` automatically sets/clears the ARMED bit when entering STABILIZING/LOCKED/ALARM_ACTIVE vs. any other state, then pushes a status notification.

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

### BLE Layer (`STM32_BLE/App/`)

Custom **LockService** (16-bit UUID `0x183E`) GATT service with three characteristics:
- `APPTOWD` (write) — iOS → device commands. First byte = opcode; remaining bytes are payload. Settings writes (no opcode match) also carry a 6-byte trailing timestamp consumed by `UpdateBootTimeFromiOS()`.
- `DEVICESTATUS` (notify) — `DEVICESTATUS_SIZE = 11` bytes. Built by `LOCKSERVICE_Devicestatus_SendNotification()` and pushed at 50 Hz when `HIGH_PERF` is set, 2 Hz otherwise (gated by `PowerMgmt_IsLowPower()`).
- `BATTERYDIAG` (notify) — 51-byte packed gauge telemetry payload (v11), attached dynamically at init in `lockservice.c`. Sent every ~1 s from `main()`'s battery tick.

iOS opcodes (`lockservice_app.h`):

| Opcode | Name | Payload |
|--------|------|---------|
| `0xF0` | `CMD_REQUEST_LOG_COUNT` | none — kicks off log transfer |
| `0xF1` | `CMD_REQUEST_EVENT` | `uint16_t` BE index of event to fetch |
| `0xF2` | `CMD_CLEAR_LOG` | none |
| `0xF3` | `CMD_ACK_EVENT` | none |
| `0xFA` | `CMD_FIND_MY_DEVICE` | byte[1] bit 0 = start |
| `0xFB` | `CMD_RESET_DEVICE` | none — calls `NVIC_SystemReset()` |
| `0xFC` | `CMD_DRAIN_MODE` | byte[1] bit 0: 1=start drain, 0=stop |

Anything not matching an opcode is interpreted as a settings write: `received_data[0] → deviceState`, `received_data[1] → deviceInfo`, then a forced status notification.

`app_ble.c` handles GAP/GATT stack init and connection events; `lockservice.c` is the auto-generated GATT server (with hand-added BATTERYDIAG inside USER CODE blocks); `lockservice_app.c` contains all application logic for processing writes and sending notifications. `blenvm.c` wraps NVM for bonding persistence. `g_bd_address[6]` is published in `app_ble.c` and `bd_address_override` in `main.c` controls whether the code-defined BD address overwrites EEPROM at boot.

Advertising uses `HCI_ADV_FILTER_ACCEPT_LIST_CONNECT` — only bonded devices can connect.

On connect, `PowerMgmt_RestoreAll()` is called and any pending logged events trigger an unsolicited `LOCKSERVICE_SendEventCount()` so iOS knows to drain them.

### Accelerometer (`Core/Src/accelerometer.c`, `Core/Src/lis2dux12_app.c`)

LIS2DUX12 communicates over I2C1 (`hi2c1`). A pre-built UCF asset is loaded at `LIS2DUX12_Init()` to configure the MLC and FSM. INT1 is on **PB15** (EXTI rising edge) — the ISR only sets `motion_detected_flag`; all I2C reads happen in the main loop. MLC states: `STATIONARY_UPRIGHT`, `STATIONARY_NOT_UPRIGHT`, `IN_MOTION`, `SHAKEN`. FSM detects discrete impact and freefall events.

In armed low-power mode the accel is put into 1.6 Hz ULP wakeup mode (`LIS2DUX12_EnterUltraLowPowerWakeup()`). On DEEPSTOP wakeup `HAL_PWR_WKUPx_Callback()` is used instead of `HAL_GPIO_EXTI_Callback()`.

### Power Management (`Core/Src/power_management.c`)

Two low-power modes, both entered when disconnected (no `stayAwakeFlag`) and PB5 (DEBUG_GPIO) is LOW:

- **`PowerMgmt_EnterLowPower_Idle()`** — used in `DISCONNECTED_IDLE`. Kills I2C bus power entirely. No motion wakeup; PB4 (cable) and PB5 (debug) can wake.
- **`PowerMgmt_EnterLowPower_Armed()`** — used in `LOCKED`. Uses `Gate_I2C_KeepPower()` (NOT `Gate_I2C()`) to keep the accelerometer powered and in ULP wakeup mode. PB15 (accel INT), PB4 (cable), and PB5 (debug) remain as wakeup sources.

`PowerMgmt_RestoreAll()` reinitialises all peripherals on wake. `PowerMgmt_IsLowPower()` gates sensor polling throughout the state machine.

### Motion Logger (`Core/Src/motion_logger.c`)

Ring buffer of up to 100 `MotionEvent_t` records (HAL tick + `MotionType_t`). Boot time is synced from iOS via a BLE write; `MotionLogger_TickToDateTime()` converts ticks to calendar time for log transfers.

### Door Detector (`Core/Src/door_detector.c`)

Compares current accelerometer orientation against a reference captured at lock time (`DoorDetector_CaptureReference()`). Detects `DOOR_EVENT_OPENED` / `DOOR_EVENT_CLOSED` based on rotation delta. Sensitivity is driven from the `SENSITIVITY` bits in `deviceState` via `DoorDetector_SetSensitivity()` when entering STABILIZING.

While LOCKED, the state machine **defers** ordinary IN_MOTION/SHAKEN alerts until the MLC settles, then either logs a door event (if rotation crossed the threshold) or the originally-pending motion type. The alarm itself fires immediately — the deferral only affects which `MotionType_t` is logged/notified. Impact and freefall (FSM) are always sent immediately. A 3 s safety timeout flushes any pending motion that never settled.

### Battery (`Core/Src/battery.c`, `Drivers/BQ27427/`)

Wraps the BQ27427 fuel gauge. Cached state is updated once a second from `main()` (`BATTERY_UpdateState()`); `BATTERY_GetSOC()` / `BATTERY_IsFullCached()` etc. read from that cache so the state machine never blocks on I2C. A long list of diagnostic getters (flags, control_status, temperature, qmax/RES learned bits, design capacity, taper rate, …) feeds the 51-byte BATTERYDIAG notification. `s_init_fail_stage` is a one-shot diagnostic that pinpoints which BATTERY_Init() step failed after the CC-Gain self-heal RESET.

### Drain Mode (`STM32_BLE/App/lockservice_app.c`)

Diagnostic high-load mode for fuel-gauge characterisation: white LED at full brightness + continuous `DRAIN_TONE_FREQUENCY_HZ` (100 Hz) buzzer tone. Started/stopped via `CMD_DRAIN_MODE`, auto-stops at `DRAIN_AUTO_STOP_SOC` (5 %). `Drain_Tick()` runs every loop iteration in `main()` and re-asserts outputs so other subsystems can't override it while active.

### `wd_system.c`

Currently empty placeholder (Oct 2025) — reserved for future system-level glue.

## Critical Hardware Constraints

**UART / PA9:** `MX_USART1_UART_Init()` must NOT be called in production. Calling UART init reconfigures PA9 as AF push-pull and wastes ~100 µA. Only enable UART explicitly for debug sessions with a cable.

**I2C bus power:** The I2C bus VDD is switched via `I2C_POWER_Pin` (PA10). Always drive PA10 HIGH before initialising `hi2c1`. In armed LP, use `Gate_I2C_KeepPower()` — cutting power destroys the ULP wakeup configuration loaded into the accelerometer.

**Buzzer:** `BUZZ_Pin` (PB0) drives TIM16_CH1 hardware PWM into an N-channel MOSFET. Frequency set via ARR, 50% duty via CCR. TIM16 is buzzer-only; TIM2 is LED-only (CH2/CH3/CH4).

**EEPROM power:** Controlled separately by `EEPROM_POW_Pin` (PB6). Off by default; use `PowerMgmt_EEPROM_PowerOn/Off()` to bracket access.

**Debug GPIO:** `DEBUG_GPIO_Pin` (PB5) is EXTI rising-edge with pulldown. While HIGH, device stays awake and won't enter low power — allows debugger attachment after DEEPSTOP wake. Also registered as a PWR wakeup pin (`LL_PWR_WAKEUP_PB5`, polarity HIGH).

**Cable plug (PB4):** Falling edge = cable plugged in. Triggers `CablePlug_IRQCallback()` which sets `cablePlugFlag` + `stayAwakeFlag`. Also registered as a PWR wakeup pin (`LL_PWR_WAKEUP_PB4`, polarity LOW). After unplug, the device stays awake for `CABLE_UNPLUG_AWAKE_MS` (5 s) before being allowed back to deep sleep.

**DEEPSTOP wakeups bypass EXTI:** On STM32WB0x, wake from DEEPSTOP routes through the PWR controller, NOT EXTI. `HAL_PWR_WKUPx_Callback()` in `accelerometer.c` is the override: it sets `motion_detected_flag` for PB15 wakeups and calls `CablePlug_IRQCallback()` for PB4 wakeups.

**Safe boot:** If the cable is plugged in at boot, `main()` plays a descending tone and busy-waits in a `while (IS_CABLE_PLUGGED())` loop before initialising BLE. This is a recovery hatch for bricked-firmware reflashing.

**Boot stayAwakeFlag clear:** Just before entering the main loop, `main()` runs `NVIC_ClearPendingIRQ(GPIOB_IRQn)` and forces `stayAwakeFlag = 0` so a stray IRQ during init can't pin the device awake forever.

## Key Pin Assignments

| Signal | Pin | Notes |
|--------|-----|-------|
| LED1 (red) | PB3 | TIM2 CH4, HW PWM |
| LED2 (green) | PB2 | TIM2 CH3, HW PWM |
| LED3 (blue) | PB7 | TIM2 CH2, HW PWM |
| Buzzer | PB0 | TIM16 CH1, HW PWM, N-ch MOSFET |
| DEBUG_GPIO | PB5 | EXTI rising, pulldown, hold-awake |
| ACCEL_INT | PB15 | EXTI rising, MLC state change |
| BQ251_PG (cable) | PB4 | EXTI falling + PWR wakeup, LOW = cable in |
| STAT (charging) | PA11 | GPIO input, LOW = charging |
| I2C_POWER | PA10 | HIGH = I2C bus on |
| EEPROM_POW | PB6 | HIGH = EEPROM on |
| GPOUT | PA8 | General-purpose output |
