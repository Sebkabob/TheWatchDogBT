# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

ONLY EDIT CODE WITHIN THE USER EDITABLE SECTIONS!!!

## Project Overview

WatchDogBT is embedded firmware for a Bluetooth Low Energy asset-tracking/alarm device built on the **STM32WB05KZV6TR** (Cortex-M0+, STM32WB0 family). The device uses a LIS2DUX12 accelerometer with an on-chip Machine Learning Core (MLC) for motion classification, a BQ25186 battery charger, an EEPROM, RGB LEDs, and a buzzer.

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

Custom **LockService** GATT service with two characteristics:
- `APPTOWD` — write-only, used by the iOS app to send commands (arm/disarm, settings, timestamp sync, find-my, log transfer trigger)
- `DEVICESTATUS` — notify, sends `[deviceState, deviceInfo, deviceBattery]` to the app

`app_ble.c` handles GAP/GATT stack init and connection events; `lockservice.c` is the auto-generated GATT server; `lockservice_app.c` contains all application logic for processing writes and sending notifications. `blenvm.c` wraps NVM for bonding persistence.

Advertising uses `HCI_ADV_FILTER_ACCEPT_LIST_CONNECT` — only bonded devices can connect.

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

Compares current accelerometer orientation against a reference captured at lock time (`DoorDetector_CaptureReference()`). Detects `DOOR_EVENT_OPENED` / `DOOR_EVENT_CLOSED` based on rotation delta.

## Critical Hardware Constraints

**UART / PA9:** `MX_USART1_UART_Init()` must NOT be called in production. Calling UART init reconfigures PA9 as AF push-pull and wastes ~100 µA. Only enable UART explicitly for debug sessions with a cable.

**I2C bus power:** The I2C bus VDD is switched via `I2C_POWER_Pin` (PA10). Always drive PA10 HIGH before initialising `hi2c1`. In armed LP, use `Gate_I2C_KeepPower()` — cutting power destroys the ULP wakeup configuration loaded into the accelerometer.

**Buzzer:** `BUZZ_Pin` (PB0) drives TIM16_CH1 hardware PWM into an N-channel MOSFET. Frequency set via ARR, 50% duty via CCR. TIM16 is buzzer-only; TIM2 is LED-only (CH2/CH3/CH4).

**EEPROM power:** Controlled separately by `EEPROM_POW_Pin` (PB6). Off by default; use `PowerMgmt_EEPROM_PowerOn/Off()` to bracket access.

**Debug GPIO:** `DEBUG_GPIO_Pin` (PB5) is EXTI rising-edge with pulldown. While HIGH, device stays awake and won't enter low power — allows debugger attachment after DEEPSTOP wake.

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
