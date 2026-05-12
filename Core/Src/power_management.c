/***************************************************************************
 * power_management.c
 * created by Sebastian Forenza 2026
 *
 * Peripheral gating for low-power advertising / armed-locked states.
 *
 * Wake sources kept live in every low-power mode:
 *   PB4 (BQ251_PG)  — EXTI falling + PWR wakeup, LOW = cable plugged in
 *   PB5 (DEBUG_GPIO)— EXTI rising  + PWR wakeup, HIGH holds device awake
 *
 * Two low-power flavours:
 *   _EnterLowPower_Idle()  kills I2C bus power; no motion wakeup.
 *   _EnterLowPower_Armed() keeps the accel powered (Gate_I2C_KeepPower)
 *                          so PB15 can wake the MCU from DEEPSTOP.
 *
 * RestoreAll() reinitialises every peripheral on a full wake; the lean
 * RestoreForMotion() path skips LED/UCF/battery init when the wake came
 * from the accelerometer in armed LP.
 ***************************************************************************/

#include "main.h"
#include "power_management.h"
#include "accelerometer.h"
#include "battery.h"
#include "lights.h"
#include "sound.h"
#include "motion_logger.h"
#include "state_machine.h"
#include "m24cxx.h"
#include <string.h>

extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim16;

static volatile uint8_t peripherals_gated = 0;

// 1 if armed LP kept MLC alive (HIGH sens). RestoreForMotion uses this to
// decide whether the UCF needs a fresh reload.
static volatile uint8_t mlc_kept_alive = 0;

// 1 if RestoreForMotion ran (lean restore) but RestoreAll hasn't yet finished
// the work it skipped (TIM2/LEDs, PA8 GPOUT, PA11 STAT input, UCF reload when
// MLC was kept alive, BATTERY_Init). Cleared by a full RestoreAll. This is
// the BLE-reconnect-while-locked fix: a motion wake that happens just before
// the user reconnects (which is the common case — they're handling the
// device) used to leave peripherals_gated=0, and the subsequent RestoreAll
// silently early-returned, leaving TIM2 dead (glitchy armed-pulse LED), the
// accel's INT1 latched (no further motion EXTI → no alarm), and the gauge
// uninitialised.
static volatile uint8_t restore_incomplete = 0;

extern void MX_I2C1_Reinit(void);
extern void MX_TIM2_Reinit(void);
extern void MX_TIM16_Reinit(void);

/***************************************************************************
 * Gate_I2C — kill I2C peripheral, hard-cut bus power, float SDA/SCL
 *   PA10 (I2C_POWER) is held as a driven LOW push-pull output across
 *   DEEPSTOP — leaving it analog/Hi-Z let the load-switch gate drift,
 *   which can partially re-power the bus rail and leak through the
 *   LIS2DUX12 / BQ27427 / EEPROM body diodes.
 ***************************************************************************/
static void Gate_I2C(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = I2C_POWER_Pin;
    HAL_GPIO_Init(I2C_POWER_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/***************************************************************************
 * Gate_I2C_KeepPower — gate I2C peripheral but keep the bus powered
 *   Used in armed LP so the accelerometer retains its ULP wake-up config
 *   and can still fire INT1. SDA/SCL go analog; external pull-ups hold
 *   them HIGH so no current flows.
 ***************************************************************************/
static void Gate_I2C_KeepPower(void)
{
    HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void Gate_EEPROM(void)
{
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = EEPROM_POW_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EEPROM_POW_GPIO_Port, &gpio);
}

/***************************************************************************
 * Gate_Timers — stop TIM2 (LEDs) and TIM16 (buzzer), float their pins
 *   With timer clocks off, AF pins default LOW which would turn on the
 *   active-low LEDs — analog mode prevents that leakage.
 ***************************************************************************/
static void Gate_Timers(void)
{
    LED_Off();
    BUZZER_Stop();

    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_DeInit(&htim2);
    __HAL_RCC_TIM2_CLK_DISABLE();

    HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
    HAL_TIM_Base_DeInit(&htim16);
    __HAL_RCC_TIM16_CLK_DISABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = LED1_Pin | LED2_Pin | LED3_Pin | BUZZ_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/***************************************************************************
 * Gate_UART — disable USART1 clock, float PA9 / PA11 / PB14
 *   PA11 (charger STAT) is also gated since it's not needed during sleep.
 ***************************************************************************/
static void Gate_UART(void)
{
    if (__HAL_RCC_USART1_IS_CLK_ENABLED()) {
        __HAL_RCC_USART1_CLK_DISABLE();
    }

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin  = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin  = STAT_Pin;
    HAL_GPIO_Init(STAT_GPIO_Port, &gpio);

    gpio.Pin  = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void Gate_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);
}

/***************************************************************************
 * Gate_SWD — drop SWDIO/SWCLK to analog and kill DEEPSTOP debug retention
 *   PWR_CR2_DBGRET defaults ON; left alone it retains PA2 SWDIO / PA3 SWCLK
 *   pin state across DEEPSTOP and keeps part of the debug logic alive —
 *   ~10–15 µA in adv. Disabling it here for the LP window reclaims that
 *   current; Restore_SWD reverses it on wake so a probe can still attach.
 ***************************************************************************/
static void Gate_SWD(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Pin  = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_2 | PWR_GPIO_BIT_3);
    HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_2 | PWR_GPIO_BIT_3);

    LL_PWR_DisableDBGRET();
}

/***************************************************************************
 * Restore_SWD — re-enable SWD pins so a probe can attach mid-session
 *   Mirrors the SWDIO config from MX_GPIO_Init (AF7, pullup) and re-enables
 *   the DEEPSTOP retention bias on PA2 so a probe sees a clean line.
 ***************************************************************************/
static void Restore_SWD(void)
{
    LL_PWR_EnableDBGRET();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_SWDIO;
    gpio.Pin       = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Alternate = GPIO_AF7_SWCLK;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Pin       = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);

    HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_2);
}

static void Keep_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/***************************************************************************
 * Keep_CablePlugInterrupt — keep PB4 active across sleep
 *   Already configured as EXTI falling + pullup in MX_GPIO_Init; we just
 *   clear the line and enrol it as a PWR wakeup pin (LOW polarity).
 ***************************************************************************/
static void Keep_CablePlugInterrupt(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(BQ251_PG_GPIO_Port, BQ251_PG_Pin);

    LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PB4);
    LL_PWR_SetWakeUpPinPolarityLow(LL_PWR_WAKEUP_PB4);

    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

static void Gate_GPIO_Outputs(void)
{
    HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = GPOUT_Pin;
    HAL_GPIO_Init(GPOUT_GPIO_Port, &gpio);
}

static void Restore_GPIO_Outputs(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPOUT_Pin;
    HAL_GPIO_Init(GPOUT_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);
}

static void Restore_AccelInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = ACCEL_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ACCEL_INT_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(ACCEL_INT_GPIO_Port, ACCEL_INT_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

static void Restore_CablePlugInterrupt(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = BQ251_PG_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BQ251_PG_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(BQ251_PG_GPIO_Port, BQ251_PG_Pin);
    HAL_NVIC_EnableIRQ(GPIOB_IRQn);
}

/***************************************************************************
 * Restore_UART_Pins — bring PA11 (STAT) back as input, leave UART floating
 *   PA9 / PB14 stay analog — UART must be explicitly enabled for debug.
 ***************************************************************************/
static void Restore_UART_Pins(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = STAT_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(STAT_GPIO_Port, &gpio);
}

void PowerMgmt_EnterLowPower_Idle(void)
{
    if (peripherals_gated) return;

    Gate_Timers();

    /* Force the LIS2DUX12 to ODR=0 + soft-reset before the I2C rail goes
     * down. If the accel is on the switched rail, this is harmless (the
     * rail kill below shuts it off anyway). If it's on always-on rail,
     * this drops it from ~5 µA MLC-running to ~0.4 µA power-down. */
    (void)LIS2DUX12_ResetAndPowerDown();

    Gate_I2C();
    Gate_EEPROM();
    Gate_UART();
    Gate_AccelInterrupt();
    Gate_GPIO_Outputs();
    Gate_SWD();
    Keep_CablePlugInterrupt();

    peripherals_gated = 1;
    restore_incomplete = 0;
}

/***************************************************************************
 * PowerMgmt_EnterLowPower_Armed — sleep with motion wake-up
 *   Sensitivity selects accel sleep mode:
 *     HIGH    keep MLC running (UCF intact). A few extra µA, but the next
 *             read is valid the moment the MCU wakes.
 *     MEDIUM  ULP wake-only at 3 Hz (~1.7 µA). Faster latency than LOW;
 *             pairs with a raw-accel probe so significant motion fires
 *             immediately and only marginal motion waits for UCF reload.
 *     LOW     ULP wake-only at 1.6 Hz (~1.5 µA, lowest power).
 *   Always uses Gate_I2C_KeepPower() — cutting bus power destroys the
 *   ULP wake-up configuration loaded into the accelerometer.
 ***************************************************************************/
void PowerMgmt_EnterLowPower_Armed(void)
{
    if (peripherals_gated) return;

    Gate_Timers();

    uint8_t sens = GET_SENSITIVITY(deviceState);
    if (sens == SENSITIVITY_HIGH) {
        LIS2DUX12_ConfigArmedSleep();
        mlc_kept_alive = 1;
    } else if (sens == SENSITIVITY_MEDIUM) {
        LIS2DUX12_EnterMediumLowPowerWakeup();
        mlc_kept_alive = 0;
    } else {
        LIS2DUX12_EnterUltraLowPowerWakeup();
        mlc_kept_alive = 0;
    }

    Gate_I2C_KeepPower();
    Gate_EEPROM();
    Gate_UART();
    Keep_AccelInterrupt();
    Gate_GPIO_Outputs();
    Gate_SWD();
    Keep_CablePlugInterrupt();

    peripherals_gated = 1;
    restore_incomplete = 0;
}

/***************************************************************************
 * Restore_I2C_Bus — reinit I2C peripheral and assert PA10 / PB6 as outputs
 *   In armed LP the chip stayed powered (Gate_I2C_KeepPower); restating
 *   PA10 HIGH here is harmless. EEPROM_POW (PB6) is left LOW until use.
 ***************************************************************************/
static void Restore_I2C_Bus(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = I2C_POWER_Pin;
    HAL_GPIO_Init(I2C_POWER_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(I2C_POWER_GPIO_Port, I2C_POWER_Pin, GPIO_PIN_SET);

    gpio.Pin = EEPROM_POW_Pin;
    HAL_GPIO_Init(EEPROM_POW_GPIO_Port, &gpio);
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);

    __HAL_RCC_I2C1_CLK_ENABLE();
    MX_I2C1_Reinit();
}

/***************************************************************************
 * PowerMgmt_RestoreAll — full peripheral restore (BLE connect / cable plug)
 *
 * Two entry conditions to handle:
 *   peripherals_gated == 1  : device is fresh out of DEEPSTOP, everything is
 *                             gated, do the full bring-up.
 *   restore_incomplete == 1 : RestoreForMotion already ran (lean restore) and
 *                             cleared peripherals_gated. Fill in only the
 *                             pieces that the lean path skipped — TIM2/LEDs,
 *                             PA8 GPOUT, PA11 STAT input, the UCF reload
 *                             when MLC was kept alive, and BATTERY_Init.
 *
 * Without the "fill the gaps" branch, a motion wake immediately followed by
 * a BLE reconnect (the common case — the user grabs the device to open the
 * app) left this function as a silent early-return: TIM2 stayed dead so
 * LED_Armed flailed, INT1 stayed latched so further motion never reached the
 * EXTI handler, and the gauge wasn't re-init'd.
 *
 * LED_Off() after MX_TIM2_Reinit clamps the CCRs to 999. Without it the
 * freshly-init'd timer leaves CCR=0 (active-low → LEDs full ON) for the tens
 * of ms it takes the rest of the restore to finish.
 ***************************************************************************/
void PowerMgmt_RestoreAll(void)
{
    if (!peripherals_gated && !restore_incomplete) return;

    if (peripherals_gated) {
        /* Full bring-up from DEEPSTOP. */
        Restore_SWD();
        Restore_I2C_Bus();
        HAL_Delay(5);

        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_TIM16_CLK_ENABLE();
        MX_TIM2_Reinit();
        LED_Off();
        MX_TIM16_Reinit();
        BUZZER_Init();

        Restore_GPIO_Outputs();
        Restore_AccelInterrupt();
        Restore_CablePlugInterrupt();
        Restore_UART_Pins();

        HAL_Delay(10);
        LIS2DUX12_Init();
        LIS2DUX12_ClearMotion();
        BATTERY_Init();
    } else {
        /* RestoreForMotion already brought up I2C, TIM16, the accel/cable
         * interrupts, and SWD. Only the LED timer, the misc GPIO restores,
         * and (conditionally) the UCF reload + battery init are missing. */
        __HAL_RCC_TIM2_CLK_ENABLE();
        MX_TIM2_Reinit();
        LED_Off();

        Restore_GPIO_Outputs();
        Restore_UART_Pins();

        if (mlc_kept_alive) {
            /* Lean path skipped the SW-reset + UCF reload because the chip
             * was still classifying. INT1 has been latched HIGH since the
             * wake event, though, so without this reset the rising-edge
             * EXTI on PB15 never fires again. */
            HAL_Delay(10);
            LIS2DUX12_Init();
            LIS2DUX12_ClearMotion();
        }
        BATTERY_Init();
    }

    peripherals_gated  = 0;
    mlc_kept_alive     = 0;
    restore_incomplete = 0;
}

/***************************************************************************
 * PowerMgmt_RestoreForMotion — lean restore for accel-wake path
 *   Brings up only what an alarm needs: I2C, buzzer, accel + cable INTs.
 *   Skips TIM2/LEDs (lights stay off), BATTERY_Init (gauge runs on its own
 *   VDD), and LIS2DUX12_Init when the chip kept MLC alive across sleep.
 *
 *   restore_incomplete is set so a subsequent RestoreAll (typically from the
 *   BLE connect callback) knows to come back and finish the work that this
 *   lean path skipped.
 ***************************************************************************/
void PowerMgmt_RestoreForMotion(void)
{
    if (!peripherals_gated) return;

    Restore_SWD();
    Restore_I2C_Bus();

    __HAL_RCC_TIM16_CLK_ENABLE();
    MX_TIM16_Reinit();
    BUZZER_Init();

    Restore_AccelInterrupt();
    Restore_CablePlugInterrupt();

    if (!mlc_kept_alive) {
        HAL_Delay(10);
        LIS2DUX12_Init();
        LIS2DUX12_ClearMotion();
    }

    peripherals_gated  = 0;
    restore_incomplete = 1;
}

uint8_t PowerMgmt_IsLowPower(void)
{
    return peripherals_gated;
}

void PowerMgmt_EEPROM_PowerOn(void)
{
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
}

void PowerMgmt_EEPROM_PowerOff(void)
{
    HAL_GPIO_WritePin(EEPROM_POW_GPIO_Port, EEPROM_POW_Pin, GPIO_PIN_RESET);
}

/* ----------------------------- Boot diagnostics ---------------------------
 * Reset-cause snapshot + EEPROM-persisted boot counter, surfaced to iOS via
 * the SYSTEM section of the on-demand diagnostic dump. EEPROM offset 0x20
 * (4 bytes, LE) is inside the existing 0x000..0x03F reserved device-info
 * region (motion_logger.h), so it cannot collide with motion-log storage.
 ***************************************************************************/

#define EEPROM_BOOT_COUNT_ADDR  0x20
#define EEPROM_BOOT_COUNT_LEN   4

static uint8_t  s_reset_cause_packed = 0;
static uint8_t  s_reset_cause_captured = 0;
static uint32_t s_boot_count = 0;

/***************************************************************************
 * PowerMgmt_CaptureResetCause — must be called as the FIRST thing in main()
 *   Reads RCC->CSR's latched reset flags, packs them into a single byte
 *   (bit 0 PAD, 1 POR, 2 SFT, 3 WDG, 4 LOCKUP), then clears them so the
 *   next boot's flags are clean. Idempotent — extra calls do nothing.
 ***************************************************************************/
void PowerMgmt_CaptureResetCause(void)
{
    if (s_reset_cause_captured) return;
    s_reset_cause_captured = 1;

    uint32_t csr = RCC->CSR;
    uint8_t cause = 0;
    if (csr & RCC_CSR_PADRSTF)    cause |= (1u << 0);
    if (csr & RCC_CSR_PORRSTF)    cause |= (1u << 1);
    if (csr & RCC_CSR_SFTRSTF)    cause |= (1u << 2);
    if (csr & RCC_CSR_WDGRSTF)    cause |= (1u << 3);
    if (csr & RCC_CSR_LOCKUPRSTF) cause |= (1u << 4);
    s_reset_cause_packed = cause;

    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t PowerMgmt_GetResetCause(void)
{
    return s_reset_cause_packed;
}

/***************************************************************************
 * PowerMgmt_BootCount_Init — read, increment, write back the boot counter
 *   Call after the EEPROM is reachable (i.e. same point where Loyalty_Init
 *   runs). Failure to read or write leaves s_boot_count = 0 — iOS will see
 *   "0" and can flag it. Brackets its own EEPROM power-on/off because boot
 *   count is independent of any other persistent record.
 ***************************************************************************/
void PowerMgmt_BootCount_Init(void)
{
    M24CXX_HandleTypeDef eeprom;
    uint8_t buf[EEPROM_BOOT_COUNT_LEN] = {0};
    uint32_t value = 0;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    if (m24cxx_read(&eeprom, EEPROM_BOOT_COUNT_ADDR, buf, EEPROM_BOOT_COUNT_LEN)
            == M24CXX_Ok) {
        value = ((uint32_t)buf[0])       |
                ((uint32_t)buf[1] <<  8) |
                ((uint32_t)buf[2] << 16) |
                ((uint32_t)buf[3] << 24);
        // Treat 0xFFFFFFFF (erased EEPROM) as zero so first-ever boot ticks
        // 0 → 1 instead of wrapping.
        if (value == 0xFFFFFFFFu) value = 0;
    }

    value++;

    buf[0] = (uint8_t)(value      );
    buf[1] = (uint8_t)(value >>  8);
    buf[2] = (uint8_t)(value >> 16);
    buf[3] = (uint8_t)(value >> 24);
    (void)m24cxx_write(&eeprom, EEPROM_BOOT_COUNT_ADDR, buf, EEPROM_BOOT_COUNT_LEN);

    s_boot_count = value;

    PowerMgmt_EEPROM_PowerOff();
}

uint32_t PowerMgmt_GetBootCount(void)
{
    return s_boot_count;
}

uint32_t PowerMgmt_GetUptimeSeconds(void)
{
    return HAL_GetTick() / 1000u;
}
