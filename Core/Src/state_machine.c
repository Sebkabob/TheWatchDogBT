/***************************************************************************
 * state_machine.c
 * created by Sebastian Forenza 2026
 *
 * Central control loop. Drives the system through:
 *
 *   DISCONNECTED_IDLE → (BLE connect) → CONNECTED_IDLE
 *   CONNECTED_IDLE    → (armed)       → STABILIZING
 *   STABILIZING       → (1.75 s still)→ LOCKED
 *   STABILIZING       → (15 s elapsed) → CONNECTED_IDLE
 *   LOCKED            → (motion)      → ALARM_ACTIVE
 *   ALARM_ACTIVE      → (melody done + no motion) → LOCKED
 *
 * Cable plug / unplug events are handled in this file too:
 *   PB4 (BQ251_PG, falling-edge EXTI) sets stayAwakeFlag. On unplug,
 *   stayAwakeFlag is held for CABLE_UNPLUG_AWAKE_MS before the device
 *   is allowed back to deep sleep.
 ***************************************************************************/

#include "state_machine.h"
#include "lights.h"
#include "sound.h"
#include "battery.h"
#include "lockservice_app.h"
#include "loyalty.h"
#include "accelerometer.h"
#include "lis2dux12_app.h"
#include "motion_logger.h"
#include "power_management.h"
#include "app_ble.h"
#include "app_common.h"
#include "eeprom_map.h"

#define M24CXX_MODEL 0
#include "m24cxx.h"

extern I2C_HandleTypeDef hi2c1;

volatile SystemState_t currentState = STATE_CONNECTED_IDLE;
volatile SystemState_t previousState = STATE_CONNECTED_IDLE;
volatile uint8_t deviceState = 0;
volatile uint8_t deviceInfo = 0;
volatile uint8_t deviceBattery = 100;
volatile uint8_t connectionStatus = 0;

static uint32_t stateEntryTime = 0;

volatile uint8_t stayAwakeFlag = 0;

// Tick at which the post-connect motion-grace window expires. While the
// current tick is below this, LOCKED suppresses motion-triggered alarm
// transitions so the UCF reload from RestoreAll doesn't fire the alarm
// the moment the user picks the device up to pair it.
static volatile uint32_t motion_grace_until = 0;

void StateMachine_StartMotionGrace(uint32_t ms)
{
    uint32_t until = HAL_GetTick() + ms;
    if ((int32_t)(until - motion_grace_until) > 0) {
        motion_grace_until = until;
    }
}

static uint8_t MotionGrace_Active(void)
{
    return (int32_t)(motion_grace_until - HAL_GetTick()) > 0;
}

/* In-progress motion bout tracker — file scope so StateMachine_ChangeState
 * can clear it on disarm. Used to be a function-static in State_Locked_Loop,
 * which meant an in-flight bout would survive ALARM_ACTIVE → CONNECTED_IDLE
 * (user unlocks mid-alarm) and re-emerge on the next re-arm: the eventual
 * MLC-settle in LOCKED, or the 60 s force-flush, would log a bogus event
 * whose duration spanned alarm-start → re-arm. Now any transition to a
 * disarmed state drops the pending bout. */
static uint8_t      s_motion_pending      = 0;
static MotionType_t s_pending_type        = MOTION_TYPE_NONE;
static uint32_t     s_motion_pending_tick = 0;

static void MotionPending_Reset(void)
{
    s_motion_pending      = 0;
    s_pending_type        = MOTION_TYPE_NONE;
    s_motion_pending_tick = 0;
}

static uint32_t cableUnplugTime = 0;
static uint8_t  cableWasPlugged = 0;

static volatile uint8_t findMyActive = 0;

/***************************************************************************
 * LED_ChargingPulse — red→yellow→green pulse based on cached SOC
 *   0 % = pure red, 50 % = yellow, 100 % = pure green.
 ***************************************************************************/
static void LED_ChargingPulse(void)
{
    uint16_t soc = BATTERY_GetSOC();
    if (soc > 100) soc = 100;

    uint8_t r, g;
    if (soc <= 50) {
        r = 255;
        g = (uint8_t)((soc * 255) / 50);
    } else {
        r = (uint8_t)(((100 - soc) * 255) / 50);
        g = 255;
    }
    LED_Pulse(4000, r, g, 0, 255);
}

/***************************************************************************
 * CablePlug_IRQCallback — called from GPIOB IRQ when PB4 falls (cable in)
 *   Safe from interrupt context. Sets stayAwakeFlag so the main loop can
 *   restore peripherals and show charging status.
 ***************************************************************************/
void CablePlug_IRQCallback(void)
{
    stayAwakeFlag = 1;
}

/***************************************************************************
 * StateMachine_Init — set defaults: LOW sens, CALM alarm, lights+logging on
 ***************************************************************************/
void StateMachine_Init(void)
{
    currentState = STATE_DISCONNECTED_IDLE;
    stateEntryTime = HAL_GetTick();

    deviceState = 0;
    SET_ARMED_BIT(deviceState, 0);
    SET_ALARM_TYPE(deviceState, ALARM_CALM);
    SET_SENSITIVITY(deviceState, SENSITIVITY_LOW);
    SET_LIGHTS_BIT(deviceState, 1);
    SET_LOGGING_BIT(deviceState, 1);
    SET_SILENCE_BIT(deviceState, 0);

    deviceInfo = 0;

    cableWasPlugged = IS_CABLE_PLUGGED() ? 1 : 0;
    cableUnplugTime = 0;
}

/***************************************************************************
 * CablePlug_UpdateState — edge-detect cable + manage post-unplug awake window
 *   Drives the loyalty reset window on debounced VBUS edges: rising edge
 *   opens it (CLAIM may overwrite the EEPROM token for ~10 s), falling edge
 *   closes it. The 50 ms debounce prevents an insertion bounce from racking
 *   up multiple Start/Cancel cycles. Boot-with-VBUS-already-high is handled
 *   inside Loyalty_Init, not here, since this function never sees that edge.
 ***************************************************************************/
#define CABLE_EDGE_DEBOUNCE_MS  50u

// Cap on how long STABILIZING will pulse blue waiting for stillness before
// bailing back to CONNECTED_IDLE. Without this the device sits forever if
// motion never settles; the fall-back un-arms via the ARMED-bit clear in
// StateMachine_ChangeState.
#define STABILIZE_TIMEOUT_MS    15000u


static void CablePlug_UpdateState(void)
{
    static uint32_t last_edge_ms = 0;
    uint8_t pluggedNow = IS_CABLE_PLUGGED() ? 1 : 0;

    if (pluggedNow) {
        if (!cableWasPlugged) {
            uint32_t now = HAL_GetTick();
            if ((now - last_edge_ms) >= CABLE_EDGE_DEBOUNCE_MS) {
                Loyalty_StartResetWindow();
                last_edge_ms = now;
            }
            LED_PlugIn_Start();
        }
        stayAwakeFlag = 1;
        cableUnplugTime = 0;
        cableWasPlugged = 1;
        return;
    }

    if (cableWasPlugged) {
        uint32_t now = HAL_GetTick();
        if ((now - last_edge_ms) >= CABLE_EDGE_DEBOUNCE_MS) {
            Loyalty_CancelResetWindow();
            last_edge_ms = now;
        }
        cableUnplugTime = now;
        cableWasPlugged = 0;
        LED_PlugOut_Start();
    }

    if (cableUnplugTime != 0) {
        if ((HAL_GetTick() - cableUnplugTime) < CABLE_UNPLUG_AWAKE_MS) {
            stayAwakeFlag = 1;
        } else {
            // Window expired. Don't touch stayAwakeFlag — the state loop
            // will re-decide based on the current state.
            cableUnplugTime = 0;
        }
    }
}

void State_Disconnected_Idle_Loop(void)
{
    if (IS_CABLE_PLUGGED()) {
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        stayAwakeFlag = 1;

        if (LED_PlugIn_InProgress()) {
            LED_PlugIn_Tick();
        } else if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
            LED_ChargingPulse();
        } else if (BATTERY_IsFullCached()) {
            LED_Solid(0, 255, 0, 255);
        } else {
            LED_Off();
        }
    } else {
        if (cableUnplugTime == 0) {
            stayAwakeFlag = 0;
        }

        if (LED_PlugOut_InProgress()) {
            LED_PlugOut_Tick();
        } else {
            LED_Off();
        }

        if (!stayAwakeFlag && !PowerMgmt_IsLowPower()) {
            PowerMgmt_EnterLowPower_Idle();
        }
    }

    if (connectionStatus) {
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
    }
}

void State_Connected_Idle_Loop(void)
{
    stayAwakeFlag = 1;

    if (!findMyActive) {
        if (IS_CABLE_PLUGGED()) {
            if (LED_PlugIn_InProgress()) {
                LED_PlugIn_Tick();
            } else if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
                LED_ChargingPulse();
            } else if (BATTERY_IsFullCached()) {
                LED_Solid(0, 255, 0, 255);
            } else {
                LED_Off();
            }
        } else if (LED_PlugOut_InProgress()) {
            LED_PlugOut_Tick();
        } else if (GET_LIGHTS_BIT(deviceState)) {
            LED_Rainbow(10, LedBrightness_Get());
        } else {
            LED_Off();
        }
    }

    if (!connectionStatus) {
        StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
    }

    if (GET_ARMED_BIT(deviceState)) {
        LIS2DUX12_ClearMotion();
        StateMachine_ChangeState(STATE_STABILIZING);
    }
}

/***************************************************************************
 * State_Stabilizing_Loop — wait for 1.75 s of stillness before locking
 *   Pulsing blue LED while waiting; both the MLC interrupt and a 10 Hz
 *   poll reset the still-timer when motion is detected. Bails to
 *   CONNECTED_IDLE after STABILIZE_TIMEOUT_MS so a never-settling device
 *   can't pulse blue forever.
 ***************************************************************************/
void State_Stabilizing_Loop(void)
{
    static uint32_t last_still_time = 0;
    static uint32_t stabilize_entry_time = 0;
    static uint8_t  stabilize_started = 0;

    // STABILIZING is only reachable from CONNECTED_IDLE, which already sets
    // this every iteration — but re-asserting here keeps the invariant
    // "stayAwakeFlag is 1 whenever currentState implies a live BLE session"
    // locally provable. See the comment in State_Locked_Loop's connected
    // branch for the chip-level DEEPSTOP-between-events failure mode this
    // flag exists to suppress.
    stayAwakeFlag = 1;

    if (!GET_ARMED_BIT(deviceState)) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState)) {
            LED_Pulse(1000, 0, 0, 255, LedBrightness_Get());
        } else {
            LED_Off();
        }
    }

    if (!stabilize_started) {
        last_still_time = HAL_GetTick();
        stabilize_entry_time = HAL_GetTick();
        stabilize_started = 1;
        LIS2DUX12_ClearMotion();
    }

    if (LIS2DUX12_IsMotionDetected()) {
        uint8_t mlc_out;
        lis2dux12_app_get_mlc_output(&mlc_out);
        if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
            last_still_time = HAL_GetTick();
        }
    }

    static uint32_t last_poll = 0;
    if (HAL_GetTick() - last_poll >= 100) {
        last_poll = HAL_GetTick();
        uint8_t mlc_out;
        if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
                last_still_time = HAL_GetTick();
            }
        }
    }

    if (HAL_GetTick() - last_still_time >= 1750) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_LOCKED);
        return;
    }

    if (HAL_GetTick() - stabilize_entry_time >= STABILIZE_TIMEOUT_MS) {
        stabilize_started = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }
}

/***************************************************************************
 * State_Locked_Loop — main armed loop, including LP wake-from-motion path
 *
 * Wake-from-LP behaviour (HIGH-sens path keeps MLC alive across sleep):
 *   - INT alone never fires the alarm.
 *   - Read MLC + FSM first; if MLC says IN_MOTION/SHAKEN seed the deferred
 *     alert and transition to ALARM_ACTIVE.
 *   - If MLC still says STATIONARY (a "breeze" wake), motion_assessing
 *     keeps peripherals up for ~10 s while the regular MLC/FSM loop
 *     watches for an escalation.
 *
 * Deferred alert: when MLC says IN_MOTION/SHAKEN we trigger the alarm
 * immediately but DEFER the log/alert until MLC settles, so the door
 * detector can override IN_MOTION with DOOR_OPENED if rotation crossed
 * the threshold. Impact and freefall (FSM) always fire immediately.
 ***************************************************************************/
/* Bout-end timeout. Bumped from 3 s to 60 s when duration tracking was
 * added — a 3 s ceiling would chop every sustained shake into a string of
 * "3 s of motion" entries and never let the user see the true bout length.
 * 60 s × 1000 / 250 = 240 ticks of 250 ms — still inside the iOS protocol's
 * 1-byte duration field (max 255). The alarm itself is unaffected by this
 * timeout; it fires immediately at the qualification site below. */
#define MOTION_BOUT_TIMEOUT_MS 60000u

/* Clamp a HAL_GetTick delta (ms) to a uint8_t count of 250 ms ticks for the
 * wire format. Floor of 1 keeps "instantaneous" bouts (FSM events, single-
 * sample notifies) from being indistinguishable from "unknown" in iOS. */
static uint8_t bout_ticks_250ms_from_ms(uint32_t elapsed_ms)
{
    uint32_t ticks = elapsed_ms / 250u;
    if (ticks < 1)   ticks = 1;
    if (ticks > 255) ticks = 255;
    return (uint8_t)ticks;
}

/* Severity ordering for in-bout type promotion: a bout that begins as
 * IN_MOTION but later sees a SHAKEN classification should log as SHAKEN.
 * IMPACT and FREEFALL are FSM events that bypass motion_pending entirely,
 * so they don't participate in promotion. */
static uint8_t motion_type_severity(MotionType_t t)
{
    switch (t) {
        case MOTION_TYPE_SHAKEN:   return 2;
        case MOTION_TYPE_IN_MOTION:return 1;
        default:                   return 0;
    }
}

/* -------- Per-sensitivity magnitude debounce (N-of-M filtering) ----------
 * Two filters make the LOW / MEDIUM / HIGH tiers feel meaningfully different:
 *
 *   1. Per-tier magnitude threshold (mg_threshold). A sample only counts as
 *      a "hit" when ||a|| deviates from 1 g by more than this many mg.
 *        LOW    500 mg  (vigorous lift / drop / shake)
 *        MED    250 mg  (moderate disturbance)
 *        HIGH   100 mg  (gentle bump)
 *
 *   2. Per-tier N-of-M debounce (required_hits / window_size). The alarm
 *      only fires when the last M sample slots contain ≥ K hits. At the
 *      100 ms poll cadence this gives:
 *        LOW    K=8 / M=10  → ~800 ms of sustained motion to fire
 *        MED    K=4 / M=10  → ~400 ms
 *        HIGH   K=1 / M=1   → single-sample, effectively instant
 *
 * SHAKEN, IMPACT and FREEFALL bypass the debounce — those are unambiguous
 * physical events and waiting for K-of-M would defeat the point.
 *
 * Ring storage is a 16-bit sliding window. push() shifts left, popcount()
 * over the configured window_size bits decides the trigger. Reset on entry
 * to STATE_LOCKED, on alarm-loop completion (re-entry to LOCKED), and on
 * motion_assessing timeout so a near-miss bout can't carry stale hits into
 * the next wake.
 ***************************************************************************/

typedef struct {
    int16_t mg_threshold;
    uint8_t required_hits;
    uint8_t window_size;
} sens_motion_config_t;

static const sens_motion_config_t SENS_CFG[3] = {
    /* [SENSITIVITY_LOW]    */ { 500, 8, 10 },
    /* [SENSITIVITY_MEDIUM] */ { 250, 4, 10 },
    /* [SENSITIVITY_HIGH]   */ { 100, 1,  1 },
};

static uint16_t motion_sample_ring = 0;

static uint8_t motion_magnitude_hit(int16_t threshold_mg)
{
    int16_t ax = 0, ay = 0, az = 0;
    if (lis2dux12_app_read_accel_mg(&ax, &ay, &az) != 0) return 0;
    int32_t mag2 = (int32_t)ax * ax + (int32_t)ay * ay + (int32_t)az * az;
    int32_t lo  = 1000 - threshold_mg; if (lo < 0) lo = 0;
    int32_t hi  = 1000 + threshold_mg;
    int32_t lo2 = lo * lo;
    int32_t hi2 = hi * hi;
    return (mag2 > hi2 || mag2 < lo2) ? 1u : 0u;
}

static void motion_ring_push(uint8_t hit)
{
    motion_sample_ring = (uint16_t)((motion_sample_ring << 1) | (hit & 1u));
}

static uint8_t motion_ring_popcount(uint8_t window)
{
    uint16_t mask = (window >= 16) ? 0xFFFFu : (uint16_t)((1u << window) - 1u);
    uint16_t v = (uint16_t)(motion_sample_ring & mask);
    uint8_t count = 0;
    while (v) { count = (uint8_t)(count + (v & 1u)); v >>= 1; }
    return count;
}

static void motion_ring_reset(void)
{
    motion_sample_ring = 0;
}

/* Take a fresh accel sample with the current tier's threshold, push the
 * hit/miss bit, and return 1 iff the ring now satisfies K-of-M. */
static uint8_t motion_sample_and_check(void)
{
    uint8_t sens = GET_SENSITIVITY(deviceState);
    if (sens > SENSITIVITY_HIGH) sens = SENSITIVITY_HIGH;
    const sens_motion_config_t *cfg = &SENS_CFG[sens];
    motion_ring_push(motion_magnitude_hit(cfg->mg_threshold));
    return (motion_ring_popcount(cfg->window_size) >= cfg->required_hits) ? 1u : 0u;
}

/* Force-push a hit (e.g. MLC IN_MOTION INT outside the 100 ms poll). On HIGH
 * a single MLC IN_MOTION INT already satisfies K=1/M=1; on MED/LOW it just
 * primes the ring and the next polls have to confirm with magnitude. */
static uint8_t motion_force_hit_and_check(void)
{
    uint8_t sens = GET_SENSITIVITY(deviceState);
    if (sens > SENSITIVITY_HIGH) sens = SENSITIVITY_HIGH;
    const sens_motion_config_t *cfg = &SENS_CFG[sens];
    motion_ring_push(1);
    return (motion_ring_popcount(cfg->window_size) >= cfg->required_hits) ? 1u : 0u;
}

/* -------- XYZ-range verifier (anti-buzzer-feedback) ----------------------
 * The MLC alone can be fooled by the loud-alarm pattern's own vibration
 * coupling back through the chassis: the buzzer is loud enough that the
 * accelerometer reads IN_MOTION continuously, which keeps refreshing
 * last_motion_time in ALARM_ACTIVE and locks the alarm on forever. The
 * MLC also misfires on table-bumps the user doesn't consider real motion.
 *
 * The verifier tracks raw X/Y/Z samples over a sliding window (8 samples
 * at the 100 ms poll cadence = ~800 ms of history) and reports the max
 * per-axis range (max - min) over that window. Real motion shifts the
 * gravity vector (pickup, tilt, walk) → large range. Buzzer-induced
 * vibration oscillates symmetrically around a fixed equilibrium → small
 * range. Per-sensitivity threshold:
 *
 *   LOW    600 mg of axis range — only vigorous motion counts
 *   MED    350 mg
 *   HIGH   150 mg — gentle moves still count
 *
 * The verifier confirms iff any axis range >= threshold AND the window
 * holds at least RING_SIZE/2 samples (so a single sample right after a
 * reset can't claim confirmation — its range is structurally 0). Used
 * as an AND gate alongside the MLC classification at both:
 *   - the alarm-trigger sites in LOCKED (so a MLC misfire on a table-bump
 *     won't fire the alarm without sustained XYZ displacement), and
 *   - the alarm-refresh path in ALARM_ACTIVE (so the buzzer's own
 *     coupling-feedback can't keep refreshing last_motion_time).
 *
 * FSM impact/freefall are NOT gated — those are unambiguous instantaneous
 * physical events worth honouring directly. Only MLC-class-driven paths
 * route through this verifier.
 ***************************************************************************/
#define MOTION_VERIFIER_RING_SIZE 8u

typedef struct {
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
} verifier_sample_t;

static const int16_t SENS_VERIFY_RANGE_MG[3] = {
    /* [SENSITIVITY_LOW]    */ 600,
    /* [SENSITIVITY_MEDIUM] */ 350,
    /* [SENSITIVITY_HIGH]   */ 150,
};

static verifier_sample_t s_verifier_ring[MOTION_VERIFIER_RING_SIZE];
static uint8_t           s_verifier_count = 0;
static uint8_t           s_verifier_idx   = 0;

static void motion_verifier_reset(void)
{
    s_verifier_count = 0;
    s_verifier_idx   = 0;
}

/* Read raw X/Y/Z mg and push into the ring. Returns 1 on success, 0 if the
 * I2C read failed (the caller can retry on the next poll). */
static uint8_t motion_verifier_sample(void)
{
    int16_t ax = 0, ay = 0, az = 0;
    if (lis2dux12_app_read_accel_mg(&ax, &ay, &az) != 0) {
        return 0;
    }
    s_verifier_ring[s_verifier_idx].x_mg = ax;
    s_verifier_ring[s_verifier_idx].y_mg = ay;
    s_verifier_ring[s_verifier_idx].z_mg = az;
    s_verifier_idx = (uint8_t)((s_verifier_idx + 1u) % MOTION_VERIFIER_RING_SIZE);
    if (s_verifier_count < MOTION_VERIFIER_RING_SIZE) {
        s_verifier_count++;
    }
    return 1u;
}

/* Returns 1 iff any axis (max - min) over the ring meets the per-sensitivity
 * threshold AND the ring holds at least RING_SIZE/2 samples. */
static uint8_t motion_verifier_confirms(void)
{
    if (s_verifier_count < (MOTION_VERIFIER_RING_SIZE / 2u)) {
        return 0u;
    }
    int16_t xmin = s_verifier_ring[0].x_mg, xmax = xmin;
    int16_t ymin = s_verifier_ring[0].y_mg, ymax = ymin;
    int16_t zmin = s_verifier_ring[0].z_mg, zmax = zmin;
    for (uint8_t i = 1; i < s_verifier_count; i++) {
        int16_t x = s_verifier_ring[i].x_mg;
        int16_t y = s_verifier_ring[i].y_mg;
        int16_t z = s_verifier_ring[i].z_mg;
        if (x < xmin) xmin = x; if (x > xmax) xmax = x;
        if (y < ymin) ymin = y; if (y > ymax) ymax = y;
        if (z < zmin) zmin = z; if (z > zmax) zmax = z;
    }
    int32_t rx = (int32_t)xmax - (int32_t)xmin;
    int32_t ry = (int32_t)ymax - (int32_t)ymin;
    int32_t rz = (int32_t)zmax - (int32_t)zmin;
    int32_t rmax = rx;
    if (ry > rmax) rmax = ry;
    if (rz > rmax) rmax = rz;

    uint8_t sens = GET_SENSITIVITY(deviceState);
    if (sens > SENSITIVITY_HIGH) sens = SENSITIVITY_HIGH;
    return (rmax >= (int32_t)SENS_VERIFY_RANGE_MG[sens]) ? 1u : 0u;
}

void State_Locked_Loop(void)
{
    static uint8_t  motion_assessing = 0;
    static uint32_t motion_assess_start = 0;
    #define MOTION_ASSESS_TIMEOUT_MS 10000

    /* Bout tracker lives at file scope now (see s_motion_pending_*). These
     * aliases preserve the local-variable readability of the loop body. */
    #define motion_pending       s_motion_pending
    #define pending_type         s_pending_type
    #define motion_pending_tick  s_motion_pending_tick

    if (!GET_ARMED_BIT(deviceState)) {
        motion_assessing = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        LED_Off();
        return;
    }

    if (PowerMgmt_IsLowPower() && LIS2DUX12_PeekMotionStatus()) {
        PowerMgmt_RestoreForMotion();
        LIS2DUX12_ClearMotion();
        stayAwakeFlag = 1;
        motion_assessing = 1;
        motion_assess_start = HAL_GetTick();

        // ONE log per wake. The prior firmware logged MOTION_TYPE_IN_MOTION
        // unconditionally here ("up-front so brief motions still recorded"),
        // then logged a SECOND event below for the FSM impact/freefall
        // branch, plus eventually a third via motion_pending → settle. That
        // tripled every wake's footprint in the ring. Track whether the
        // classifier produced a qualifying log; only fall back to a generic
        // IN_MOTION at the end if it did not.
        uint8_t wake_handled = 0;

        // Per-tier significant-motion fast-path. The wake itself is one
        // sample for the debounce ring; on HIGH (K=1/M=1) a single hit over
        // the 100 mg threshold fires immediately, on MED/LOW one sample is
        // never enough on its own (K=4/8 of M=10) so the device just stays
        // in motion_assessing and lets the 100 ms poll loop accumulate more
        // samples. The XYZ-range verifier is also reset here — its ring
        // fills over the 100 ms poll that follows; the wake's single
        // sample structurally can't confirm on its own.
        motion_ring_reset();
        motion_verifier_reset();
        uint8_t fast_fired = 0;
        if (motion_sample_and_check()) {
            /* Magnitude debounce passed; verifier may not yet have enough
             * samples (just reset) to confirm. If it can't confirm, keep
             * motion_assessing alive so the 100 ms poll path can fill the
             * verifier ring and fire then if motion is real and sustained. */
            (void)motion_verifier_sample();
            if (motion_verifier_confirms()) {
                motion_assessing = 0;
                /* motion_pending drives the bout-settle log on return to LOCKED.
                 * Only arm it when the alarm actually transitions — silent or
                 * filtered detections must NOT produce a log entry. */
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    if (!motion_pending) {
                        motion_pending      = 1;
                        pending_type        = MOTION_TYPE_IN_MOTION;
                        motion_pending_tick = HAL_GetTick();
                    }
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
                fast_fired = 1;
                wake_handled = 1;
            }
        }

        uint8_t mlc_out;
        if (!fast_fired && lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            lis2dux12_app_update_cached_state(mlc_out);

            uint8_t impact = 0, freefall = 0;
            lis2dux12_app_check_fsm_events(&impact, &freefall);
            if (impact || freefall) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                motion_assessing = 0;
                if (GET_LOGGING_BIT(deviceState)) {
                    /* FSM events are inherently instantaneous — duration=1
                     * tick is the "happened, no measurable length" sentinel. */
                    MotionLogger_LogEvent(mt, 1);
                    LOCKSERVICE_SendMotionAlert(mt, 1);
                }
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
                wake_handled = 1;
            } else if (mlc_out == MLC_STATE_IN_MOTION ||
                       mlc_out == MLC_STATE_SHAKEN) {
                MotionType_t observed = (mlc_out == MLC_STATE_SHAKEN)
                    ? MOTION_TYPE_SHAKEN : MOTION_TYPE_IN_MOTION;
                /* MLC says motion — but the verifier still has to agree.
                 * SHAKEN no longer bypasses: the buzzer's own vibration can
                 * trip MLC SHAKEN during ALARM_ACTIVE feedback, and a hard
                 * table-thump can pop SHAKEN in LOCKED without the device
                 * actually moving. Both must show sustained XYZ
                 * displacement to fire. IN_MOTION additionally runs through
                 * the existing per-tier debounce. At wake time the
                 * verifier ring is fresh and won't confirm yet — that's
                 * intentional; motion_assessing keeps the device awake and
                 * the 100 ms poll path takes over. */
                (void)motion_verifier_sample();
                uint8_t verified = motion_verifier_confirms();
                uint8_t debounce = (mlc_out == MLC_STATE_SHAKEN)
                                          ? 1u : motion_force_hit_and_check();
                uint8_t should_fire = verified && debounce;
                if (should_fire) {
                    /* Bout tracking + log only when the alarm actually fires.
                     * Without this gate, every MLC IN_MOTION INT that the
                     * debounce filtered out would still seed motion_pending
                     * and produce a "log says alarm went off" entry on settle. */
                    if (!motion_pending) {
                        motion_pending      = 1;
                        pending_type        = observed;
                        motion_pending_tick = HAL_GetTick();
                    } else if (motion_type_severity(observed) > motion_type_severity(pending_type)) {
                        /* Promote: bout escalated mid-flight (IN_MOTION → SHAKEN). */
                        pending_type = observed;
                    }
                    motion_assessing = 0;
                    if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                        StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                    }
                }
                wake_handled = 1;
            }
        }

        /* No fallback log on bare LP wake. Under the per-tier debounce
         * (MED/LOW), most LP wakes intentionally don't qualify — the
         * single sample available at wake time can't reach K-of-M. The
         * old "wake is evidence of motion" log produced a spurious
         * IN_MOTION entry for every accel INT, which iOS rendered as
         * "the alarm went off" even though nothing fired. If the motion
         * is real and sustained, the 100 ms poll loop will reach K-of-M
         * and the bout-settle path will log it correctly with full
         * duration. */
        (void)wake_handled;
    }

    if (!findMyActive) {
        if (GET_LIGHTS_BIT(deviceState) && !PowerMgmt_IsLowPower()) {
            LED_Armed(10, LedBrightness_Get());
        } else {
            LED_Off();
        }
    }

    if (!PowerMgmt_IsLowPower() && !MotionGrace_Active()) {

        if (LIS2DUX12_IsMotionDetected()) {
            uint8_t mlc_out;
            lis2dux12_app_get_mlc_output(&mlc_out);
            lis2dux12_app_update_cached_state(mlc_out);

            uint8_t impact, freefall;
            lis2dux12_app_check_fsm_events(&impact, &freefall);
            if (impact || freefall) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                stayAwakeFlag = 1;
                motion_assessing = 0;
                if (GET_LOGGING_BIT(deviceState)) {
                    MotionLogger_LogEvent(mt, 1);
                    LOCKSERVICE_SendMotionAlert(mt, 1);
                }
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
            }

            if (mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN) {
                MotionType_t observed = (mlc_out == MLC_STATE_SHAKEN)
                    ? MOTION_TYPE_SHAKEN : MOTION_TYPE_IN_MOTION;
                stayAwakeFlag = 1;
                /* MLC class change INT — verify with the XYZ-range tracker.
                 * The 100 ms poll below keeps the verifier ring filled with
                 * recent samples, so an INT that lands during active LOCKED
                 * usually has a populated window to consult. If the verifier
                 * doesn't confirm (table bump, MLC false positive), the
                 * alarm stays silent. */
                (void)motion_verifier_sample();
                uint8_t verified = motion_verifier_confirms();
                uint8_t debounce = (mlc_out == MLC_STATE_SHAKEN)
                                          ? 1u : motion_force_hit_and_check();
                uint8_t should_fire = verified && debounce;
                if (should_fire) {
                    /* Only arm motion_pending (the bout-settle log driver)
                     * when the alarm actually fires. Filtered INTs must NOT
                     * leave a log entry behind. */
                    if (!motion_pending) {
                        motion_pending = 1;
                        pending_type = observed;
                        motion_pending_tick = HAL_GetTick();
                    } else if (motion_type_severity(observed) > motion_type_severity(pending_type)) {
                        pending_type = observed;
                    }
                    motion_assessing = 0;
                    if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                        StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                    }
                }
            }

            if (mlc_out == MLC_STATE_STATIONARY_UPRIGHT ||
                mlc_out == MLC_STATE_STATIONARY_NOT_UPRIGHT) {
                if (motion_pending) {
                    if (GET_LOGGING_BIT(deviceState)) {
                        uint8_t dur = bout_ticks_250ms_from_ms(
                            HAL_GetTick() - motion_pending_tick);
                        MotionLogger_LogEvent(pending_type, dur);
                        LOCKSERVICE_SendMotionAlert(pending_type, dur);
                    }
                    motion_pending = 0;
                }
            }
        }

        /* 60 s bout cap. Was 3 s — bumped to give duration tracking room to
         * actually measure sustained bouts. Flushes the pending bout when
         * MLC never settles (e.g. continuous carrying / sustained shake). */
        if (motion_pending && (HAL_GetTick() - motion_pending_tick > MOTION_BOUT_TIMEOUT_MS)) {
            if (GET_LOGGING_BIT(deviceState)) {
                uint8_t dur = bout_ticks_250ms_from_ms(
                    HAL_GetTick() - motion_pending_tick);
                MotionLogger_LogEvent(pending_type, dur);
                LOCKSERVICE_SendMotionAlert(pending_type, dur);
            }
            motion_pending = 0;
        }

        static uint32_t last_poll = 0;
        if (HAL_GetTick() - last_poll >= 100) {
            last_poll = HAL_GetTick();

            // Poll the MLC class alongside FSM. The INT-driven path above
            // only sees MLC *transitions* (rising edge of INT1 on a class
            // change). If MLC transitioned to IN_MOTION/SHAKEN during a
            // window where the INT was consumed without acting on it — the
            // 2 s post-reconnect motion-grace drain is the canonical case:
            // motion that starts in-grace and persists past it leaves MLC
            // latched in IN_MOTION with no further edge to wake the active
            // branch — the alarm never fires until MLC settles back to
            // STATIONARY, by which time mlc_out no longer qualifies.
            // Polling here closes that gap and also covers any missed INT
            // (chip glitch, brief EXTI masking during a peripheral reinit).
            uint8_t mlc_out;
            if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
                lis2dux12_app_update_cached_state(mlc_out);

                /* Feed the debounce ring every poll. A "hit" is either a
                 * magnitude sample over the per-tier mg threshold OR an
                 * MLC class of IN_MOTION/SHAKEN this tick (both sources
                 * are OR'd so vibration noise that the MLC misses can
                 * still trip LOW, and an MLC IN_MOTION classification
                 * that the magnitude reading just barely fails on the
                 * given tick still accumulates). One push per 100 ms keeps
                 * the M window's time meaning consistent: at K=8/M=10 LOW
                 * fires only after ~800 ms of sustained motion. */
                uint8_t sens = GET_SENSITIVITY(deviceState);
                if (sens > SENSITIVITY_HIGH) sens = SENSITIVITY_HIGH;
                const sens_motion_config_t *cfg = &SENS_CFG[sens];
                uint8_t mlc_moving = (mlc_out == MLC_STATE_IN_MOTION ||
                                      mlc_out == MLC_STATE_SHAKEN);
                uint8_t mag_hit    = motion_magnitude_hit(cfg->mg_threshold);
                motion_ring_push((mlc_moving || mag_hit) ? 1u : 0u);
                uint8_t debounce_satisfied =
                    (motion_ring_popcount(cfg->window_size) >= cfg->required_hits)
                        ? 1u : 0u;

                /* Feed the XYZ-range verifier every poll. Sampling
                 * continuously keeps the window fresh so an INT that
                 * lands between polls has populated history to consult.
                 * The reads themselves are cheap (~1 ms I2C). */
                (void)motion_verifier_sample();
                uint8_t verified = motion_verifier_confirms();

                if (mlc_out == MLC_STATE_IN_MOTION ||
                    mlc_out == MLC_STATE_SHAKEN) {
                    MotionType_t observed = (mlc_out == MLC_STATE_SHAKEN)
                        ? MOTION_TYPE_SHAKEN : MOTION_TYPE_IN_MOTION;
                    stayAwakeFlag = 1;
                    /* MLC must be confirmed by XYZ displacement. SHAKEN
                     * still bypasses the per-tier K-of-M (it's an
                     * unambiguous classification on its own) but the
                     * verifier gate is now non-negotiable for both
                     * classes — buzzer feedback can paint SHAKEN
                     * continuously during ALARM_ACTIVE, and we don't
                     * want LOCKED → ALARM_ACTIVE bounce on a
                     * shake-shaped table thump that didn't move the
                     * device. */
                    uint8_t debounce_pass =
                        (mlc_out == MLC_STATE_SHAKEN) ? 1u : debounce_satisfied;
                    uint8_t should_fire = verified && debounce_pass;
                    if (should_fire) {
                        /* Only seed motion_pending when the alarm fires,
                         * so the eventual bout-settle log corresponds to
                         * an actual alarm event. */
                        if (!motion_pending) {
                            motion_pending = 1;
                            pending_type = observed;
                            motion_pending_tick = HAL_GetTick();
                        } else if (motion_type_severity(observed) > motion_type_severity(pending_type)) {
                            pending_type = observed;
                        }
                        motion_assessing = 0;
                        if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                            StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                        }
                    }
                } else if (debounce_satisfied && verified) {
                    /* Magnitude-only fire: MLC hasn't classified IN_MOTION
                     * (could be lag, a noise pattern outside its training,
                     * or MLC wiped in MED/LOW LP wake) but K-of-M over the
                     * raw accel + the XYZ-range verifier both agree we're
                     * moving. Treat as IN_MOTION. */
                    stayAwakeFlag = 1;
                    motion_assessing = 0;
                    if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                        if (!motion_pending) {
                            motion_pending = 1;
                            pending_type = MOTION_TYPE_IN_MOTION;
                            motion_pending_tick = HAL_GetTick();
                        }
                        StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                    }
                } else if (motion_pending &&
                           (mlc_out == MLC_STATE_STATIONARY_UPRIGHT ||
                            mlc_out == MLC_STATE_STATIONARY_NOT_UPRIGHT)) {
                    /* Settle detection in the poll loop, not just on INT.
                     * Many UCFs only fire INT1 on motion-onset transitions,
                     * not on the return to STATIONARY. Without this check,
                     * a brief bout would sit unflushed until the 60 s safety
                     * timeout — which the user perceives as "events don't
                     * show up." Catching the settle in the 100 ms poll flushes
                     * the bout within ~100-200 ms of motion actually ending. */
                    if (GET_LOGGING_BIT(deviceState)) {
                        uint8_t dur = bout_ticks_250ms_from_ms(
                            HAL_GetTick() - motion_pending_tick);
                        MotionLogger_LogEvent(pending_type, dur);
                        LOCKSERVICE_SendMotionAlert(pending_type, dur);
                    }
                    motion_pending = 0;
                }
            }

            uint8_t impact, freefall;
            lis2dux12_app_check_fsm_events(&impact, &freefall);
            if (impact || freefall) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
                stayAwakeFlag = 1;
                motion_assessing = 0;
                if (GET_LOGGING_BIT(deviceState)) {
                    MotionLogger_LogEvent(mt, 1);
                    LOCKSERVICE_SendMotionAlert(mt, 1);
                }
                if (!GET_SILENCE_BIT(deviceState) || !connectionStatus) {
                    StateMachine_ChangeState(STATE_ALARM_ACTIVE);
                }
            }
        }

    } else if (!PowerMgmt_IsLowPower() && MotionGrace_Active()) {
        // During the post-connect grace window, drain any motion the UCF
        // reload + user handling produced so it doesn't latch and fire the
        // alarm the instant grace ends.
        if (LIS2DUX12_IsMotionDetected()) {
            uint8_t mlc_out;
            lis2dux12_app_get_mlc_output(&mlc_out);
            lis2dux12_app_update_cached_state(mlc_out);
        }
        uint8_t impact = 0, freefall = 0;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        (void)impact; (void)freefall;
        motion_pending = 0;
    }

    if (!connectionStatus) {
        LED_Off();

        if (motion_assessing) {
            if (HAL_GetTick() - motion_assess_start >= MOTION_ASSESS_TIMEOUT_MS) {
                motion_assessing = 0;
                stayAwakeFlag = 0;
                /* Clear the debounce ring so stale near-miss hits from this
                 * bout don't carry into the next wake — would otherwise let
                 * a second mild bout cross K-of-M faster than it should.
                 * Same reasoning for the XYZ-range verifier. */
                motion_ring_reset();
                motion_verifier_reset();
            }
        } else {
            if (cableUnplugTime == 0 && !IS_CABLE_PLUGGED()) {
                stayAwakeFlag = 0;
            }
        }

        // Guard LP re-entry on currentState still being LOCKED. Without this,
        // a transition to STATE_ALARM_ACTIVE earlier in this same call (LP-
        // wake classify path or active-polling path) falls through to here
        // and re-gates TIM16/LEDs. The chip then DEEPSTOPs while currentState
        // is ALARM_ACTIVE; the next motion INT wakes Alarm_Active_Loop, which
        // RestoreAll's, starts the buzzer for a few ms, then exits to LOCKED
        // because the freshly-reloaded MLC reports STATIONARY (hasn't had
        // time to classify yet) and alarm_duration_ms gates accept the exit.
        // Result: ~5 ms click on every accel INT (~ once per 2 s), no real
        // alarm, no motion log past the wake's fallback IN_MOTION entry.
        if (!stayAwakeFlag && !PowerMgmt_IsLowPower() && currentState == STATE_LOCKED) {
            motion_pending = 0;
            PowerMgmt_EnterLowPower_Armed();
        }
    } else {
        if (PowerMgmt_IsLowPower()) {
            PowerMgmt_RestoreAll();
        }
        motion_assessing = 0;

        // stayAwakeFlag gates the BLE stack's chip-level LPM in
        // app_entry.c::App_PowerSaveLevel_Check. With it clear AND a live
        // connection, UTIL_SEQ_Idle picks POWER_SAVE_LEVEL_STOP_LS_CLOCK_ON
        // and the chip enters DEEPSTOP between BLE connection events.
        // That's catastrophic in connected-and-locked: the main loop only
        // ticks at the connection interval (iOS often picks ~1 s on a
        // known-bonded reconnect), TIM2/TIM16 state isn't preserved
        // across DEEPSTOP so the LED pulse flails, and motion polling
        // gets crushed to the same ~1 Hz cadence — alarms miss, MLC
        // logs never get pushed.
        //
        // State_Connected_Idle_Loop and State_Alarm_Active_Loop both
        // set this as their first line; State_Locked_Loop was the gap.
        // The disconnect branch above intentionally clears the flag to
        // allow LP-armed entry, so on reconnect-while-locked we land
        // here with stayAwakeFlag=0 and need to re-assert it.
        stayAwakeFlag = 1;
    }

    #undef motion_pending
    #undef pending_type
    #undef motion_pending_tick
}

/***************************************************************************
 * State_Alarm_Active_Loop — alarm runs while motion + alarm_duration tail
 *   Refresh-based: every qualifying motion event (MLC IN_MOTION/SHAKEN, FSM
 *   impact/freefall) updates last_motion_time. The alarm exits when
 *   alarm_duration_s seconds have elapsed since the last motion sample.
 *
 *   Motion-log duration on exit = last_motion_time - motion_pending_tick,
 *   i.e., the time the device was actually in motion. The alarm tail
 *   (alarm_duration_s) is not included.
 *
 *   FSM impact / freefall are logged + alerted at the INT (each is a
 *   distinct sharp event worth its own record). MLC IN_MOTION / SHAKEN
 *   does not log here — the bout-settle log fires once on exit with the
 *   full motion duration.
 *
 *   ALARM_LOUD self-feedback: the SUPER_LOUD pattern's vibration couples
 *   back into the LIS2DUX12 and the MLC reads it as continuous IN_MOTION,
 *   which used to keep refreshing last_motion_time and lock the alarm on
 *   forever. MLC-driven refreshes now go through motion_verifier_confirms:
 *   the buzzer's vibration oscillates symmetrically around a fixed
 *   equilibrium so the X/Y/Z min/max range over the verifier window stays
 *   below the per-tier threshold, and the alarm tail expires normally.
 *   FSM impact/freefall still refresh unconditionally — those are real
 *   shock events the buzzer can't fake at the FSM's calibrated levels.
 ***************************************************************************/
void State_Alarm_Active_Loop(void)
{
    stayAwakeFlag = 1;

    if (PowerMgmt_IsLowPower()) {
        PowerMgmt_RestoreAll();
    }

    static uint8_t  alarm_started    = 0;
    static uint32_t last_motion_time = 0;

    if (!GET_ARMED_BIT(deviceState)) {
        BUZZER_Stop();
        LED_Off();
        alarm_started = 0;
        StateMachine_ChangeState(STATE_CONNECTED_IDLE);
        return;
    }

    uint8_t  alarm_duration_s  = AlarmDuration_Get();
    uint32_t alarm_duration_ms = (uint32_t)alarm_duration_s * 1000u;

    if (!alarm_started) {
        uint8_t alarmType  = GET_ALARM_TYPE(deviceState);
        uint8_t showLights = GET_LIGHTS_BIT(deviceState);
        uint8_t led_b      = LedBrightness_Get();
        switch (alarmType) {
            case ALARM_NONE:
                break;
            case ALARM_CALM:
                if (showLights) LED_Alarm(300, 255, 0, 0, led_b);
                BUZZER_StartCalmAlarm();
                break;
            case ALARM_NORMAL:
                if (showLights) LED_Alarm(300, 255, 0, 0, led_b);
                BUZZER_StartNormalAlarm();
                break;
            case ALARM_LOUD:
                if (showLights) LED_Alarm(125, 255, 225, 0, led_b);
                BUZZER_StartSuperLoudAlarm();
                break;
            default:
                break;
        }
        alarm_started    = 1;
        last_motion_time = HAL_GetTick();
        /* Fresh verifier ring on alarm entry. The motion that triggered the
         * alarm has already happened; we want the verifier to characterise
         * what's happening DURING the alarm, not before. Without the reset,
         * the trigger sample(s) would carry a large-range hit into the
         * alarm loop and the first verifier check inside the alarm would
         * spuriously confirm even on a still device. */
        motion_verifier_reset();
    }

    /* 100 ms verifier sampler — independent of the MLC/FSM poll cadence,
     * so the verifier ring stays populated regardless of when an INT
     * lands. The alarm has to run for ~RING_SIZE/2 * 100 ms = 400 ms
     * before the verifier can confirm; until then MLC-driven refreshes
     * are suppressed and only FSM impact/freefall can hold the alarm on.
     * That's intentional — the user's bug is "buzzer alone keeps alarm
     * forever"; suppressing MLC-driven refresh during the first 400 ms
     * doesn't matter because real motion is still being held in
     * last_motion_time from the trigger-entry assignment above. */
    static uint32_t last_verifier_sample = 0;
    if (HAL_GetTick() - last_verifier_sample >= 100) {
        last_verifier_sample = HAL_GetTick();
        (void)motion_verifier_sample();
    }

    /* INT-driven motion: MLC IN_MOTION/SHAKEN only refreshes
     * last_motion_time when the XYZ-range verifier also confirms.
     * FSM impact/freefall refresh + log unconditionally. */
    if (LIS2DUX12_IsMotionDetected()) {
        uint8_t mlc_out;
        lis2dux12_app_get_mlc_output(&mlc_out);
        lis2dux12_app_update_cached_state(mlc_out);

        uint8_t impact, freefall;
        lis2dux12_app_check_fsm_events(&impact, &freefall);

        uint8_t mlc_moving = (mlc_out == MLC_STATE_IN_MOTION ||
                              mlc_out == MLC_STATE_SHAKEN);
        if ((mlc_moving && motion_verifier_confirms()) || impact || freefall) {
            last_motion_time = HAL_GetTick();
        }

        if (impact || freefall) {
            MotionType_t mt = impact ? MOTION_TYPE_IMPACT : MOTION_TYPE_FREEFALL;
            if (GET_LOGGING_BIT(deviceState)) {
                MotionLogger_LogEvent(mt, 1);
                LOCKSERVICE_SendMotionAlert(mt, 1);
            }
        }
    }

    /* 500 ms polled fallback for MLC class changes and FSM events that
     * didn't latch an INT (chip glitch, brief EXTI masking). MLC-driven
     * refresh here is also gated by motion_verifier_confirms — same
     * reason as the INT branch above. */
    static uint32_t last_mlc_poll = 0;
    if (HAL_GetTick() - last_mlc_poll > 500) {
        last_mlc_poll = HAL_GetTick();

        uint8_t mlc_out;
        if (lis2dux12_app_get_mlc_output(&mlc_out) == 0) {
            lis2dux12_app_update_cached_state(mlc_out);
            if ((mlc_out == MLC_STATE_IN_MOTION || mlc_out == MLC_STATE_SHAKEN)
                && motion_verifier_confirms()) {
                last_motion_time = HAL_GetTick();
            }
        }

        uint8_t impact, freefall;
        lis2dux12_app_check_fsm_events(&impact, &freefall);
        if (impact || freefall) {
            last_motion_time = HAL_GetTick();
            if (GET_LOGGING_BIT(deviceState)) {
                MotionType_t mt = impact ? MOTION_TYPE_IMPACT
                                         : MOTION_TYPE_FREEFALL;
                MotionLogger_LogEvent(mt, 1);
                LOCKSERVICE_SendMotionAlert(mt, 1);
            }
        }
    }

    /* Exit when the alarm tail (alarm_duration_s) has elapsed since the
     * last motion observation. */
    if ((HAL_GetTick() - last_motion_time) >= alarm_duration_ms) {
        /* Log motion duration = last_motion_time - motion_pending_tick.
         * That's the actual time the device was in motion; the alarm tail
         * is excluded. Equivalent to (alarm_total_run_time -
         * alarm_duration_s), which is what the user asked for.
         *
         * Cleared via MotionPending_Reset to prevent State_Locked_Loop's
         * settle-detect path from firing a duplicate when we transition. */
        if (s_motion_pending && GET_LOGGING_BIT(deviceState)) {
            uint32_t motion_ms = last_motion_time - s_motion_pending_tick;
            uint8_t  dur = bout_ticks_250ms_from_ms(motion_ms);
            MotionLogger_LogEvent(s_pending_type, dur);
            LOCKSERVICE_SendMotionAlert(s_pending_type, dur);
        }
        MotionPending_Reset();

        BUZZER_Stop();
        LED_Off();
        alarm_started = 0;
        StateMachine_ChangeState(STATE_LOCKED);
        return;
    }
}

/***************************************************************************
 * StateMachine_ChangeState — set ARMED bit, push BLE notify, record entry
 *   Status byte 6 carries CACHED_STATE_STABILIZING (0xFE) while the device
 *   is settling so the iOS app can show "Stabilizing…". Centrally enforces
 *   the alarmDisabled gate: any incoming STATE_ALARM_ACTIVE is dropped when
 *   the persisted flag is set, so every motion-trigger site is covered
 *   without per-site changes.
 ***************************************************************************/
void StateMachine_ChangeState(SystemState_t newState)
{
    if (newState == STATE_ALARM_ACTIVE && AlarmDisabled_Get()) {
        // Suppress at the trigger. Motion logging + BLE motion alerts have
        // already run in the caller; only the alarm path itself is gated.
        return;
    }

    if (newState != currentState) {
        previousState = currentState;
        currentState = newState;
        stateEntryTime = HAL_GetTick();

        uint8_t newState_is_armed = (newState == STATE_STABILIZING ||
                                     newState == STATE_LOCKED ||
                                     newState == STATE_ALARM_ACTIVE);
        uint8_t prevState_was_armed = (previousState == STATE_STABILIZING ||
                                       previousState == STATE_LOCKED ||
                                       previousState == STATE_ALARM_ACTIVE);
        if (newState_is_armed) {
            SET_ARMED_BIT(deviceState, 1);
        } else {
            SET_ARMED_BIT(deviceState, 0);
        }

        /* armed → disarmed transition (user unlocked mid-alarm, or any other
         * path that leaves the armed states). Drop the in-progress motion
         * bout so it doesn't re-emerge on the next re-arm.
         *
         * Without this, s_motion_pending survives across the disarm and the
         * eventual MLC-settle in LOCKED (or the 60 s force-flush) fires
         * LOCKSERVICE_SendMotionAlert with duration = (now -
         * motion_pending_tick_from_before_disarm). iOS stamps live alerts
         * with its current wall clock and backdates to (receive_time -
         * duration), so the bogus alert shows up in the app as a motion
         * event whose start time is wherever the alarm originally fired.
         *
         * The EEPROM ring itself doesn't need a sweep — MotionLogger_LogEvent
         * early-returns when connectionStatus is true (the user unlock path
         * runs over an active BLE connection), so nothing was written to
         * the ring during the alarm in the first place. */
        if (prevState_was_armed && !newState_is_armed) {
            MotionPending_Reset();
        }

        // EEPROM motion writes block the main loop ~15-25 ms (HAL_Delay +
        // i2c_wait on the M24C08 page commit), which freezes BUZZER_Update
        // and stretches whatever tone TIM16 happens to be playing. Defer
        // while ALARM_ACTIVE so the alarm pattern stays clean; flush after
        // BUZZER_Stop has already silenced the buzzer.
        if (newState == STATE_ALARM_ACTIVE) {
            MotionLogger_SetDeferEEPROM(1);
        } else if (previousState == STATE_ALARM_ACTIVE) {
            MotionLogger_SetDeferEEPROM(0);
            MotionLogger_FlushPending();
        }

        // Checkpoint the iOS-sync time anchor to EEPROM on LOCKED entry so
        // events logged on this boot still resolve to correct calendar times
        // if the device resets during the upcoming alarm window. No-op if
        // iOS hasn't synced the time yet.
        if (newState == STATE_LOCKED) {
            MotionLogger_PersistAnchor();
            /* Fresh debounce ring + XYZ verifier on every entry to LOCKED —
             * covers user arming, post-stabilize entry, and the alarm-loop
             * returning here after the duration timer expires. Without the
             * reset a just-finished alarm bout's tail of hits / XYZ history
             * would re-trigger the alarm before motion fully settled. */
            motion_ring_reset();
            motion_verifier_reset();
        }

        lis2dux12_app_set_stabilizing(newState == STATE_STABILIZING ? 1 : 0);

        // When ARMED clears, reset the cached MLC byte so the status notify
        // we're about to push doesn't carry a stale "Moving"/"Shaken" value
        // latched during LOCKED or ALARM_ACTIVE. The cache otherwise stays
        // pinned until the next class-change INT — which in MEDIUM/LOW
        // sensitivity may never come because LP wakeup wipes the MLC. Motion
        // classification is not meaningful while disarmed, so forcing to
        // STATIONARY is honest.
        if (newState != STATE_STABILIZING && newState != STATE_LOCKED && newState != STATE_ALARM_ACTIVE) {
            lis2dux12_app_update_cached_state(MLC_STATE_STATIONARY_UPRIGHT);
        }

        LOCKSERVICE_SendStatusUpdate();
    }
}

void ChargingCheck(void)
{
    static uint32_t last_check = 0;
    if ((HAL_GetTick() - last_check) < 1000) return;
    last_check = HAL_GetTick();

    if (IS_CABLE_PLUGGED()) {
        if (IS_CHARGING_NOW() && !BATTERY_IsFullCached()) {
            SET_BATTERY_CHARGING(deviceBattery);
        } else {
            CLEAR_BATTERY_CHARGING(deviceBattery);
        }
    }
}

/***************************************************************************
 * Find My Device — non-blocking ping with a green LED synced to each tone
 ***************************************************************************/

void FindMyDevice_Start(void)
{
    BUZZER_StartFindMe();
    findMyActive = 1;
}

void FindMyDevice_Update(void)
{
    if (!findMyActive) return;

    if (!BUZZER_IsPlaying()) {
        LED_Off();
        findMyActive = 0;
        return;
    }

    if (BUZZER_IsToneActive()) {
        LED_Solid(0, 255, 0, LedBrightness_Get());
    } else {
        LED_Off();
    }
}

void StateMachine_Run(void)
{
    CablePlug_UpdateState();

    ChargingCheck();
    BUZZER_Update();
    FindMyDevice_Update();

    switch (currentState) {
        case STATE_DISCONNECTED_IDLE:
            State_Disconnected_Idle_Loop();
            break;
        case STATE_CONNECTED_IDLE:
            State_Connected_Idle_Loop();
            break;
        case STATE_STABILIZING:
            State_Stabilizing_Loop();
            break;
        case STATE_LOCKED:
            State_Locked_Loop();
            break;
        case STATE_ALARM_ACTIVE:
            State_Alarm_Active_Loop();
            break;
        default:
            StateMachine_ChangeState(STATE_DISCONNECTED_IDLE);
            break;
    }
}

/***************************************************************************
 * Persisted device-state record — EEPROM-backed mirror of the user-facing
 * settings byte (alarm type, sensitivity, lights, logging, silence, AND
 * ARMED) plus deviceInfo bit 0 (HIGH_PERF). Layout:
 *
 *   [0] magic
 *   [1] deviceState (full byte, including ARMED — see policy note below)
 *   [2] deviceInfo HIGH_PERF (bit 0 only; bit 1 alarmDisabled lives in its
 *       own record, owned by sound.c)
 *
 * Address + length + magic live in eeprom_map.h. Magic was bumped (0xC6 →
 * 0xCA) in the commit that introduced the central map: the record moved
 * from 0x1E to 0x20 to clear collisions, AND the ARMED-bit policy reversed.
 *
 * ARMED persistence policy: the ARMED bit is now persisted across power
 * cycles and resets. A locked device that browns out and resets comes back
 * up in LOCKED — without this, an attacker who can induce a brownout (e.g.
 * cable wiggling, a near-empty battery) would defeat the alarm. The prior
 * design (force-clear ARMED on boot) was motivated by "stolen device, owner
 * can't disarm" — weaker in practice because the loyalty token already
 * prevents non-owners from sending the un-arm settings write.
 *
 * Brownout-loop caveat: if the *cause* of frequent resets is a sag during
 * the alarm itself (high LED brightness + +8 dBm BLE + buzzer), persisting
 * ARMED means each wake-up immediately re-arms and the alarm fires again
 * the moment the accel triggers, potentially dragging the rail back down.
 * Mitigate with battery health and BLE_TX_POWER_NORMAL, not by re-disabling
 * persistence.
 ***************************************************************************/

static M24CXX_HandleTypeDef s_sm_eeprom;

void DeviceSettings_Init(void)
{
    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sm_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) != M24CXX_Ok) {
        PowerMgmt_EEPROM_PowerOff();
        return;
    }

    uint8_t buf[EEPROM_DEVICE_SETTINGS_LEN] = {0};
    if (m24cxx_read(&s_sm_eeprom, EEPROM_DEVICE_SETTINGS_ADDR, buf,
                    EEPROM_DEVICE_SETTINGS_LEN) == M24CXX_Ok) {
        if (buf[0] == EEPROM_DEVICE_SETTINGS_MAGIC) {
            // Apply persisted bits, ARMED bit included. If ARMED comes back
            // true, the post-init transition in main() / StateMachine_Init's
            // caller is responsible for entering STATE_LOCKED. We don't do
            // it from here because the state machine prep (timers, anchors)
            // hasn't finished yet.
            deviceState = buf[1];
            // Preserve any deviceInfo bits already loaded by other inits
            // (alarmDisabled doesn't touch the RAM byte at boot, so this
            // is mostly defensive for future bits).
            deviceInfo = (deviceInfo & ~0x01) | (buf[2] & 0x01);
        } else {
            // Blank EEPROM / wrong magic — seed from the StateMachine_Init
            // defaults that already populated deviceState/deviceInfo.
            uint8_t fresh[EEPROM_DEVICE_SETTINGS_LEN];
            fresh[0] = EEPROM_DEVICE_SETTINGS_MAGIC;
            fresh[1] = deviceState;          // ARMED persisted (defaults off)
            fresh[2] = deviceInfo  &  0x01;
            (void)m24cxx_write(&s_sm_eeprom, EEPROM_DEVICE_SETTINGS_ADDR,
                               fresh, EEPROM_DEVICE_SETTINGS_LEN);
        }
    }

    PowerMgmt_EEPROM_PowerOff();
}

/***************************************************************************
 * DeviceSettings_Persist — write the current deviceState/deviceInfo to
 *   EEPROM. Called after the iOS settings dispatcher applies a write.
 *   Skips the I2C transaction when the persisted-relevant bits are
 *   unchanged, to avoid wear on settings notifications that don't actually
 *   modify any user-facing flag.
 ***************************************************************************/
void DeviceSettings_Persist(void)
{
    static uint8_t cached_state = 0xFF;   // forces first write
    static uint8_t cached_info  = 0xFF;

    // ARMED is now persisted — see the policy block above the record def.
    uint8_t to_save_state = deviceState;
    uint8_t to_save_info  = deviceInfo  &  0x01;

    if (to_save_state == cached_state && to_save_info == cached_info) {
        return;
    }

    cached_state = to_save_state;
    cached_info  = to_save_info;

    PowerMgmt_EEPROM_PowerOn();
    HAL_Delay(2);

    if (m24cxx_init(&s_sm_eeprom, &hi2c1, EEPROM_I2C_ADDRESS) == M24CXX_Ok) {
        uint8_t buf[EEPROM_DEVICE_SETTINGS_LEN];
        buf[0] = EEPROM_DEVICE_SETTINGS_MAGIC;
        buf[1] = to_save_state;
        buf[2] = to_save_info;
        (void)m24cxx_write(&s_sm_eeprom, EEPROM_DEVICE_SETTINGS_ADDR, buf,
                           EEPROM_DEVICE_SETTINGS_LEN);
    }

    PowerMgmt_EEPROM_PowerOff();
}

/***************************************************************************
 * StateMachine_RestoreArmedFromEEPROM — boot-time helper
 *   If DeviceSettings_Init restored ARMED=1, jump currentState directly to
 *   STATE_LOCKED. We skip STABILIZING because that state's whole purpose is
 *   "user just hit arm — give the device a few seconds to settle before
 *   trusting motion classifications." A power-cycle / reset has no such
 *   user-input event; we just want to come back where we were.
 *
 *   StateMachine_ChangeState handles all the bookkeeping: ARMED-bit re-set,
 *   forced status notify, anchor checkpoint, MLC stabilizing flag clear.
 *
 *   No-op when ARMED is clear, so this is always safe to call once at boot.
 ***************************************************************************/
void StateMachine_RestoreArmedFromEEPROM(void)
{
    if (!GET_ARMED_BIT(deviceState)) {
        return;
    }

    /* Brief motion-grace window so the accel/UCF can settle after I2C init
     * without immediately tripping the alarm on a phantom edge. Same idea
     * as the post-connect grace window. */
    StateMachine_StartMotionGrace(2000);

    StateMachine_ChangeState(STATE_LOCKED);
}
