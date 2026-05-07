# Firmware: 15-second stabilize timeout

## What to change

Add a 15-second timeout to `STATE_STABILIZING`. If the device never settles into 3 s of stillness within 15 s of entering the state, give up and fall back to `STATE_CONNECTED_IDLE` (which automatically clears the `ARMED` bit and pushes a BLE status update).

Currently `State_Stabilizing_Loop` in `Core/Src/state_machine.c` has no timeout — if motion never settles, the device sits there forever pulsing blue.

## Where

`Core/Src/state_machine.c::State_Stabilizing_Loop` (around lines 260–308).

The function already tracks `last_still_time` (the most recent moment the MLC reported motion) and uses it for the 3 s settle check:

```c
if (HAL_GetTick() - last_still_time >= 3000) {
    stabilize_started = 0;
    StateMachine_ChangeState(STATE_LOCKED);
}
```

Add a parallel `stabilize_entry_time` that captures the tick the state was first entered, and bail when 15 s have elapsed regardless of motion state.

## Suggested implementation

Add a constant near the top of the file (or in `Core/Inc/state_machine.h` if that fits the project's style better — match the surrounding code):

```c
// Maximum time to wait for stillness before giving up and falling back
// to CONNECTED_IDLE. Pulses blue this long; if the device never settles
// the user gets un-armed instead of a permanent stuck pulse.
#define STABILIZE_TIMEOUT_MS  15000u
```

Then inside `State_Stabilizing_Loop`, alongside the existing `last_still_time` / `stabilize_started` statics, track entry time:

```c
static uint32_t stabilize_entry_time = 0;

if (!stabilize_started) {
    last_still_time      = HAL_GetTick();
    stabilize_entry_time = HAL_GetTick();
    stabilize_started    = 1;
    LIS2DUX12_ClearMotion();
}
```

After the existing 3 s settle check, add the timeout:

```c
if (HAL_GetTick() - stabilize_entry_time >= STABILIZE_TIMEOUT_MS) {
    stabilize_started = 0;
    StateMachine_ChangeState(STATE_CONNECTED_IDLE);
    return;
}
```

`StateMachine_ChangeState(STATE_CONNECTED_IDLE)` already does the right thing on entry: it clears the `ARMED` bit (per the table in `StateMachine_ChangeState`, line 696), clears the stabilizing override (`lis2dux12_app_set_stabilizing(0)`), and calls `LOCKSERVICE_SendStatusUpdate()` so iOS sees the device fall back to unlocked. No additional notify or LED-clear call is needed.

The `LED_Off()` happens implicitly: `State_Stabilizing_Loop` only drives the blue pulse while the function runs, and once we've changed state the dispatcher in `StateMachine_Run` switches to `State_Connected_Idle_Loop` next tick which sets the appropriate LED.

## Edge cases to think through

- **User explicitly un-arms during stabilizing.** Already handled — the existing early return at the top of `State_Stabilizing_Loop` checks `GET_ARMED_BIT(deviceState)` and bails. The new timeout doesn't change that path.
- **Motion-grace window.** Doesn't intersect — the grace window only suppresses transitions to `STATE_ALARM_ACTIVE` from `STATE_LOCKED`. Stabilize runs after.
- **`stabilize_started` reset.** Make sure both new bailouts (timeout, ARMED-cleared) zero `stabilize_started` so a subsequent re-entry resets entry/still timestamps cleanly. The ARMED-cleared path at the top of the loop currently does NOT reset `stabilize_started` — it's reset by the success path on `STATE_LOCKED` transition. Audit that and reset on every exit.
- **Tick wraparound.** All comparisons use `HAL_GetTick() - <stored_tick>` arithmetic, which is wraparound-safe at uint32 width as long as the interval is < ~24 days. 15 s is fine.

## Docs

Update `CLAUDE.md`:

- The state-machine sketch at the top of the architecture section currently shows:
  ```
  STABILIZING       → (3s still)    → LOCKED
  ```
  Add a sibling arrow for the timeout:
  ```
  STABILIZING       → (3s still)    → LOCKED
  STABILIZING       → (15s elapsed) → CONNECTED_IDLE
  ```
- If there's a longer prose paragraph about the state machine, mention that stabilize bails to CONNECTED_IDLE (un-armed) after `STABILIZE_TIMEOUT_MS` so a stuck pulse is impossible.

## Coordinated iOS change

The iOS side has its own prompt at `WatchDog_iOS/STABILIZE_TIMEOUT_PROMPT.md`. It handles the "Hold to Stop" button label change. iOS doesn't need any code change for the timeout itself — when the firmware transitions to `STATE_CONNECTED_IDLE` it pushes a status update with `ARMED=0`, and iOS's existing `syncLockedFromDeviceIfApplicable` already updates `isLocked` reactively.

## Don't touch

- The 3 s settle threshold — keep that as-is.
- The MLC interrupt handler or polling logic — only the timeout check is new.
- `StateMachine_ChangeState` — its existing ARMED-bit and notify logic is exactly what we need.

## Version bump

Per `CLAUDE.md`, this is a `main`-branch commit:
- Recompute `MAIN = git rev-list --count --first-parent main` (then +1 for the commit you're about to make).
- `V2 = 0` (cascading reset on `MAIN` bump).
- Update `Core/Inc/firmware_version.h` and the `Current: V…` + reconciled-sha line in `CLAUDE.md` in the same commit.
- Commit message: `version bump to Vx.y.z`.
