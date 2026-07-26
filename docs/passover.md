# EDFHV2 Flight Controller — Rebuild Notes & Handoff

This document captures the architecture, conventions, decisions, and open work
for the rebuilt flight-controller software stack. It's written for whoever (or
whatever agent) picks up the integration work next. Read it before touching the
code.

---

## 1. What this is

A ground-up rewrite of the flight software for a VTOL "hopper" — a single
gimballed EDF (electric ducted fan) vehicle with an RCS motor pair for yaw.
Target hardware is a **Teensy 4.1**, built with **PlatformIO**. The rewrite
replaces the old `siamakr/EDFH_V0.1` code (branch `flight_gs`), which worked but
had accumulated dead code, duplicated constants, and several latent bugs (see
§7).

The controller is a **cascaded LQR**:

```
PositionController      AttitudeController      ControlAllocator     ActuatorManager
  (outer loop)            (inner loop)            (mapping)            (hardware)
  x,y,vx,vy      ──►   roll/pitch setpoint  ──►  torques/force  ──►  gimbal deg,
  → desired            + attitude + alt          → actuator          thrust N,
    roll/pitch         → [u0,u1,tau_z,Fz]         commands            yaw torque
```

Each stage is a pure function of its input and is unit-testable off-hardware.
The inner loop can run standalone (feed attitude setpoints directly) for bench
testing before the position loop is trusted.

---

## 2. Design principles (follow these when extending)

1. **Each class does one thing and doesn't know about the layers around it.**
   The controller computes; it does not drive hardware. The allocator maps; it
   does not know about servos. The manager drives; it does not know about
   control law. Keep these boundaries.

2. **Dependency injection.** `ActuatorManager` and the controllers receive
   their collaborators by reference; they don't construct them. Objects are
   built in `main.cpp` from `consts.h` values and passed in. This keeps
   pin numbers and calibration out of the logic classes and lets a bench
   harness inject mocks.

3. **`consts.h` is the single source of truth for every constant.** A physical
   quantity is declared exactly once. No `#define`. No magic numbers in code.
   (The old code had the max TVC angle defined twice with two different values —
   10° and 8°. That class of bug is why this rule exists.)

4. **Units are in the names.** Every variable carrying a physical quantity has a
   unit suffix: `_rad`, `_deg`, `_N`, `_Nm`, `_m`, `_mps`, `_rps`, `_us`, `_kg`.
   Exceptions, by deliberate choice:
   - **Dimensionless** values stay bare (`ratioX`, IIR `alpha`).
   - **Generic base-class** I/O stays bare: `Actuator::drive(float value)` and
     `getCurrentValue()` — the unit varies by subclass (N, deg, N·m), so a
     suffix would be a lie.
   - **The raw LQR output vector `u`** stays bare. `u(0)/u(1)` are radians today
     but become N·m after the torque re-derivation (see §5). A `_rad` suffix
     would become wrong the day the gains change.

5. **Naming convention:**
   - Types/classes: `PascalCase`
   - Functions/methods: `camelCase`
   - Members: `_camelCase`
   - Locals/params: `camelCase`
   - Compile-time constants: `ALL_CAPS`, inside lowercase namespaces
     (`cfg::vehicle::MASS_KG`)
   - Gain matrices stay short and mathematical (`K`, `Kpos`) — spelling them out
     fights the controls math.
   - **No `snake_case`** anywhere. The old code was full of it; it's gone.

---

## 3. Class inventory (what's built, all host-tested)

| File | Purpose |
|------|---------|
| `consts.h` | All constants, namespaced under `cfg::`. Single source of truth. |
| `Actuator.h/.cpp` | Base class. Owns a `Servo`, a quadratic regression `PWM = P0 + P1·x + P2·x²`, min/max clamp, and an explicit neutral. `drive(float value)` takes natural units. |
| `RCSActuator.h/.cpp` | Subclass. `drive(float torque_Nm)` converts vehicle yaw torque → per-motor force via `F = T / (2·momentArm)`, then through a single-motor force→PWM regression. Both motors share one PWM pin. |
| `ActuatorManager.h/.cpp` | Owns nothing; holds four `Actuator&` (edf, gimbalX, gimbalY, rcs) by injection. Exposes vehicle-level commands, an arm/disarm gate on thrust, `safeAll()`, and `servoDance()` (a self-test that traces a cone with the thrust vector). |
| `ControlAllocator.h/.cpp` | Maps controller output → actuator commands. Two modes (see §5). No dependency on BLA or Servo — takes a `float[4]`, returns `AllocatorOutput`. Host-testable in isolation. |
| `AttitudeController.h/.cpp` | Inner-loop LQR. `setReference(roll,pitch,yaw,z)` (or individual `setXRef`), `update(8 scalars)` → `Matrix<4,1>`. No integral action, no allocation, no actuator knowledge. |

**Not yet built:** `PositionController`, `main.cpp` integration. See §6.

---

## 4. Data flow contract (exact units at each boundary)

```
AttitudeController::update(roll_rad, pitch_rad, yaw_rad,
                           gyroX_rps, gyroY_rps, gyroZ_rps,
                           z_m, velZ_mps)
    → Matrix<4,1> u = [ u0, u1, yawTorque_Nm, thrust_N ]
         u0,u1 are the roll/pitch control outputs (radians today — see §5)

extract to float[4], then:

ControlAllocator::allocate(u[4]) → AllocatorOutput {
    gimbalX_deg, gimbalY_deg, yawTorque_Nm, thrust_N
}

then route to ActuatorManager:
    setGimbal(gimbalX_deg, gimbalY_deg)
    setYawTorque(yawTorque_Nm)     → RCSActuator converts to per-motor force
    setThrust(thrust_N)            → gated by arm state
```

**Sign convention:** `AttitudeController` negates yaw and gyroZ when loading the
state vector (`{roll, pitch, -yaw, gyroX, gyroY, -gyroZ, z, velZ}`). This matches
the frame the MATLAB gain `K` was derived against. **Do not remove without
re-deriving K.**

**Gravity feedforward:** `u(3) += WEIGHT_N` so the LQR only regulates deviation
from hover, not the whole weight.

---

## 5. The dual-mode allocator (important — read before changing the controller)

The old controller's `u(0)/u(1)` were gimbal **angles** (radians), written
directly to servos. That works near hover but loses accuracy as thrust departs
from hover, because the true relationship is:

```
tau = Fz · sin(delta) · L        →        delta = asin( tau / (Fz · L) )
```

i.e. control authority per degree depends on live thrust `Fz`. The old code
actually computed this (`asin(Tx/Tm)`) but **threw the result away** and flew the
raw angle — the torque path was dead code the author was mid-way to adopting.

`ControlAllocator` supports **both**, switchable at runtime via `setMode()`:

- **`AllocMode::PASSTHROUGH`** — `u(0),u(1)` are gimbal angles (rad). Allocation
  is just rad→deg + routing. **This is what flies today.** Use it to confirm the
  new airframe works on hardware with the existing, already-tuned gains.

- **`AllocMode::TORQUE`** — `u(0),u(1)` are body torques (N·m). Allocation
  inverts the geometry above using live thrust. Correct off-hover. **Not usable
  until K is re-derived in MATLAB against a torque-input plant.**

**The migration path:** fly PASSTHROUGH now to validate hardware → re-derive K in
the MATLAB sim for torque output → flip to `setMode(AllocMode::TORQUE)`. One line
changes; nothing downstream moves, because both modes return the same
`AllocatorOutput` struct.

Yaw is identical in both modes: `tau_z` passes straight through as a torque;
`RCSActuator` owns the torque→force→PWM conversion.

---

## 6. Next steps (in order)

1. **Drop the finished files into the repo and get it compiling in PlatformIO.**
   PlatformIO wants a flat `src/` for `.cpp`, or library subfolders under
   `lib/`. Decide whether the actuator classes live flat in `src/` or as a
   `lib/actuators/` library. (Currently they assume flat includes like
   `#include "consts.h"`.)

2. **Fill in the TODO constants in `consts.h`** — these need real bench data and
   cannot be guessed (see §8).

3. **Write `main.cpp` integration.** This is the real work and where a
   repo-aware agent helps most. The old `flight_gs/main.cpp` (581 lines) has the
   state machine, sensor sampling (BNO080 IMU, LIDAR-Lite v3HP, PMW3901 flow),
   and the 200 Hz flight loop. The new loop should be:
   ```
   sample sensors → run estimator
   → attitude.update(...) → extract u[4]
   → allocator.allocate(u) → AllocatorOutput
   → manager.setGimbal / setYawTorque / setThrust
   ```
   Keep the estimator ordering correct: in the old code the estimator ran
   *after* the LQR call, so `vz`/`velZ` was one cycle stale. Fix that — estimate
   before control.

4. **Build `PositionController`** (outer loop) once attitude flies. State
   `[x, y, vx, vy]` (drop the old integral states — no integral action in the
   rewrite), gain `Kpos` (2×4), outputs desired `roll/pitch` which feed
   `attitude.setRollRef()` / `setPitchRef()`. Same class shape as
   `AttitudeController`: `setReference`, `update(scalars)` → `Matrix<2,1>`.

5. **Measure real loop `dt`** instead of trusting a hardcoded `DT`. The old code
   assumed the nominal interval was always exact and never checked. `consts.h`
   has `DT_MAX_S` for a staleness guard — use it to reject bad control steps.

6. **Wire up the abort check.** The old `emergency_check()` (a tilt abort) was
   written and never called. `consts.h` has `MAX_TILT_DEG`. Hook it into the
   flight loop.

---

## 7. Bugs found in the old `flight_gs` code (do NOT reintroduce)

- **`lqr_pos` argument-order mismatch.** The call site passed velocities into
  the position params and positions into the velocity params. The outer loop
  was feeding swapped state. Verify arg order when rebuilding
  `PositionController`.
- **`U_pos` computed then discarded.** `lqr_pos` calculated its output, then the
  assignment was commented out ("done inside Main.cpp") — but main read a
  never-written `U_pos`. Outer-loop output was effectively zero.
- **Gravity constant typo.** `G` was `9.87` on `main` (should be `9.81`) — a
  0.6% error directly in the hover-thrust feedforward. `flight_gs` had `9.807`.
  `consts.h` uses `9.81`.
- **Duplicated constants** with divergent values: max TVC angle (10° vs 8°),
  `d2r`/`r2d` defined in three headers, `MASS` disagreeing between `Constants.h`
  and a commented block (2.79 vs 3.373).
- **Integral action never actually ran** on `main` (all integral lines
  commented; `lqr_int` never called) and was still half-wired on `flight_gs`.
  The rewrite drops it entirely — this was a deliberate decision, not an
  oversight.
- **Yaw grams hop.** Old code converted torque→force→**grams** because the
  reaction-wheel `writeRW()` spoke grams. The new RCS is calibrated directly in
  force→PWM, so grams is gone. Don't add it back.
- **Stale estimator ordering** (see §6 step 3).

---

## 8. Open TODO constants (need bench data — cannot be guessed)

From `consts.h`:

- `cfg::vehicle::COM_TO_RCS_M` — currently `0.0`. Measure COM→RCS distance.
- `cfg::gimbal::x/y::MIN_DEG / MAX_DEG` — currently ±5/8° placeholders. Confirm
  actual travel on the new linkage. **Travel is asymmetric about neutral**, so
  `NEUTRAL_DEG` is declared explicitly and is NOT `(min+max)/2`.
- `cfg::rcs::P0/P1/P2` — currently `0`. Calibrate the single-motor force→PWM
  curve.
- `cfg::rcs::MAX_N` — currently `0`. Measure per-motor force limit.
- `cfg::ctrl::MAX_YAW_TORQUE_NM` — currently `0.5` placeholder. The old
  reaction-wheel value was **1.61 N·m**; the prop-based RCS may differ. Confirm
  the RCS pair can actually produce whatever you set here.
- **Gain matrix zeros.** `AttitudeController` hardcodes the off-diagonal
  couplings as `0.0f`. If the MATLAB solve produced small nonzero cross-terms
  you care about, they must come from `consts.h`, not be assumed zero. Verify
  against the actual K.
- **`K_int` column labels.** On `flight_gs`, `K_int`'s header comment labeled
  columns 6/7 as `vz`/`z` while the state loaded `z`/`vz`. Comment and code
  disagreed. Verify state ordering against the MATLAB model before reusing gains.

---

## 9. Related project context

- **Outer guidance loop:** a Raspberry Pi runs a slower outer loop (MPC/LQR,
  ~2 Hz) talking to the Teensy inner loop (target 500 Hz, currently 200 Hz)
  over UART. The `PositionController` here may eventually be superseded or fed by
  that, depending on the split chosen.
- **Vehicle lineage:** the previous vehicle used a gimballed EDF with a reaction
  wheel for yaw and dual-IMU differential thrust-vector calibration. The new
  vehicle replaces the reaction wheel with the RCS motor pair (two CCW props
  countering the EDF's angular momentum).
- **Data pipeline:** experimental data lands in S3, queried and visualized via
  Grafana; there's a Python/`scipy` FIR-filter step in that pipeline.

---

## 10. How the code was validated

Every finished class was host-compiled with `g++ -std=c++17` and run against
small test harnesses (BLA and Servo/Arduino symbols stubbed for the host).
Verified numerically:

- `AttitudeController`: zero output at setpoint; corrective sign on roll error;
  thrust rises below target altitude; gimbal output saturates at `MAX_TVC_RAD`;
  IIR filter behavior on first call.
- `ControlAllocator`: PASSTHROUGH = exact rad→deg; TORQUE = correct
  `asin(tau/(Fz·L))`; zero-thrust guard (no NaN); saturation clamp (no NaN).
- `RCSActuator`: 1 N·m → 2.5 N per motor → 1 N·m round-trip.

These are host sanity checks, not hardware validation. Bench-test on the actual
Teensy before trusting any of it in flight.