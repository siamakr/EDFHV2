#pragma once
#include "Actuator.h"

// ─────────────────────────────────────────────────────────────────────────────
// ActuatorManager
//
// Single point of contact between the control loop and the hardware. The
// controller talks in vehicle-level quantities (thrust, gimbal angles, yaw
// torque) and never touches PWM, pins, or regressions.
//
// Dependency injection
// ────────────────────
// Actuators are constructed in main.cpp and passed in by reference. The
// manager does not own them. This keeps the manager free of pin numbers and
// calibration data, and lets a bench/HIL harness inject mock Actuator
// subclasses that log instead of driving hardware.
//
// All four channels are held as Actuator& -- drive() is virtual, so the RCS
// channel still dispatches to RCSActuator::drive() and gets its torque->force
// conversion. The units below are whatever each injected object was
// calibrated in.
//
// Arming
// ──────
// setThrust() is a no-op until arm() is called. Gimbal and RCS commands are
// always allowed so attitude can be checked on the bench without the EDF
// spinning. disarm() drops thrust to minimum immediately.
// ─────────────────────────────────────────────────────────────────────────────

class ActuatorManager {
public:
    ActuatorManager(Actuator& edf,
                    Actuator& gimbalPitch,
                    Actuator& gimbalRoll,
                    Actuator& rcs);

    // Attaches every channel and drives it to its safe position.
    void initialize();

    // ── Arming ────────────────────────────────────────────────────────────
    void arm();
    void disarm();
    bool isArmed() const { return _armed; }

    // ── Vehicle-level commands ────────────────────────────────────────────
    void setThrust(float newtons);                  // ignored while disarmed

    // Writes a raw spin-up pulse to the EDF, bypassing the regression.
    // Ignored while disarmed. Non-blocking: the caller holds it for
    // cfg::edf::PRIME_MS and then hands over to setThrust().
    void primeEdf(uint16_t us);
    void setGimbal(float pitchDeg, float rollDeg);
    void setYawTorque(float torqueNm);

    // Every channel to its own neutral. Does not change armed state.
    void safeAll();

    // ── Self-test ─────────────────────────────────────────────────────────
    // Sweeps the thrust vector around a cone: pitch = A*sin(t),
    // roll = A*cos(t). Confirms both gimbal axes move, that linkages are
    // free through full travel, and that the two axes are wired to the
    // correct pins (a swapped pair traces the circle backwards).
    //
    // BLOCKING -- uses delay(). Intended for bench checkout and pre-flight,
    // not for use inside the control loop.
    //
    // Does not touch thrust or RCS. Returns gimbals to neutral when done.
    //
    //   amplitudeDeg : cone half-angle. <= 0 means use the largest angle
    //                  that stays inside both axes' travel limits.
    //   revolutions  : full circles to trace
    //   periodMs     : milliseconds per revolution
    //   stepsPerRev  : sample points per circle (higher = smoother)
    void servoDance(float    amplitudeDeg = -1.0f,
                    uint8_t  revolutions  = 2,
                    uint16_t periodMs     = 2000,
                    uint8_t  stepsPerRev  = 60);

    // Largest cone half-angle that keeps both axes inside their limits.
    // Travel is measured from each axis' neutral, and asymmetric travel is
    // handled -- an axis that goes -5..+8 deg contributes 5, not 6.5.
    float getMaxSafeGimbalAmplitude() const;

    // ── Telemetry ─────────────────────────────────────────────────────────
    float getThrust()    const { return _edf.getCurrentValue(); }
    float getPitch()     const { return _gimbalPitch.getCurrentValue(); }
    float getRoll()      const { return _gimbalRoll.getCurrentValue(); }
    float getRcsForce()  const { return _rcs.getCurrentValue(); }

private:
    // Smaller of the two travel directions from neutral. This is the usable
    // symmetric swing for an axis whose travel is lopsided.
    static float symmetricTravel(const Actuator& a);

    Actuator& _edf;
    Actuator& _gimbalPitch;
    Actuator& _gimbalRoll;
    Actuator& _rcs;

    bool _armed;
};
