#include "ActuatorManager.h"
#include <Arduino.h>   // delay(), PI
#include <math.h>      // sinf(), cosf()

ActuatorManager::ActuatorManager(Actuator& edf,
                                 Actuator& gimbalPitch,
                                 Actuator& gimbalRoll,
                                 Actuator& rcs)
    : _edf(edf)
    , _gimbalPitch(gimbalPitch)
    , _gimbalRoll(gimbalRoll)
    , _rcs(rcs)
    , _armed(false)
{}

void ActuatorManager::initialize()
{
    _edf.initialize();
    _gimbalPitch.initialize();
    _gimbalRoll.initialize();
    _rcs.initialize();

    safeAll();
}

// ── Arming ──────────────────────────────────────────────────────────────────

void ActuatorManager::arm()
{
    _armed = true;
}

void ActuatorManager::disarm()
{
    _armed = false;
    _edf.neutral();   // cut thrust immediately
}

// ── Vehicle-level commands ──────────────────────────────────────────────────

void ActuatorManager::setThrust(float newtons)
{
    if (!_armed) return;
    _edf.drive(newtons);
}

void ActuatorManager::primeEdf(uint16_t us)
{
    if (!_armed) return;
    _edf.writeRaw(us);
}

void ActuatorManager::setGimbal(float pitchDeg, float rollDeg)
{
    _gimbalPitch.drive(pitchDeg);
    _gimbalRoll.drive(rollDeg);
}

void ActuatorManager::setYawTorque(float torqueNm)
{
    _rcs.drive(torqueNm);
}

void ActuatorManager::safeAll()
{
    _edf.neutral();
    _gimbalPitch.neutral();
    _gimbalRoll.neutral();
    _rcs.neutral();
}

// ── Self-test ───────────────────────────────────────────────────────────────

float ActuatorManager::symmetricTravel(const Actuator& a)
{
    float n = a.getNeutral();
    // An axis calibrated -5..+8 about neutral 0 can only swing 5 deg and
    // stay symmetric. Take the tighter side.
    return std::min(n - a.getMin(), a.getMax() - n);
}

float ActuatorManager::getMaxSafeGimbalAmplitude() const
{
    // A circle needs equal amplitude on both axes, so the tighter axis sets
    // the limit. Back off slightly so we never sit on a hard stop.
    return std::min(symmetricTravel(_gimbalPitch),
                    symmetricTravel(_gimbalRoll)) * 0.9f;
}

void ActuatorManager::servoDance(float    amplitudeDeg,
                                 uint8_t  revolutions,
                                 uint16_t periodMs,
                                 uint8_t  stepsPerRev)
{
    if (stepsPerRev == 0) return;

    float maxAmp = getMaxSafeGimbalAmplitude();
    float amp    = (amplitudeDeg <= 0.0f) ? maxAmp
                                          : std::min(amplitudeDeg, maxAmp);

    float pitchNeutral = _gimbalPitch.getNeutral();
    float rollNeutral  = _gimbalRoll.getNeutral();

    uint16_t stepDelay = periodMs / stepsPerRev;

    for (uint8_t rev = 0; rev < revolutions; ++rev) {
        for (uint8_t i = 0; i < stepsPerRev; ++i) {
            float theta = (2.0f * PI * i) / stepsPerRev;

            // Offset from each axis' declared neutral, not from mid-travel.
            _gimbalPitch.drive(pitchNeutral + amp * sinf(theta));
            _gimbalRoll.drive( rollNeutral  + amp * cosf(theta));

            delay(stepDelay);
        }
    }

    // Park at neutral.
    _gimbalPitch.neutral();
    _gimbalRoll.neutral();
}
