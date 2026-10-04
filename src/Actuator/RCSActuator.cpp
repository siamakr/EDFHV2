#include "RCSActuator.h"

RCSActuator::RCSActuator(uint8_t pin,
                         float p0, float p1, float p2,
                         float minForce, float maxForce,
                         float momentArm,
                         float cutoffNm, uint16_t offUs)
    // neutral = 0 N*m: no yaw torque commanded
    : Actuator(pin, p0, p1, p2, minForce, maxForce, 0.0f, cutoffNm, offUs)
    , _momentArm(momentArm)
{}

void RCSActuator::drive(float torque_Nm)
{
    if (torque_Nm <= _cutoff) {
        _currentValue = 0.0f;
        _cutOff       = true;
        _lastUs       = _offUs;
        _servo.writeMicroseconds(_offUs);
        return;
    }
    _cutOff = false;

    // T = 2*F*r  ->  F = T / (2*r)
    float forcePerMotor_N = torque_Nm / (2.0f * _momentArm);

    _currentValue = std::clamp(forcePerMotor_N, _min, _max);

    // Single wire, both motors follow it.
    writeRegressed(_currentValue);
}
