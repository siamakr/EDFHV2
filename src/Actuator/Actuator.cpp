#include "Actuator.h"
#include <math.h>

Actuator::Actuator(uint8_t pin,
                   float p0, float p1, float p2,
                   float minVal, float maxVal,
                   float neutralVal,
                   float cutoffVal,
                   uint16_t offUs)
    : _pin(pin)
    , _p0(p0), _p1(p1), _p2(p2)
    , _min(minVal), _max(maxVal)
    , _neutral(neutralVal)
    , _currentValue(neutralVal)
    , _cutoff(cutoffVal)
    , _offUs(offUs)
    , _cutOff(false)
{}

void Actuator::initialize()
{
    _servo.attach(_pin);

    // attach() latches 1500 us, which on the EDF is roughly 21 N. Never leave
    // the pin there between attach and the first command.
    neutral();
}

void Actuator::drive(float value)
{
    // Below the cutoff the regression does not apply -- emit the raw off
    // pulse. Defaults to never firing.
    if (value <= _cutoff) {
        _currentValue = _neutral;
        _cutOff       = true;
        _lastUs       = _offUs;
        _servo.writeMicroseconds(_offUs);
        return;
    }

    _cutOff       = false;
    _currentValue = std::clamp(value, _min, _max);
    writeRegressed(_currentValue);
}

void Actuator::writeRegressed(float value)
{
    const long us = lroundf(toPwm(value));
    _lastUs = (uint16_t)std::clamp<long>(us, _minUs, _maxUs);
    _servo.writeMicroseconds(_lastUs);
}

void Actuator::writeRaw(uint16_t us)
{
    _cutOff = false;
    _lastUs = us;
    _servo.writeMicroseconds(us);
}

void Actuator::setPwmLimits(uint16_t minUs, uint16_t maxUs)
{
    _minUs = minUs;
    _maxUs = maxUs;
}

// PWM = P0 + P1*x + P2*x^2
float Actuator::toPwm(float value) const
{
    return _p0 + _p1 * value + _p2 * value * value;
}
