#pragma once
#include <Servo.h>
#include <algorithm>
#include <cfloat>
#include <cstdint>

class Actuator {
public:
    // pin        : PWM pin
    // p0, p1, p2 : regression, natural units -> PWM (us)
    // minVal     : lower limit, natural units
    // maxVal     : upper limit, natural units
    // neutralVal : the value to pass to drive() that puts this actuator at
    //              rest. Defined in drive()'s INPUT units, which is not
    //              always the same as min/max -- see RCSActuator.
    //
    //              EDF     -> 0 N       (no thrust)
    //              Gimbal  -> 0 deg     (thrust vector straight down);
    //                         NOT the midpoint of travel, which is wrong
    //                         whenever travel is asymmetric (e.g. -5..+8)
    //              RCS     -> 0 N*m     (no yaw torque)
    //
    // cutoffVal / offUs : optional un-modelled "off" state, for actuators
    //              whose regression does not reach their rest position. A
    //              command at or below cutoffVal writes offUs directly
    //              instead of going through the regression.
    //
    //              Only the EDF needs this. Its thrust curve is only fitted
    //              over 25-60 N, so drive(0) would otherwise clamp up to
    //              25 N and spin the fan up in response to a shutdown.
    //              Extrapolating the fit to 0 N is no better -- it gives
    //              1201 us, which really produces about 11 N.
    //
    //              cutoffVal sits just above zero, NOT at minVal: a descent
    //              command of 24 N must saturate to the floor, not stop the
    //              motor. Nothing commands between cutoffVal and minVal in
    //              flight, so that gap is reachable only via neutral().
    //
    //              Gimbal and RCS leave these defaulted. Their rest positions
    //              are ordinary points on their regressions, so the branch
    //              never fires.
    Actuator(uint8_t pin,
             float p0, float p1, float p2,
             float minVal, float maxVal,
             float neutralVal,
             float cutoffVal = -FLT_MAX,
             uint16_t offUs  = 0);

    virtual void initialize();
    virtual void drive(float value);

    // Drive to the rest position.
    void neutral() { drive(_neutral); }

    // Bypasses the regression and every limit. For ESC priming and other
    // raw-pulse procedures only; never from the control loop.
    void writeRaw(uint16_t us);

    // Hard bounds on the pulse drive() may emit, applied after the
    // regression. Does not affect the cutoff/off pulse or writeRaw(). The EDF
    // uses this for its flight floor above the thrust notch.
    void setPwmLimits(uint16_t minUs, uint16_t maxUs);

    // Pulse last written to the pin, from any path.
    uint16_t getLastPwmUs() const { return _lastUs; }

    // Inverse of toPwm(): pulse (us) -> natural units. Used by the servo
    // bench mode so a raw 5 us nudge still has an angle next to it.
    float fromPwm(uint16_t us) const;

    float getCurrentValue() const { return _currentValue; }
    float getMin()          const { return _min; }
    float getMax()          const { return _max; }
    float getNeutral()      const { return _neutral; }

    // True when the last command fell through to the raw off pulse. Always
    // false for actuators that did not configure a cutoff.
    bool  isCutOff()        const { return _cutOff; }

protected:
    float toPwm(float value) const;

    // Rounds, applies the PWM limits, writes, and records the pulse.
    void writeRegressed(float value);

    Servo    _servo;
    uint8_t  _pin;
    float    _p0, _p1, _p2;
    float    _min, _max;
    float    _neutral;
    float    _currentValue;
    float    _cutoff;
    uint16_t _offUs;
    bool     _cutOff;
    uint16_t _minUs  = 0;
    uint16_t _maxUs  = UINT16_MAX;
    uint16_t _lastUs = 0;
};
