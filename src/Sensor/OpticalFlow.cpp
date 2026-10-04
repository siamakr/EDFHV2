#include "OpticalFlow.h"
#include "consts.h"
#include <Arduino.h>
#include <SPI.h>

OpticalFlow::OpticalFlow()
    : Sensor("flow")
{}

bool OpticalFlow::initialize()
{
    // Same SPI1 pins V0.1 used. MOSI was left on the Teensy default in that
    // driver; set it here so begin() is not at the mercy of whoever last
    // touched SPI1.
    SPI1.setMOSI(26);
    SPI1.setMISO(1);
    SPI1.setSCK(27);

    if (!_flow.begin()) {
        _initialized = false;
        return false;
    }
    _flow.setLed(true);
    _initialized = true;
    _lastUs = 0;
    return true;
}

void OpticalFlow::setLed(bool on)
{
    if (_initialized) _flow.setLed(on);
}

bool OpticalFlow::sample()
{
    if (!_initialized) return false;

    const uint32_t now = micros();
    float dt = (now - _lastUs) / 1000000.0f;
    _lastUs = now;

    int16_t dx = 0, dy = 0;
    _flow.readMotionCount(&dx, &dy);

    _d.dx_px = dx;
    _d.dy_px = dy;
    _d.dt_s  = dt;

    if (dt > 0.0f) {
        _d.vx = ((float)dx / dt) / cfg::flow::FOCAL_PIXELS;
        _d.vy = ((float)dy / dt) / cfg::flow::FOCAL_PIXELS;
        _d.xInt += _d.vx * dt;
        _d.yInt += _d.vy * dt;
    }

    markSampled(now);
    return true;
}
