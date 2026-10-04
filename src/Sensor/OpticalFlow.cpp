#include "OpticalFlow.h"
#include "consts.h"
#include <Arduino.h>

OpticalFlow::OpticalFlow()
    : Sensor("flow")
{}

bool OpticalFlow::initialize()
{
    if (!_flow.begin()) {
        _initialized = false;
        return false;
    }
    _flow.setLed(true);
    _initialized      = true;
    _lastSampleUsFlow = 0;   // force the first interval to be discarded
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

    int16_t dx = 0, dy = 0;
    _flow.readMotionCount(&dx, &dy);

    _d.dx_px = dx;
    _d.dy_px = dy;

    // First call after init: there is no valid previous timestamp to difference
    // against, so record it and wait for the next one.
    if (_lastSampleUsFlow == 0) {
        _lastSampleUsFlow = now;
        return false;
    }

    const float dt = (now - _lastSampleUsFlow) / 1000000.0f;
    _lastSampleUsFlow = now;
    _d.dt_s = dt;

    if (dt < cfg::flow::MIN_DT_S || dt > cfg::flow::MAX_DT_S) {
        markError();
        return false;
    }

    _d.vx = ((float)dx / dt) / cfg::flow::FOCAL_PIXELS;
    _d.vy = ((float)dy / dt) / cfg::flow::FOCAL_PIXELS;

    _d.xInt += _d.vx * dt;
    _d.yInt += _d.vy * dt;

    markSampled(now);
    return true;
}
