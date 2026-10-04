#include "OpticalFlow.h"
#include "consts.h"
#include <Arduino.h>
#include <SPI.h>

OpticalFlow::OpticalFlow()
    : Sensor("flow")
    , _flow(cfg::pin::FLOW_CS)
{}

bool OpticalFlow::initialize()
{
    SPI1.setMOSI(cfg::pin::FLOW_MOSI);
    SPI1.setMISO(cfg::pin::FLOW_MISO);
    SPI1.setSCK(cfg::pin::FLOW_SCK);

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
    const float dt = (now - _lastUs) / 1000000.0f;
    _lastUs = now;

    int16_t dx = 0, dy = 0;
    uint8_t squal = 0;
    _flow.readMotionCount(&dx, &dy, &squal);

    _d.dx_px  = dx;
    _d.dy_px  = dy;
    _d.squal  = squal;
    _d.dt_s   = dt;
    _d.vx     = 0.0f;
    _d.vy     = 0.0f;

    // First call after boot has a meaningless dt (time since power-on).
    if (dt < cfg::flow::MIN_DT_S || dt > cfg::flow::MAX_DT_S) {
        return false;
    }

    // PX4 drops quality-0 and |delta| > 240 (SPI garbage).
    if (squal < cfg::flow::MIN_QUALITY || dx > 240 || dy > 240 || dx < -240 || dy < -240) {
        return false;
    }

    // Angular rate, rad/s. Height and gyro compensation are the estimator's.
    _d.vx = ((float)dx / dt) / cfg::flow::FOCAL_PIXELS;
    _d.vy = ((float)dy / dt) / cfg::flow::FOCAL_PIXELS;
    _d.xInt += _d.vx * dt;
    _d.yInt += _d.vy * dt;

    markSampled(now);
    return true;
}
