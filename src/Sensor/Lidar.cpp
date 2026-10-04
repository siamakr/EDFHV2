#include "Lidar.h"
#include "consts.h"
#include <Arduino.h>
#include <Wire.h>

Lidar::Lidar()
    : Sensor("lidar")
{}

bool Lidar::initialize()
{
    // Only I2C device on the vehicle, so the bus is brought up here. If a
    // second one is ever added, hoist this into main so the order is explicit.
    Wire.begin();
    delay(100);

    _lidar.configure(cfg::lidar::CONFIGURE_MODE, cfg::lidar::I2C_ADDRESS);
    delay(200);

    // The v3HP gives no identification response, so there is nothing to test
    // against -- a missing unit shows up as a stream of rejected readings and
    // a climbing dropout count rather than a failure here.
    _initialized = true;

    // Prime the first acquisition so the next sample() has something to read.
    _lidar.takeRange();
    return true;
}

bool Lidar::sample()
{
    if (!_initialized || _lidar.getBusyFlag() != 0) {
        return false;
    }

    // Read the acquisition that just finished, then start the next one.
    const uint16_t cm = _lidar.readDistance();
    _lidar.takeRange();

    _d.raw_cm = cm;

    const float z = (cm / 100.0f) - cfg::lidar::MOUNT_OFFSET_M;

    if (z < cfg::lidar::MIN_VALID_M || z > cfg::lidar::MAX_VALID_M) {
        _d.valid = false;
        markError();
        return false;
    }

    _d.z_m   = z;
    _d.valid = true;
    markSampled(micros());
    return true;
}
