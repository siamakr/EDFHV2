#include "Lidar.h"
#include "consts.h"
#include <Arduino.h>
#include <Wire.h>

Lidar::Lidar()
    : Sensor("lidar")
{}

bool Lidar::initialize()
{
    Wire.begin();
    delay(100);

    _lidar.configure(cfg::lidar::CONFIGURE_MODE, cfg::lidar::I2C_ADDRESS);
    delay(200);

    _initialized = true;
    return true;
}

bool Lidar::sample()
{
    if (!_initialized || _lidar.getBusyFlag() != 0) {
        return false;
    }

    _lidar.takeRange();
    const uint16_t cm = _lidar.readDistance();
    _d.raw_cm = cm;

    const float z = (cm / 100.0f) - cfg::lidar::MOUNT_OFFSET_M;

    if (cm == 0 || z < cfg::lidar::MIN_VALID_M || z > cfg::lidar::MAX_VALID_M) {
        _d.valid = false;
        markError();
        return false;
    }

    _d.z_m   = z;
    _d.valid = true;
    markSampled(micros());
    return true;
}
