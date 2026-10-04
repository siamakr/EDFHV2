#pragma once
#include "Sensor.h"
#include <LIDARLite_v3HP.h>

// ─────────────────────────────────────────────────────────────────────────────
// Lidar -- Garmin LIDAR-Lite v3HP over I2C
//
// Downward-facing altimeter. Reports height of the LANDING FEET above ground,
// not height of the sensor: the mount offset is subtracted here so nothing
// downstream has to know where on the airframe the unit is bolted. V0.1
// subtracted a bare `- 0.08f` inline in sample_lidar().
//
// The v3HP is used in its non-blocking mode: each call triggers the next
// acquisition and reads back the one before it. So a reading is always one
// cycle old, which at 200 Hz is 5 ms and is why the estimator treats it as a
// position measurement rather than differentiating it aggressively.
//
// Dropout rejection is new. When the beam misses the surface -- over a lip,
// through a gap in the pad, or off the end of the range -- the part returns 0
// or a wild value. V0.1 fed that straight into the Kalman update as a height,
// which yanks the altitude estimate and, through the z/vz gains, the thrust
// command. Out-of-range readings are now dropped and counted instead.
// ─────────────────────────────────────────────────────────────────────────────

struct LidarData {
    float    z_m     = 0.0f;   // corrected height of the feet above ground
    uint16_t raw_cm  = 0;      // as returned by the device
    bool     valid   = false;  // false if the last reading was rejected
};

class Lidar : public Sensor {
public:
    Lidar();

    bool initialize() override;
    bool sample() override;

    const LidarData& getData() const { return _d; }

    float    getZ()        const { return _d.z_m; }
    uint32_t getDropouts() const { return getErrorCount(); }

private:
    LIDARLite_v3HP _lidar;
    LidarData      _d;
};
