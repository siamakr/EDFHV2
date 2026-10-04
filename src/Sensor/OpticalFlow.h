#pragma once
#include "Sensor.h"
#include <PMW3901.h>

// ─────────────────────────────────────────────────────────────────────────────
// OpticalFlow -- PMW3901 over SPI
//
// Reports UNITLESS angular flow (1/s), not velocity. Turning it into m/s needs
// the height above ground and gyro compensation, both of which live in the
// Estimator -- this class does not know how high the vehicle is and should not.
// The conversion here is only pixels -> rate:
//
//     flow = (dpixels / dt) / FOCAL_PIXELS
//
// WHY dt IS MEASURED HERE, when everything else uses cfg::loop::DT_S
// ─────────────────────────────────────────────────────────────────────
// The control loop runs on a fixed 5 ms schedule, so the estimator's A and B
// are built once from the DT_S constant. This sensor is different: the PMW3901
// accumulates pixel counts continuously, so the count returned depends on the
// real elapsed time since the last read, whatever that happened to be. Using
// the nominal 5 ms when the true gap was 7 ms scales the velocity by 0.7.
//
// So dt is measured, but bounded: a gap outside [MIN_DT_S, MAX_DT_S] means
// either the first call after boot (where the previous timestamp is
// meaningless) or a loop that fell badly behind. Either way the sample is
// dropped rather than converted into a huge bogus velocity. V0.1 had no such
// guard, and its very first flow reading was computed against a dt of
// micros()/1e6 -- i.e. the time since power-on.
// ─────────────────────────────────────────────────────────────────────────────

struct FlowData {
    // Angular flow, 1/s. Multiply by height to get m/s.
    float vx = 0.0f;
    float vy = 0.0f;

    // Raw pixel deltas, for diagnosing a blinded or unfocused sensor
    int16_t dx_px = 0;
    int16_t dy_px = 0;

    // Interval this sample was computed against
    float dt_s = 0.0f;

    // Dead-reckoned integral of the UNCOMPENSATED flow. Drifts, and is not
    // used for control -- it exists so a log can be compared against the
    // estimator's fused position.
    float xInt = 0.0f;
    float yInt = 0.0f;
};

// NOTE ON THE CHIP SELECT PIN
// ───────────────────────────
// This constructor deliberately takes no pin. The PMW3901 driver hardcodes
// _cs = 29 in its own constructor and exposes no way to change it, so a pin
// argument here would be accepted and silently ignored -- worse than not
// offering one. cfg::pin::FLOW_CS records the value for reference only; moving
// the sensor means editing lib/PWM3901/PMW3901.cpp.
class OpticalFlow : public Sensor {
public:
    OpticalFlow();

    bool initialize() override;
    bool sample() override;

    const FlowData& getData() const { return _d; }

    // Turns the sensor LED on/off. Useful on the bench to confirm the part is
    // alive and to check surface illumination.
    void setLed(bool on);

private:
    PMW3901  _flow;
    FlowData _d;
    uint32_t _lastSampleUsFlow = 0;   // 0 = no previous sample yet
};
