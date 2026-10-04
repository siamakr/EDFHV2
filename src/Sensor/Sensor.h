#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Sensor
//
// Common lifecycle and freshness bookkeeping for every physical sensor. What
// the subclasses share is NOT their data -- an IMU, a lidar and a flow sensor
// produce completely different shapes -- it is the answers to:
//
//     did you come up on the bus?
//     do you have something new since I last looked?
//     when did you last produce a reading?
//     are you still producing them?
//
// So this base owns exactly that, and each subclass exposes its own concrete
// data struct through its own getter. No attempt is made to force a common
// read() return type; that would either lose information or push everything
// through a lowest-common-denominator float array.
//
// This replaces V0.1's single `data.status` bitfield, where the freshness bits
// for all three sensors lived in the shared fsm_data_t struct and any sensor
// could clear any other's flag. Freshness now belongs to the sensor that
// produced it.
//
// Nothing here touches Serial. Diagnostics are exposed as state for the caller
// to print or log; see the note on the telemetry boundary in SensorManager.h.
// ─────────────────────────────────────────────────────────────────────────────

class Sensor {
public:
    explicit Sensor(const char* name) : _name(name) {}
    virtual ~Sensor() = default;

    // Bring the device up. Returns false if it did not respond; the caller
    // decides whether that is fatal. V0.1 spun in `while(1)` inside the sensor
    // itself, which made a missing sensor indistinguishable from a hang and
    // left no way to run the rest of the vehicle on the bench.
    virtual bool initialize() = 0;

    // Poll the device. Returns true if a NEW reading was produced this call.
    // Must not block and must not delay().
    virtual bool sample() = 0;

    bool        isInitialized()  const { return _initialized; }
    bool        hasNewData()     const { return _newData; }
    const char* getName()        const { return _name; }
    uint32_t    getSampleCount() const { return _sampleCount; }
    uint32_t    getErrorCount()  const { return _errorCount; }
    uint32_t    getLastSampleUs() const { return _lastSampleUs; }

    // Cleared by whoever consumed the reading -- the estimator, once it has
    // folded the measurement in.
    void clearNewData() { _newData = false; }

    // True when the device has gone quiet for longer than it should have.
    bool isStale(uint32_t nowUs, uint32_t limitUs) const
    {
        return !_initialized || (nowUs - _lastSampleUs) > limitUs;
    }

protected:
    // Subclasses call this the moment they accept a reading.
    void markSampled(uint32_t nowUs)
    {
        _newData      = true;
        _lastSampleUs = nowUs;
        ++_sampleCount;
    }

    void markError() { ++_errorCount; }

    const char* _name;
    bool        _initialized  = false;
    bool        _newData      = false;
    uint32_t    _lastSampleUs = 0;
    uint32_t    _sampleCount  = 0;
    uint32_t    _errorCount   = 0;
};
