#pragma once
#include "Imu.h"
#include "Lidar.h"
#include "OpticalFlow.h"

// ─────────────────────────────────────────────────────────────────────────────
// SensorManager
//
// Polls every sensor once per control cycle and presents the results as a
// single coherent frame. It does NOT fuse, rotate, filter or estimate -- that
// is the Estimator's job. The split matters because V0.1's Sensors class did
// both, so there was no way to test the Kalman filter without a BNO080 on the
// bench, and no way to see whether a bad altitude came from the sensor or the
// filter.
//
//     SensorManager  = hardware in, calibrated measurements out
//     Estimator      = measurements in, state out
//
// The two are peers, and main.cpp drives them in that order.
//
// Dependency injection, matching ActuatorManager: the sensors are constructed
// in main.cpp and passed in by reference. A bench harness can inject fakes
// that replay a CSV instead of talking to hardware.
//
// A sensor that fails initialize() is not fatal. Its sample() returns false
// forever, its frame data goes stale, and the Estimator simply never gets that
// measurement -- which is exactly how V0.1 flew, with flow_init() commented
// out. What is different is that the condition is now visible through
// isFlowHealthy() rather than implied by a commented-out line.
//
// TELEMETRY
// ─────────
// Nothing here prints. Everything a log needs is reachable as a const
// reference: getFrame() for the measurements, or getImu()/getLidar()/
// getFlow() for per-sensor diagnostics including sample and error counts. The
// future Telemetry class takes those same references and formats them; no
// class in the sensor path needs to change to support it.
// ─────────────────────────────────────────────────────────────────────────────

// A zero-copy view of one cycle's measurements. Holds references to the
// sensors' own structs, so building one costs three pointers -- the ImuData is
// not duplicated. Valid until the next sampleAll().
struct SensorFrame {
    const ImuData&   imu;
    const LidarData& lidar;
    const FlowData&  flow;

    // Whether each sensor produced a NEW reading in the cycle this frame
    // describes. The estimator uses these to decide which rows of H to enable,
    // replacing V0.1's shared data.status bitfield.
    bool imuFresh;
    bool lidarFresh;
    bool flowFresh;

    uint32_t timestampUs;
};

class SensorManager {
public:
    SensorManager(Imu& imu, Lidar& lidar, OpticalFlow& flow);

    // Brings up every sensor. Returns true only if all three answered, but
    // always attempts all three -- one missing sensor must not prevent the
    // others from coming up. Check the individual health getters to find out
    // which one failed.
    bool initialize();

    // IMU only -- call on every loop() pass, before any timer gate.
    void pollImu() { _imu.sample(); }

    // Lidar + one more IMU read. Call on the 5 ms tick.
    void sampleAll();

    // V0.1 ran flow at half rate (10 ms). Call on that timer, not from sampleAll().
    void sampleFlow();

    // ── Measurements ──────────────────────────────────────────────────────
    SensorFrame getFrame() const;

    // ── Direct access, for diagnostics and telemetry ───────────────────────
    const Imu&         getImu()   const { return _imu; }
    const Lidar&       getLidar() const { return _lidar; }
    const OpticalFlow& getFlow()  const { return _flow; }

    // Non-const IMU access, for the calibration commands main dispatches.
    Imu& imu() { return _imu; }

    // ── Health ────────────────────────────────────────────────────────────
    // Attitude is the only measurement the vehicle cannot fly without, so it
    // is the only one that gates arming.
    bool isImuHealthy(uint32_t nowUs)   const;
    bool isLidarHealthy(uint32_t nowUs) const;
    bool isFlowHealthy(uint32_t nowUs)  const;

    bool imuFresh()   const { return _imuFresh; }
    bool lidarFresh() const { return _lidarFresh; }
    bool flowFresh()  const { return _flowFresh; }

    uint32_t getTimestampUs() const { return _timestampUs; }

private:
    Imu&         _imu;
    Lidar&       _lidar;
    OpticalFlow& _flow;

    bool     _imuFresh    = false;
    bool     _lidarFresh  = false;
    bool     _flowFresh   = false;
    uint32_t _timestampUs = 0;
};
