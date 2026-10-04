#include "SensorManager.h"
#include "consts.h"
#include <Arduino.h>

SensorManager::SensorManager(Imu& imu, Lidar& lidar, OpticalFlow& flow)
    : _imu(imu)
    , _lidar(lidar)
    , _flow(flow)
{}

bool SensorManager::initialize()
{
    // Every sensor gets a chance to come up even if an earlier one failed --
    // no short-circuiting. A vehicle with a dead flow sensor should still be
    // able to hold attitude and altitude on the bench.
    // V0.1 setup(): flow, lidar, IMU.
    const bool flowOk  = _flow.initialize();
    const bool lidarOk = _lidar.initialize();
    delay(100);
    const bool imuOk   = _imu.initialize();

    // Seed the attitude solution and capture the heading that counts as zero.
    if (imuOk) {
        _imu.sample();
        _imu.setYawOrigin();
    }

    return imuOk && lidarOk && flowOk;
}

void SensorManager::sampleAll()
{
    // Latch each sensor's answer for this cycle. These flags stay valid until
    // the next sampleAll(), so there is no "who clears the flag" ordering
    // hazard -- V0.1 cleared status bits inside run_estimator(), which meant
    // calling the estimator twice silently discarded measurements.
    _imuFresh   = _imu.sample();
    _lidarFresh = _lidar.sample();
    _timestampUs = micros();
}

void SensorManager::sampleFlow()
{
    _flowFresh = _flow.sample();
}

SensorFrame SensorManager::getFrame() const
{
    return SensorFrame{
        _imu.getData(),
        _lidar.getData(),
        _flow.getData(),
        _imuFresh,
        _lidarFresh,
        _flowFresh,
        _timestampUs
    };
}

bool SensorManager::isImuHealthy(uint32_t nowUs) const
{
    return !_imu.isStale(nowUs, cfg::imu::STALE_LIMIT_US);
}

bool SensorManager::isLidarHealthy(uint32_t nowUs) const
{
    return !_lidar.isStale(nowUs, cfg::lidar::STALE_LIMIT_US);
}

bool SensorManager::isFlowHealthy(uint32_t nowUs) const
{
    return !_flow.isStale(nowUs, cfg::flow::STALE_LIMIT_US);
}
