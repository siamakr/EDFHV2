#pragma once
#include <cstddef>
#include <cstdint>

#include "Estimator.h"
#include "Imu.h"
#include "PositionController.h"
#include "AttitudeController.h"
#include "ActuatorManager.h"
#include "Actuator.h"

// Sequences either a hover/land flight table or a static attitude hold-down
// table. Main still owns Serial and actuator routing for the other bench modes.

struct Waypoint {
    uint8_t number   = 0;
    char    name[16] = {};
    float   alt_m    = 0.0f;
    float   x_m      = 0.0f;
    float   y_m      = 0.0f;
    float   dwell_s  = 0.0f;
    bool    isLand   = false;
};

struct MissionEntry {
    const char* name;
    float alt_m, x_m, y_m, dwell_s;
};

class Wayfinder {
public:
    static constexpr uint8_t MAX_MISSION = 16;

    enum class Mode : uint8_t { Flight, AttitudeTest };

    Wayfinder(PositionController& position,
              AttitudeController& attitude,
              ActuatorManager&    actuators,
              Actuator&           edf,
              Imu&                imu);

    void setMode(Mode m) { _mode = m; }
    Mode mode() const    { return _mode; }

    // lidar_z_m is the altitude hold for AttitudeTest (ignored in Flight).
    void start(float lidar_z_m);
    void abort();

    void update(const StateEstimate& est);

    bool isActive()          const { return _active; }
    bool isComplete()        const { return _complete; }
    bool isLanded()          const { return _landed; }
    bool landingConfirmed()  const { return _landingConfirmed; }
    bool driveRcs()          const { return _mode == Mode::Flight; }

    const Waypoint& current() const { return _wp; }
    uint8_t index()           const { return _index; }
    float   dwellElapsed_s()  const { return _elapsedUs / 1000000.0f; }
    float   dwellTarget_s()   const;
    float   rollRef_deg()     const { return _rollRef_deg; }
    float   pitchRef_deg()    const { return _pitchRef_deg; }
    float   zRef_m()          const { return _zHold_m; }

    static const char* csvHeader();

    void formatLine(char* buf, size_t n,
                    const StateEstimate& est,
                    const ImuData& att,
                    float gimbalX_deg, float gimbalY_deg,
                    uint16_t edfUs) const;

    void formatCsv(char* buf, size_t n,
                   const StateEstimate& est,
                   const ImuData& att,
                   float gimbalX_deg, float gimbalY_deg,
                   uint16_t gimbalX_us, uint16_t gimbalY_us,
                   uint16_t edfUs) const;

private:
    void loadFlight(uint8_t number, const MissionEntry& entry);
    void loadAttitude(uint8_t index);
    void serviceFlight(const StateEstimate& est);
    void serviceAttitude();
    void cut();
    static bool inTolPct(float meas, float target, float frac, float zeroScale);

    PositionController& _position;
    AttitudeController& _attitude;
    ActuatorManager&    _actuators;
    Actuator&           _edf;
    Imu&                _imu;

    Mode _mode = Mode::Flight;

    MissionEntry _mission[MAX_MISSION]{};
    uint8_t      _missionLen = 0;

    Waypoint _wp;
    bool     _active            = false;
    bool     _complete          = false;
    bool     _landed            = false;
    bool     _landingConfirmed  = false;
    uint32_t _elapsedUs         = 0;
    uint32_t _startUs           = 0;
    uint8_t  _index             = 0;
    float    _prevAlt_m         = 0.0f;
    float    _zHold_m           = 0.0f;
    float    _rollRef_deg       = 0.0f;
    float    _pitchRef_deg      = 0.0f;
};
