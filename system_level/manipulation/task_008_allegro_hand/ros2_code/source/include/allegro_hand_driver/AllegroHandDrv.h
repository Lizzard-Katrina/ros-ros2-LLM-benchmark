#ifndef __ALLEGROHAND_DRV_H__
#define __ALLEGROHAND_DRV_H__

#include <fcntl.h>
#include <list>
#include <string>
#include "AllegroHandDef.h"

namespace allegro
{

class AllegroHandDrv
{

public:
    AllegroHandDrv();
    ~AllegroHandDrv();

    bool init(int mode = 0);

    void setTorque(double *torque);
    void getJointInfo(double *position);

    bool emergencyStop() { return _emergency_stop; }
    double torqueConversion() { return _tau_cov_const; }
    double inputVoltage() { return _input_voltage; }

    int readCANFrames();
    int writeJointTorque();
    bool isJointInfoReady();
    void resetJointInfoReady();

    bool HAND_TYPE_A;
    bool RIGHT_HAND;

    // For testing: allow injecting position data
    void injectJointPositions(double *positions);
    void setAllJointsReady();

    // Sim mode: continuously mark joints as ready
    void setSimMode(bool sim);

private:
    double _curr_position[DOF_JOINTS];
    double _curr_torque[DOF_JOINTS];
    double _desired_position[DOF_JOINTS];
    double _desired_torque[DOF_JOINTS];

    double _hand_version;
    double _tau_cov_const;
    double _input_voltage;

    int _curr_position_get;

    double _pwm_max_global;
    double _pwm_max[DOF_JOINTS];
    int    _encoder_offset[DOF_JOINTS];
    int    _encoder_direction[DOF_JOINTS];
    int    _motor_direction[DOF_JOINTS];

    volatile bool _emergency_stop;
    bool _sim_mode;

private:
    void _readDevices();
    void _writeDevices();
    void _parseMessage(int id, int len, unsigned char* data);
};

}

#endif // __ALLEGROHAND_DRV_H__