#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

struct backstepping_flags
{
    bool pilot_override;

};

class AC_Backstepping
{
public:
    AC_Backstepping(const AP_AHRS_View& ahrs, //const AP_InertialNav& inav,
                    const AP_Motors& motors,
                    AC_AttitudeControl& attitude_control);

    void get_pos(int16_t pos_y, int16_t pos_z);

    void update_alt_controller();
    void update_lateral_controller();

private:
    int16_t _pos_y;
    int16_t _pos_z;
    // y and z velocity

    float _throttle_out;    // from 0-1
    float roll_angle_desired;

    backstepping_flags _flags;
};
