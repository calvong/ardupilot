#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include "AP_FakeSensor/AP_FakeSensor.h"
#include <RC_Channel/RC_Channel.h>

#define G                                   -9.81 // gravity
#define DEFAULT_IMAX                        10
#define BACKSTEPPING_THROTTLE_CUTOFF_FREQ   2.0f    // low-pass filter on accel error (unit: hz)
#define THRUST_SCALE_FACTOR                 0.01f

//TODO: check inav vel direction +/-ve
class AC_Backstepping
{
public:
    // variables
    struct backstepping_flags
    {
        bool pilot_override;
        bool reset_integral;

    }flags;

    pos_error_t perr;

    // functions
    AC_Backstepping(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                    const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                    const AP_FakeSensor& fs);

    void set_dt(float delta_sec);
    void pos_update();
    void get_imax(float imax_y, float imax_z);
    void get_gains(float yk1, float yk2, float yk3, float zk1, float zk2, float zk3);
    void get_target_pos(float yd, float zd);
    float get_u1();

    void update_alt_controller();
    void update_lateral_controller();
    void reset_integral_y();
    void reset_integral_z();

    void write_log();

private:
    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    const AP_Motors&            _motors;
    AC_AttitudeControl&         _attitude_control;
    const AP_FakeSensor&        _fs;

    struct controller_gains_t
    {
        float k1_y;
        float k2_y;
        float k3_y;
        float k1_z;
        float k2_z;
        float k3_z;
    }_gains;

    struct position_t
    {
        float y;    // m
        float z;    // m

        float vel_y = 0;
        float vel_z = 0;

        float prev_ey = 0;    // previous pos y error
        float prev_ez = 0;    // previous pos z error

        float iey = 0;   // error integral of pos y
        float iez = 0;   // error integral of pos z
    }_pos;

    float _dt;
    float _pos_target_z;
    float _pos_target_y;

    float _imax_z;   // max integral term for pos z
    float _imax_y;   // max integral term for pos y

    float _thr_out;    // thrust output for motor, range from 0-1
    float _u1;               // thrust: raw controller output from altitude controller
    float _target_roll;     // desired roll output from lateral controller

    float _limit_integral(float gain, float current_err, char yz);
    float _limit_thrust(float thr);
};
