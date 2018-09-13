#pragma once

#include <AP_HAL/AP_HAL.h>
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
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#define G                                   -9.81f  // gravity
#define DEFAULT_IMAX                        10
#define BACKSTEPPING_THROTTLE_CUTOFF_FREQ   2.0f    // low-pass filter on accel error (unit: hz)
#define BACKSTEPPING_VEL_ERROR_CUTOFF_FREQ  5.0f
#define THRUST_SCALE_FACTOR                 0.01f
#define POS_ERROR_THRESHOLD                 1.5f    // in m, max allowed change in position
#define THROTTLE_TRANSITION_TIME            1.5f    // second
#define MANUAL_OVERRIDE_TIME                1.5f    // second
#define ROLL_DTERM_MAX                      sin(30*M_PI/180.0f)*0.36    // about 20 deg of d term

// defines for PID controller
#define PID_DTERM_MAX                       2000    // 20 centidegree
#define PID_ITERM_MAX                       1000    // 10 centidegree
#define PID_MAX_CLIMB_RATE                  150     // cm/s

class AC_Backstepping
{
public:
    // variables
    struct backstepping_flags
    {
        bool switched_mode;
        bool mode_transition_completed;
        bool manual_override;
    }flags;

    pos_error_t perr;

    // functions
    AC_Backstepping(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                    const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                    const AP_FakeSensor& fs);

    void pos_update(position_t pos);
    void get_imax(float imax_y, float imax_z);
    void get_gains(float yk1, float yk2, float yk3, float zk1, float zk2, float zk3);
    void get_target_pos(float yd, float zd);
    void get_target_vel(float vyd, float vzd);
    void get_pilot_lean_angle_input(float target_roll, float roll_max);
    float get_u1();

    void update_alt_controller();
    float update_lateral_controller();
    void reset_integral();
    void reset_mode_switch();
    void write_log();

    // PID lateral controller
    float update_PID_lateral_controller();
    float get_PID_alt_climb_rate();    // cm/s
    void get_PID_gains(float alt_p, float kp, float ki, float kd);
    void reset_PID_integral();

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

    position_t _pos;

    int _manual_counter;

    float _dt;
    float _pos_target_z;
    float _pos_target_y;
    float _vel_target_y;
    float _vel_target_z;
    int   _prev_nset;

    float _imax_z;   // max integral term for pos z
    float _imax_y;   // max integral term for pos y

    float _last_mode_thr_out;
    float _mode_switch_counter;
    float _thr_out;    // thrust output for motor, range from 0-1
    float _u1;               // thrust: raw controller output from altitude controller
    float _target_roll;     // desired roll to the attitude controller
    float _pilot_roll;      // pilot roll input
    float _BS_roll;         // desired roll output from backstepping
    float _roll_max;        // max roll angle

    float _angle_transition(float target_roll);
    float _throttle_transition(float BS_thr_out);
    float _limit_derivative(float d_term, float threshold);
    float _limit_integral(float gain, float current_err, char yz);
    float _limit_thrust(float thr);
    float _limit_sin_phi(float sp);
    float _rad2cdeg(float in);

    // PID lateral controller
    struct pid_gains_t
    {
        float alt_p;
        float kp;
        float ki;
        float kd;
    }_pid;

    float _pid_iey;
    float _pid_roll;

    float _limit_PID_integral(float error, float in);

};
