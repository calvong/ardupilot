#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_FakeSensor/AP_FakeSensor.h"
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#define WAYPOINT_TIME_INTERVAL  10000*1000  // ms*1000
#define ad                      0.1         // m/s^2, desired accleration
#define WP_WAIT_TIME            2           // s - wait time for each waypoint
#define N_WP                    5           // number of waypoints
#define DIST_THRES              0.1         // m
#define PITCH_OSCILLATION       5           // degree - for experiment only
#define PITCH_PERIOD            1           // second

class AC_PathPlanner
{
public:

    AC_PathPlanner();
    position_t run_setpoint();
    position_t run_circular_trajectory();
    position_t run_line_trajectory();
    position_t run_waypoint_trajectory();
    position_t get_target_pos();
    void get_current_pos(position_t pos);
    void get_default_target(float yd, float zd);

    // for experiment only
    float pitch_oscillator(float pilot_pitch);
    float pitch_forward(float pilot_pitch);

private:
    struct flags_t
    {
        bool atGoal = false;
        bool start_flight = false;
        bool yGoal = false;
        bool zGoal = false;

    }_flags;

    position_t _pos;
    position_t _pos_d;   // target position

    float _ftimer = 0;
    uint64_t _timer = 0;    // ms * 10
    uint64_t _t0;

    // circular trajectory parameters
    float _cir_radius = -0.3;            // circle radius
    float _cir_height_offset = 0.4;     // height offset
    float _w = 0.3;                    // time constant/frequency : small = slow

    // line trajectory parameters - OBSOLETE (to be removed)
    float _line_yd[2] = {0, 0};
    float _line_zd[2] = {0.3, 0.8};
    float _line_period;

    // waypoint trajectory parameters
    uint8_t _wp_idx = 0;   // current waypoint index;
    char _wp_state = 's';   // s:start, m:middle, e:end

    float _wp_yd[N_WP] = {0, 0.3, 0.3,-0.3, 0.3};
    float _wp_zd[N_WP] = {0.6, 0.6, 0.9, 0.6, 0.4};

    float _wp_period[N_WP-1];
    float _wp_y_accel_angle[N_WP-1];
    float _wp_z_accel_angle[N_WP-1];

    int _accel_dir = 1; // +1 or -1 for acceleration

    float _v0_y = 0;
    float _v0_z = 0;
    float _y0 = 0;
    float _z0 = 0;

    // obsolete code - to be removed in next commit
    float _wp_y[4] = {0.55,0.55,0.55,0.55};
    float _wp_z[4] = {0.4, 0.8, 1.2, 1.6};


    void _check_flight_init();
};
