#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_FakeSensor/AP_FakeSensor.h"
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#define WAYPOINT_TIME_INTERVAL  8000*1000    // ms*1000

class AC_PathPlanner
{
public:

    AC_PathPlanner();
    position_t run();
    position_t get_target_pos();
    void get_default_target(float yd, float zd);

private:
    struct flags_t
    {
        bool atGoal = false;
        bool start_flight = false;
    }_flags;

    position_t _pos;
    position_t _pos_d;   // target position

    uint64_t _timer = 0;    // ms * 10
    uint64_t _t0;

    // square
    float _wp_y[6] = {0  , 0.6, 0.6, -0.6, -0.6, 0};
    float _wp_z[6] = {0.6, 0.6, 1.7,  1.7,  0.6, 0.6};

    // X
    //float _wp_y[6] = {0, -0.55, 0.55, -0.55, 0.55, 0};
    //float _wp_z[6] = {0.6, 0.6, 1.7, 1.7, 0.6, 0.6};

    // up down
    //float _wp_y[3] = {0, 0, 0};
    //float _wp_z[3] = {0.6, 1.6, 0.6};

    uint16_t _wp_idx = 0;   // current waypoint index;
    uint16_t _nwp;          // number of waypoints

    void _check_flight_init();
};
