#include "AC_PathPlanner.h"

extern const AP_HAL::HAL &hal;

AC_PathPlanner::AC_PathPlanner()
{
    _nwp = sizeof(_wp_y)/sizeof(*_wp_y);
    _wp_idx = 0;
    _timer = 0;

    _flags.atGoal = false;
    _flags.start_flight = false;
}

position_t AC_PathPlanner::run()
{
    _check_flight_init();

    if (_flags.start_flight && !_flags.atGoal)
    {
        // check if reached goal
        if (_wp_idx >= _nwp)
        {
            _flags.atGoal = true;
        }
        else
        {
            _pos_d.y = _wp_y[_wp_idx];
            _pos_d.z = _wp_z[_wp_idx];
        }

        uint32_t dt = AP_HAL::micros64() - _t0;
        _timer += dt; 


        if (_timer >= WAYPOINT_TIME_INTERVAL)
        {
            _wp_idx++;
            _timer = 0;
        }
    }
    else
    {
        _timer = 0;
        _wp_idx = 0;

        _pos_d.y = _wp_y[0];
        _pos_d.z = _wp_z[0];
    }

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, timer %d\n", _pos_d.y, _pos_d.z , _timer);
    return _pos_d;
}

void AC_PathPlanner::get_default_target(float yd, float zd)
{
    _wp_y[0] = yd;
    _wp_z[0] = zd;
}

void AC_PathPlanner::_check_flight_init()
{
    int ch7 = rc().channel(CH_7)->get_radio_in();

    if (ch7 > 1500)
    {
        _flags.start_flight = true;
    }
    else
    {
        _flags.start_flight = false;
        _flags.atGoal = false;
    }
}
