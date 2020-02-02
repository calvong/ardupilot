#include "AC_PathPlanner.h"

extern const AP_HAL::HAL &hal;

AC_PathPlanner::AC_PathPlanner()
{
    _wp_idx = 0;
    _timer = 0;

    _pos_d.ay = 0;
    _pos_d.az = 0;
    _pos_d.vy = 0;
    _pos_d.vz = 0;
    _pos_d.y = _wp_yd[0];
    _pos_d.z = _wp_zd[0];

    // calculate time for line trajectory
    float line_d = sqrt((_line_yd[0]-_line_yd[1])*(_line_yd[0]-_line_yd[1]) + (_line_zd[0] - _line_zd[1])*(_line_zd[0] - _line_zd[1]));

    _line_period = 2 * sqrt(line_d/ad);

    // pre-calculate for waypoint trajectory
    for (int i=0; i<N_WP-1; i++)
    {
        // compute time
        float wp_d = sqrt((_wp_yd[i]-_wp_yd[i+1])*(_wp_yd[i]-_wp_yd[i+1]) + (_wp_zd[i]-_wp_zd[i+1])*(_wp_zd[i]-_wp_zd[i+1]));

        _wp_period[i] = sqrt(wp_d/ad);  // time from point A to point B

        // compute acceleration angle
        if ((_wp_yd[i]-_wp_yd[i+1]) == 0)
        {
            _wp_y_accel_angle[i] = 0;

            if ((_wp_zd[i]-_wp_zd[i+1]) < 0)    _wp_z_accel_angle[i] = 1;
            else                                _wp_z_accel_angle[i] = -1;
        }
        else
        {
            float the = atan2((_wp_zd[i+1]-_wp_zd[i]), (_wp_yd[i+1]-_wp_yd[i]));

            _wp_y_accel_angle[i] = cos(the);
            _wp_z_accel_angle[i] = sin(the);
        }
    }

    _flags.atGoal = false;
    _flags.start_flight = false;
}

position_t AC_PathPlanner::run_setpoint()
{
    _check_flight_init();

    _flags.start_flight = false; // only run a single setpoint

    float dt = (float) (AP_HAL::micros64() - _t0)*0.001f*0.001f;

    _ftimer += dt;

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, timer %d\n", _pos_d.y, _pos_d.z , _timer);
    return _pos_d;
}

position_t AC_PathPlanner::run_waypoint_trajectory()
{
    _check_flight_init();

    if (_flags.start_flight && !_flags.atGoal)
    {
        float dt = (float) (AP_HAL::micros64() - _t0)*0.001f*0.001f;

        if (_wp_idx == 0)
        {
            // position
            _pos_d.y = _wp_yd[0];
            _pos_d.z = _wp_zd[0];


            // velocity
            _pos_d.vy = 0;//_cir_radius * _w * M_PI;
            _pos_d.vz = 0;

            // no init velocity and init position = setpoints
            _v0_y = 0;
            _v0_z = 0;
            _y0 = _pos_d.y;
            _z0 = _pos_d.z;

            // check if reached the starting point
            if (fabs(_pos_d.y - _pos.y) < DIST_THRES && fabs(_pos_d.z - _pos.z) < DIST_THRES)
            {
                _wp_idx++;
            }
        }
        else if ((_wp_idx > 0) && (_wp_idx < 50))
        {
            // acceleration
            _pos_d.ay = ad * _accel_dir * _wp_y_accel_angle[_wp_idx-1];
            _pos_d.az = ad * _accel_dir * _wp_z_accel_angle[_wp_idx-1];

            // velocity
            _pos_d.vy = _v0_y + _pos_d.ay * _ftimer;
            _pos_d.vz = _v0_z + _pos_d.az * _ftimer;

            // position
            _pos_d.y = _y0 + _v0_y * _ftimer + 0.5f * _pos_d.ay * _ftimer * _ftimer;
            _pos_d.z = _z0 + _v0_z * _ftimer + 0.5f * _pos_d.az * _ftimer * _ftimer;

            _ftimer += dt;

            if (_ftimer >= _wp_period[_wp_idx - 1])
            {
                if (_wp_state == 's')
                {
                    _ftimer = 0;

                    _y0 = _pos_d.y;
                    _z0 = _pos_d.z;
                    _v0_y = _pos_d.vy;
                    _v0_z = _pos_d.vz;

                    _accel_dir = -1;

                    _wp_state = 'm';
                }
                else if (_wp_state == 'm')
                {
                    if ((_ftimer - _wp_period[_wp_idx - 1] - WP_WAIT_TIME) < 0)
                    {
                        // make sure staying at the same point
                        _pos_d.ay = 0;
                        _pos_d.az = 0;

                        _pos_d.vy = 0;
                        _pos_d.vz = 0;

                        _pos_d.y = _wp_yd[_wp_idx];
                        _pos_d.z = _wp_zd[_wp_idx];

                        _v0_y = 0;
                        _v0_z = 0;

                    }
                    else
                    {
                        // reseting init position and velocity for the next waypoint
                        _y0 = _wp_yd[_wp_idx];
                        _z0 = _wp_zd[_wp_idx];

                        _v0_y = 0;
                        _v0_z = 0;

                        _wp_idx++;  // moving on to the next waypoint
                        _accel_dir = 1;
                        _ftimer = 0;
                        _wp_state = 's';

                    }
                }
            }

            // deal with end of waypoint
            if (_wp_idx >= N_WP) _wp_idx = 99;
        }
        else if (_wp_idx == 99)
        {
            // make sure staying at the same point
            _pos_d.ay = 0;
            _pos_d.az = 0;

            _pos_d.vy = 0;
            _pos_d.vz = 0;

            _pos_d.y = _wp_yd[N_WP-1];
            _pos_d.z = _wp_zd[N_WP-1];
        }

    }

    hal.uartA->printf("yd %f, zd %f, t %f, i %d, state %c, dir %d, ya %f, za %f , yv %f, zv %f\n", _pos_d.y, _pos_d.z , _ftimer, _wp_idx, _wp_state, _accel_dir, _pos_d.ay, _pos_d.az, _v0_y, _v0_z);
    _t0 = AP_HAL::micros64();

    return _pos_d;
}


position_t AC_PathPlanner::run_circular_trajectory()
{
    _check_flight_init();

    if (_flags.start_flight && !_flags.atGoal)
    {
        float dt = (float) (AP_HAL::micros64() - _t0)*0.001f*0.001f;

        if (_wp_idx == 0)
        {
            // position
            _pos_d.y = 0;
            _pos_d.z = _cir_height_offset + _cir_radius;

            // velocity
            _pos_d.vy = 0;//_cir_radius * _w * M_PI;
            _pos_d.vz = 0;

            // check if reached the starting point
            if (fabs(_pos_d.y - _pos.y) < DIST_THRES && fabs(_pos_d.z - _pos.z) < DIST_THRES)
            {
                _wp_idx++;
            }

        }
        else if (_wp_idx == 1)
        {
            // position
            _pos_d.y = _cir_radius * sin(_w*M_PI*_ftimer);
            _pos_d.z = _cir_height_offset + _cir_radius * cos(_w*M_PI*_ftimer);

            // velocity
            _pos_d.vy = _cir_radius * _w * M_PI * cos(_w*M_PI*_ftimer);
            _pos_d.vz = - _cir_radius * _w * M_PI * sin(_w*M_PI*_ftimer);

            // acceleration
            _pos_d.ay = -_cir_radius * _w * _w * M_PI * M_PI * sin(_w*M_PI*_ftimer);
            _pos_d.az = -_cir_radius * _w * _w * M_PI * M_PI * cos(_w*M_PI*_ftimer);

            _ftimer += dt;
        }


    }

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, vyd %f, vzd %f, y %f, z %f\n", _pos_d.y, _pos_d.z,_pos_d.vy, _pos_d.vz, _pos.y, _pos.z);
    return _pos_d;
}

position_t AC_PathPlanner::run_line_trajectory()
{
    _check_flight_init();

    static float last_v0_y;
    static float last_v0_z;
    static float last_y0;
    static float last_z0;

    if (_flags.start_flight)
    {
        float dt = (float) (AP_HAL::micros64() - _t0)*0.001f*0.001f;    // second

        if (_wp_idx == 0)
        {
            // position
            _pos_d.y = _line_yd[0];
            _pos_d.z = _line_zd[0];

            // velocity
            _pos_d.vy = 0;//_cir_radius * _w * M_PI;
            _pos_d.vz = 0;

            // check if reached the starting point
            if (fabs(_pos_d.y - _pos.y) < DIST_THRES && fabs(_pos_d.z - _pos.z) < DIST_THRES)
            {
                _wp_idx++;
            }
        }
        else
        {
            if (_ftimer <= _line_period * 0.5f)  // +ve accel - from point 1 to point 2
            {
                _accel_dir = 1;

                _v0_y = 0;
                _v0_z = 0;

                _y0 = 0;
                _z0 = 0;

                last_y0 = _pos_d.y;
                last_z0 = _pos_d.z;

                last_v0_y = _pos_d.vy;
                last_v0_z = _pos_d.vz;
            }
            else if ((_ftimer < _line_period) && (_ftimer > _line_period * 0.5f))   // +ve accel - from point 2 to point 1
            {
                _accel_dir = -1;

                _v0_y = last_v0_y;
                _v0_z = last_v0_z;

                _y0 = last_y0;
                _z0 = last_z0;
            }
            else
            {
                _ftimer = 0;    // reset timer and trajectory
            }

            // acceleration
            _pos_d.ay = 0;
            _pos_d.az = 0;

            // TODO: bug fix
            if ((_line_yd[0]-_line_yd[1]) != 0) _pos_d.ay = ad * _accel_dir;
            if ((_line_zd[0]-_line_zd[1]) != 0) _pos_d.az = ad * _accel_dir;

            // velocity
            _pos_d.vy = _pos_d.ay * _ftimer;
            _pos_d.vz = _pos_d.az * _ftimer;

            // position
            _pos_d.y = _y0 + _v0_y * _ftimer + 0.5 * _pos_d.ay * _ftimer*_ftimer;
            _pos_d.z = _z0 + _v0_z * _ftimer + 0.5 * _pos_d.az * _ftimer*_ftimer;

            _ftimer += dt;
        }
    }

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, vyd %f, vzd %f, y %f, z %f\n", _pos_d.y, _pos_d.z,_pos_d.vy, _pos_d.vz, _pos.y, _pos.z);
    return _pos_d;
}

float AC_PathPlanner::pitch_oscillator(float pilot_pitch)
{
    _check_flight_init();

    float target_pitch = 0;

    float manual_pitch_trim = pilot_pitch/2;

    // in centidegree
    if (_flags.start_flight)
    {
        target_pitch = PITCH_OSCILLATION * sin(2*M_PI/PITCH_PERIOD * _ftimer) * 100 + manual_pitch_trim;
    }
    else
    {
        target_pitch = pilot_pitch;
    }

    //hal.uartA->printf("target pitch: %f, time: %f, manual: %f\n", target_pitch, _ftimer, manual_pitch_trim);

    return target_pitch;
}


float AC_PathPlanner::pitch_forward(float pilot_pitch)
{
    _check_flight_init();

    float target_pitch = 0;

    float manual_pitch_trim = pilot_pitch/2;

    // in centidegree
    if (_flags.start_flight)
    {
        target_pitch = PITCH_OSCILLATION * 100 + manual_pitch_trim;
    }
    else
    {
        target_pitch = pilot_pitch;
    }

    //hal.uartA->printf("target pitch: %f, time: %f, manual: %f\n", target_pitch, _ftimer, manual_pitch_trim);

    return target_pitch;
}

void AC_PathPlanner::get_default_target(float yd, float zd)
{
    _wp_y[0] = yd;
    _wp_z[0] = zd;

    _pos_d.y = yd;
    _pos_d.z = zd;
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
        _flags.yGoal = false;
        _flags.zGoal = false;
        _pos_d.y = _wp_yd[0];
        _pos_d.z = _wp_zd[0];
        _timer = 0;
        _ftimer = 0;
        _wp_idx = 0;
    }
}

void AC_PathPlanner::get_current_pos(position_t pos)
{
    _pos = pos;
}
