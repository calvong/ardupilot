#include "AC_Backstepping.h"

extern const AP_HAL::HAL &hal;

AC_Backstepping::AC_Backstepping(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                 const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                                 const AP_FakeSensor& fs) :
                                 _ahrs(ahrs),
                                 _inav(inav),
                                 _motors(motors),
                                 _attitude_control(attitude_control),
                                 _fs(fs)
{
    // init variables
    _imax_z = _motors.get_throttle_hover() * 0.5f;  // TEMP
    _imax_y = sin(10 *M_PI/180.0f) * 0.36;  // 10 deg -> max roll angle for pos y integral

    _pos_target_z = 0.5f; // 50 cm above ground

    _dt = 0.0025f;

    _mode_switch_counter = 0;
    _manual_counter = 0;

    _vel_target_y = 0;
    _vel_target_z = 0;

    // init flags
    flags.switched_mode = false;
    flags.mode_transition_completed = false;
    flags.manual_override = false;

    //hal.uartA->begin(115200); // debug
}

void AC_Backstepping::update_alt_controller()
{
    float cphi = _ahrs.cos_roll();  // cos(phi)
    float cthe = _ahrs.cos_pitch(); // cos(theta)

    // position error
    float ez = _pos_target_z - _pos.z;

    //TODO: discard crazy values

    perr.ez = ez;   // log

    // d term
    _pos.vel_z_err = _vel_target_z -_pos.vz; //(ez - _pos.prev_ez) / _dt; // TODO: need to add target velocity

    // i term
    _pos.iez += ez * _dt;

    _pos.prev_ez = ez;

    // update d term
    float dterm_z = _pos.vel_z_err*(_gains.k2_z + _gains.k3_z);
    perr.dterm_z = dterm_z;  // log

    // restrict integral
    float iterm_z = _limit_integral(_gains.k1_z*_gains.k3_z, ez, 'z');
    perr.iterm_z = iterm_z;    // log

    float thrHover = 0.34;//_motors.get_throttle_hover();

    // restrict derivative to be hover throttle at max
    dterm_z = _limit_derivative(dterm_z, thrHover*0.5f);

    // compute u1
    _u1 = (dterm_z + ez*(_gains.k1_z + _gains.k2_z*_gains.k3_z) + iterm_z + thrHover) / (cphi*cthe);

    _thr_out = _limit_thrust(_u1);

    // mode transition throttle ramping, 0.5s
    if (!flags.mode_transition_completed)   _thr_out = _throttle_transition(_thr_out);

    //hal.uartA->printf("u1 %f, p %f, i %f, d %f, ThrH %f, z %f\n", _thr_out, ez*(_gains.k1_z + _gains.k2_z*_gains.k3_z), iterm_z, dterm_z,_motors.get_throttle_hover(),_pos.z);

    // output throttle to attitude controller -> motor
    // dont use throttle boost, irrelevant for backstepping
    _attitude_control.set_throttle_out(_thr_out, false, BACKSTEPPING_THROTTLE_CUTOFF_FREQ);
}

float AC_Backstepping::update_lateral_controller()
{
    // position error
    float ey = _pos_target_y - _pos.y;

    perr.ey = ey;   // log

    // d term
    _pos.vel_y_err = _vel_target_z -_pos.vy;

    // i term
    _pos.iey += ey * _dt;

    _pos.prev_ey = ey;

    // update d term
    float dterm_y = _pos.vel_y_err*(_gains.k2_y + _gains.k3_y);

    // restrict derivative to be hover throttle at max
    dterm_y = _limit_derivative(dterm_y, ROLL_DTERM_MAX);

    perr.dterm_y = dterm_y;  // log

    // restrict integral
    float iterm_y = _limit_integral(_gains.k1_y*_gains.k3_y, ey, 'y');
    perr.iterm_y = iterm_y;    // log

    float sin_phi = _limit_sin_phi( ((_gains.k1_y + _gains.k2_y*_gains.k3_y)*ey + iterm_y + dterm_y) / _u1 );

    _BS_roll = asin(sin_phi);

    // convert from radian to centidegrees for attitude controller
    _BS_roll = _rad2cdeg(_BS_roll);

    // check for manual override
    if (!flags.manual_override)   _target_roll = _angle_transition(_BS_roll);
    else                          _target_roll = _pilot_roll;

    //hal.uartA->printf("tar %f, p %f, i %f, iey %f, iez %f\n", _target_roll,(_gains.k1_y + _gains.k2_y*_gains.k3_y)* ey, iterm_y, _pos.iey, _pos.iez);

    return _target_roll;
}

float AC_Backstepping::_angle_transition(float target_roll)
{
    if (_manual_counter > 0)
    {
        float roll_out = (1- (float) _manual_counter/(MANUAL_OVERRIDE_TIME/_dt)) * target_roll;

        _manual_counter--;

        return roll_out;
    }
    else
    {
        return target_roll;
    }
}

void AC_Backstepping::get_target_vel(float vyd, float vzd)
{
    _vel_target_y = vyd;
    _vel_target_z = vzd;
}

void AC_Backstepping::get_pilot_lean_angle_input(float target_roll, float roll_max)
{
    // in centidegrees
    _pilot_roll = target_roll;
    _roll_max = roll_max;

    if (fabs(target_roll) > 0.1*roll_max)
    {
        flags.manual_override = true;
        _manual_counter = (int) MANUAL_OVERRIDE_TIME/_dt;
    }
    else
    {
        flags.manual_override = false;
    }
}

void AC_Backstepping::write_log()
{
    // write log to dataflash
    DataFlash_Class::instance()->Log_Write("BS", "TimeUS,KFY,KFZ,VELY,VELZ,U1,BSROLL,PIDROLL,THRH, YD, ZD, VYD, VZD",
                                           "smmmmnn--mmnn", "F000000000000", "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           (double) _pos.y,
                                           (double) _pos.z,
                                           (double) _pos.vy,
                                           (double) _pos.vz,
                                           (double) _u1,
                                           (double) _BS_roll,
                                           (double) _pid_roll,
                                           (double) _motors.get_throttle_hover(),
                                           (double) _pos_target_y,
                                           (double) _pos_target_z,
                                           (double) _vel_target_y,
                                           (double) _vel_target_z);
}

void AC_Backstepping::reset_mode_switch()
{
    flags.mode_transition_completed = false;
    flags.switched_mode = false;
    _mode_switch_counter = 0;
}

float AC_Backstepping::_throttle_transition(float BS_thr_out)
{
    float thr_out;

    if (!flags.switched_mode)   _last_mode_thr_out = _motors.get_throttle();

    flags.switched_mode = true;

    float switch_time = (float) 400.0f*THROTTLE_TRANSITION_TIME;

    // 400 counts = 1s
    if (_mode_switch_counter < switch_time)
    {
        thr_out = (switch_time - _mode_switch_counter)*_last_mode_thr_out/switch_time + _mode_switch_counter*BS_thr_out/switch_time;
        _mode_switch_counter++;
    }
    else
    {
        flags.mode_transition_completed = true;
        thr_out = BS_thr_out;
    }


    return thr_out;
}

void AC_Backstepping::pos_update(position_t pos)
{
    _pos.y  = pos.y;
    _pos.vy = pos.vy;
    _pos.z  = pos.z;
    _pos.vz = pos.vz;
}


void AC_Backstepping::get_target_pos(float yd, float zd)
{
    _pos_target_y = yd;
    _pos_target_z = zd;
}

void AC_Backstepping::get_gains(float yk1, float yk2, float yk3, float zk1, float zk2, float zk3)
{
    _gains.k1_y = yk1;
    _gains.k2_y = yk2;
    _gains.k3_y = yk3;
    _gains.k1_z = zk1;
    _gains.k2_z = zk2;
    _gains.k3_z = zk3;
}

void AC_Backstepping::reset_integral()
{
    _pos.iez = 0;
    _pos.iey = 0;
}

float AC_Backstepping::_limit_thrust(float thr)
{
    if (thr > 1)        return 1.0f;
    else if (thr < 0)   return 0;
    else                return thr;
}

float AC_Backstepping::_limit_derivative(float d_term, float threshold)
{
    if (d_term > threshold)         return threshold;
    else if (d_term < -threshold)   return -threshold;
    else                            return d_term;
}

float AC_Backstepping::_limit_sin_phi(float sp)
{
    float sp_max = sin(_roll_max*M_PI/180.0f*0.01f); // convert from centidegree to radian

    if (sp > sp_max)         return sp_max;
    else if (sp < -sp_max)   return -sp_max;
    else                     return sp;
}

float AC_Backstepping::_limit_integral(float gain, float current_err, char yz)
{
    float out = 0;

    switch(yz)
    {
        case 'y': {
            float i_term = gain * _pos.iey;

            if (i_term > _imax_y)
            {
                out = _imax_y;
                _pos.iey -= current_err*_dt;
            }
            else if (i_term < -_imax_y)
            {
                out = -_imax_y;
                _pos.iey -= current_err*_dt;
            }
            else    out = i_term;
        }break;

        case 'z': {
            float i_term = gain * _pos.iey;

            if (i_term > _imax_z)
            {
                out =  _imax_z;
                _pos.iez -= current_err*_dt;
            }
            else if (i_term < -_imax_z)
            {
                out =  -_imax_z;
                _pos.iez -= current_err*_dt;
            }
            else    out = i_term;
        }break;
    }

    return out;
}

float AC_Backstepping::_rad2cdeg(float in)
{
    return in*180/M_PI*100.0f;
}

float AC_Backstepping::get_u1()
{
    return _u1;
}


//==============================================================================
//  PID controller
//==============================================================================
float AC_Backstepping::update_PID_lateral_controller()
{
    // position error
    float ey = (_pos_target_y - _pos.y) * 100.0f;    // convert error from m to cm, so gain does not need to be a big number

    // d term
    float pid_vel_err = (_vel_target_y - _pos.vy) * 100.0f;  // cm/s

    // i term
    _pid_iey += ey * _dt;

    // update d term
    float dterm_y = pid_vel_err*_pid.kd;

    // restrict derivative to be hover throttle at max
    dterm_y = _limit_derivative(dterm_y, PID_DTERM_MAX);

    // restrict integral
    float iterm_y = _limit_PID_integral(ey, _pid.ki*_pid_iey);

    _pid_roll = _pid.kp * ey + iterm_y + dterm_y;

    // limit roll output
    if (_pid_roll > _roll_max)          _pid_roll = _roll_max;
    else if (_pid_roll < -_roll_max)    _pid_roll = -_roll_max;

    // check for manual override
    if (!flags.manual_override)   _target_roll = _angle_transition(_pid_roll);
    else                          _target_roll = _pilot_roll;

    //hal.uartA->printf("tar %f, p %f, i %f, d %f\n", _target_roll, _pid.kp*ey, iterm_y, dterm_y);

    return _target_roll;
}

float AC_Backstepping::get_PID_alt_climb_rate()
{
    float cr = _pid.alt_p * (_pos_target_z - _pos.z);   // cm/s

    if (cr > PID_MAX_CLIMB_RATE)
    {
        cr = PID_MAX_CLIMB_RATE;
    }
    else if (cr < -PID_MAX_CLIMB_RATE)
    {
        cr = -PID_MAX_CLIMB_RATE;
    }

    return cr;
}

void AC_Backstepping::reset_PID_integral()
{
    _pid_iey = 0;
}

float AC_Backstepping::_limit_PID_integral(float error, float in)
{
    if (in > PID_ITERM_MAX)
    {
        in = PID_ITERM_MAX;
        _pid_iey -= error * _dt;
    }
    else if (in < -PID_ITERM_MAX)
    {
        in = -PID_ITERM_MAX;
        _pid_iey -= error * _dt;
    }

    return in;
}

void AC_Backstepping::get_PID_gains(float alt_p, float kp, float ki, float kd)
{
    _pid.alt_p = alt_p;
    _pid.kp = kp;
    _pid.ki = ki;
    _pid.kd = kd;
}
