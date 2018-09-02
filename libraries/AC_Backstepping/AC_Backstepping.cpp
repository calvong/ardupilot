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
    _imax_z = _motors.get_throttle_hover() * 0.2f;  // TEMP
    _pos_target_z = 0.5f; // 50 cm above ground
    _dt = 0.025f;
    _mode_switch_counter = 0;
    _vel_error_filter.set_cutoff_frequency(BACKSTEPPING_VEL_ERROR_CUTOFF_FREQ);

    // init flags
    flags.switched_mode = false;
    flags.mode_transition_completed = false;
    //hal.uartA->begin(115200); // debug
}

void AC_Backstepping::update_alt_controller()
{
    float cphi = _ahrs.cos_roll();  // cos(phi)
    float cthe = _ahrs.cos_pitch(); // cos(theta)

    // position error
    float ez = _pos_target_z - _pos.z;

    // discard crazy values
    if (fabs(ez - _pos.prev_ez) > POS_ERROR_THRESHOLD)   ez = _pos.prev_ez;

    perr.ez = ez;   // log

    // check if new data set is received
    if (_prev_nset != _fs.data.pos.nset)
    {
        // d term with LPF
        _pos.vel_z_err = _vel_error_filter.apply((ez - _pos.prev_ez) / _dt, _dt);
        perr.dtermfil_z = _pos.vel_z_err;

        // i term
        _pos.iez += ez * _dt;

        // update previous data
        _prev_nset = _fs.data.pos.nset;
    }

    _pos.prev_ez = ez;

    // update d term
    float dterm_z = _pos.vel_z_err*(_gains.k2_z + _gains.k3_z);
    perr.dterm_z = _pos.vel_z_err;  // log

    // restrict integral
    float iterm_z = _limit_integral(_gains.k1_z*_gains.k3_z, ez, 'z');
    perr.iterm_z = _pos.iez;    // log

    // restrict derivative to be hover throttle at max
    dterm_z = _limit_derivative(dterm_z, _motors.get_throttle_hover()*0.3f);

    // compute u1
    _u1 = (dterm_z + ez*(_gains.k1_z + _gains.k2_z*_gains.k3_z) + iterm_z + _motors.get_throttle_hover()) / (cphi*cthe);

    _thr_out = _limit_thrust(_u1);

    // mode transition throttle ramping, 0.5s
    if (!flags.mode_transition_completed)   _thr_out = _throttle_transition(_thr_out);

    // output throttle to attitude controller -> motor
    // dont use throttle boost, irrelevant for backstepping
    _attitude_control.set_throttle_out(_thr_out, false, BACKSTEPPING_THROTTLE_CUTOFF_FREQ);
}

void AC_Backstepping::update_lateral_controller()
{

}

void AC_Backstepping::write_log()
{

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

    if (_mode_switch_counter < 200)
    {
        thr_out = (200.0f - _mode_switch_counter)*_last_mode_thr_out/200.0f + _mode_switch_counter*BS_thr_out/200.0f;
        _mode_switch_counter++;
    }
    else
    {
        flags.mode_transition_completed = true;
        thr_out = BS_thr_out;
    }

    return thr_out;
}

void AC_Backstepping::debug_print()
{
    if (_loop_counter >= 100)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "vel %f, u1 %f, thrO %f, ez %f\n", _inav.get_velocity_z(), _u1, _thr_out, perr.ez);
        _loop_counter = 0;
    }
    _loop_counter++;
}

void AC_Backstepping::pos_update()
{
    _pos.y = _fs.data.pos.y;
    _pos.z = _fs.data.pos.z;   
}


void AC_Backstepping::get_target_pos(float yd, float zd)
{
    _pos_target_y = yd;
    _pos_target_z = zd;
}

void AC_Backstepping::set_dt(float delta_sec)
{
    _dt = 0.025f;//delta_sec;
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

float AC_Backstepping::get_u1()
{
    return _u1;
}
