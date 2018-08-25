#include "AC_Backstepping.h"


AC_Backstepping::AC_Backstepping(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                 const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                                 const AP_FakeSensor& fs) :
                                 _ahrs(ahrs),
                                 _inav(inav),
                                 _motors(motors),
                                 _attitude_control(attitude_control),
                                 _fs(fs)
{
    _pos_target_z = 0.5f; // 50 cm above ground
}

void AC_Backstepping::update_alt_controller()
{
    float cphi = _ahrs.cos_roll();  // cos(phi)
    float cthe = _ahrs.cos_pitch(); // cos(theta)

    float ez = _pos_target_z - _pos.z;
    float  dez = (ez - _pos.prev_ez) / _dt;
    _pos.iez += ez * _dt;

    perr.ez = ez;

    // update d term
    _pos.prev_ez = ez;
    float dterm_z = dez*(_gains.k2_z + _gains.k3_z);
    perr.dterm_z = dterm_z;

    // restrict integral
    float iterm_z = _limit_integral(_gains.k1_z*_gains.k3_z, ez, 'z');
    perr.iterm_z = iterm_z;

    // compute u1
    _u1 = (dterm_z + ez*(_gains.k1_z + _gains.k2_z*_gains.k3_z) + iterm_z) / (cphi*cthe);

    _thr_out = _limit_thrust(_u1 + _motors.get_throttle_hover());

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

void AC_Backstepping::debug_print()
{
    if (_loop_counter >= 100)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "iez %f, u1 %f, thrO %f, ez %f\n", _pos.iez, _u1, _thr_out, perr.ez);
        _loop_counter = 0;
    }
    _loop_counter++;
}

void AC_Backstepping::pos_update()
{
    _pos.y = _fs.data.pos_y;
    _pos.z = _fs.data.alt;    // TEMP: change back to pos z later
}


void AC_Backstepping::get_target_pos(float yd, float zd)
{
    _pos_target_y = yd;
    _pos_target_z = zd;
}

void AC_Backstepping::set_dt(float delta_sec)
{
    _dt = delta_sec;
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

void AC_Backstepping::get_imax(float imax_y, float imax_z)
{
    _imax_y = imax_y;
    _imax_z = imax_z;
}

float AC_Backstepping::get_u1()
{
    return _u1;
}
