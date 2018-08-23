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
    /*
    _gains.k1_y = 1;
    _gains.k2_y = 1;
    _gains.k3_y = 1;
    _gains.k1_z = 1;
    _gains.k2_z = 1;
    _gains.k3_z = 1;

    _imax_y = DEFAULT_IMAX;
    _imax_z = DEFAULT_IMAX;
    */
}

void AC_Backstepping::update_alt_controller()
{
    float cphi = _ahrs.cos_roll();  // cos(phi)
    float cthe = _ahrs.cos_pitch(); // cos(theta)

    float ez = _pos_target_z - _pos.z;
    float  dez = (ez - _pos.prev_ez) / _dt;
    _pos.iez += ez * _dt;

    // update d term
    _pos.prev_ez = ez;
    dez = 0;   // TEMP

    // restrict integral
    float integral = _limit_integral(_gains.k2_z*_gains.k3_z*_pos.iez, 'z');
    integral = 0; // TEMP


    // compute u1
    _u1 = (G + dez*(_gains.k1_z + _gains.k3_z) + ez*(_gains.k2_z + _gains.k1_z*_gains.k3_z) + integral) / (cphi*cthe);

    // output throttle to attitude controller -> motor
    // dont use throttle boost, irrelevant for backstepping
    //_attitude_control.set_throttle_out(_thr_out, false, BACKSTEPPING_THROTTLE_CUTOFF_FREQ);
}

void AC_Backstepping::update_lateral_controller()
{

}

void AC_Backstepping::write_log()
{

}

void AC_Backstepping::pos_update()
{
    _pos.y = _fs.data.pos_y;
    _pos.z = _fs.data.pos_z;
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


void AC_Backstepping::reset_integral_y()
{
    _pos.iey = 0;
}

void AC_Backstepping::reset_integral_z()
{
    _pos.iez = 0;
}

float AC_Backstepping::_limit_integral(float i_term, char yz)
{
    float out = 0;

    switch(yz)
    {
        case 'y':
            if (i_term > _imax_y)       out = _imax_y;
            else if (i_term < -_imax_y) out = -_imax_y;
            break;

        case 'z':
            if (i_term > _imax_z)       out =  _imax_z;
            else if (i_term < -_imax_z) out =  -_imax_z;
            break;
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
