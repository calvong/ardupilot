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
    _pos_target_z = 0.5f; // 50 cm above ground
    _dt = 0.0025f;
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
    //if (fabs(ez - _pos.prev_ez) > POS_ERROR_THRESHOLD)
    //{
    //    ez = _pos.prev_ez;
    //    hal.uartA->printf("WTF? ");
    //}


    perr.ez = ez;   // log

    // d term with LPF
    //_pos.vel_z_err = _vel_error_filter.apply((ez - _pos.prev_ez) / _dt, _dt);
    _pos.vel_z_err = -_pos.vz; //(ez - _pos.prev_ez) / _dt; // TODO: need to add target velocity

    // i term
    _pos.iez += ez * _dt;

    // update previous data
    _prev_nset = _fs.data.pos.nset;


    _pos.prev_ez = ez;

    // update d term
    float dterm_z = _pos.vel_z_err*(_gains.k2_z + _gains.k3_z);
    perr.dterm_z = dterm_z;  // log

    // restrict integral
    float iterm_z = _gains.k1_z*_gains.k3_z*_pos.iez; //_limit_integral(_gains.k1_z*_gains.k3_z, ez, 'z');
    perr.iterm_z = iterm_z;    // log

    float thrHover = _motors.get_throttle_hover();

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

void AC_Backstepping::update_lateral_controller()
{

}

void AC_Backstepping::write_log()
{
    // write log to dataflash
    DataFlash_Class::instance()->Log_Write("BS", "TimeUS,Y,Z,KFY,KFZ,VELY,VELZ,U1,THRH,EZ,I,D,K1,K2,K3",
                                           "smmmmnn--------", "F00000000000000", "Qffffffffffffff",
                                           AP_HAL::micros64(),
                                           (double) _fs.data.pos.y,
                                           (double) _fs.data.pos.z,
                                           (double) _pos.y,
                                           (double) _pos.z,
                                           (double) _pos.vy,
                                           (double) _pos.vz,
                                           (double) _u1,
                                           (double) _motors.get_throttle_hover(),
                                           (double) perr.ez*(_gains.k1_z + _gains.k2_z*_gains.k3_z),
                                           (double) perr.iterm_z,
                                           (double) _pos.vel_z_err*(_gains.k2_z + _gains.k3_z),
                                           (double) _gains.k1_z,
                                           (double) _gains.k2_z,
                                           (double) _gains.k3_z);
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

void AC_Backstepping::debug_print()
{
    if (_loop_counter >= 100)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "vel %f, u1 %f, thrO %f, ez %f\n", _inav.get_velocity_z(), _u1, _thr_out, perr.ez);
        _loop_counter = 0;
    }
    _loop_counter++;
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
