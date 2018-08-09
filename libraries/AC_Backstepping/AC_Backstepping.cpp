#include "AC_Backstepping.h"



AC_Backstepping::AC_Backstepping(const AP_AHRS_View& ahrs, //const AP_InertialNav& inav,
                                 const AP_Motors& motors,
                                 AC_AttitudeControl& attitude_control)
{

}

void AC_Backstepping::get_pos(int16_t pos_y, int16_t pos_z)
{
    _pos_y = pos_y;
    _pos_z = pos_z;
}


void AC_Backstepping::update_alt_controller()
{
    int current_pos_z = _pos_z;
    float current_vel_z = 1;

    // dummy
    current_pos_z += 1;
    current_vel_z += 2;
}

void AC_Backstepping::update_lateral_controller()
{

}
