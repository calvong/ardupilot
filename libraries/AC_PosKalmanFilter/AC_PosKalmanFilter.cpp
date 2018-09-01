#include "AC_PosKalmanFilter.h"

extern const AP_HAL::HAL &hal;

AC_PosKalmanFilter::AC_PosKalmanFilter(const AP_AHRS_View& ahrs,
                                       const AP_InertialNav& inav,
                                       const AP_FakeSensor& fs) :
                                       _ahrs(ahrs),
                                       _inav(inav),
                                       _fs(fs)
{
    hal.uartA->begin(115200); // debug

    _init();
}

void AC_PosKalmanFilter::_init()
{
    _Xp.reserve(4);
    _Xe.reserve(4);
    _Pp.reserve(8);
    _Pe.reserve(8);
    _K.reserve(4);

    for (int i=0;i<4;i++)
    {
        _Xp.push_back(0);
        _Xe.push_back(0);
    }

    _Pp.push_back(1);
    _Pp.push_back(0);
    _Pp.push_back(0);
    _Pp.push_back(1);
    _Pp.push_back(1);
    _Pp.push_back(0);
    _Pp.push_back(0);
    _Pp.push_back(1);

    _Pe =_Pp;
}

void AC_PosKalmanFilter::run()
{
    // get accelerations
    const Vector3f &accel = _ahrs.get_accel_ef_blended();
    float ay = accel.y; // TODO: need to check sign
    float az = -(accel.z + G_KF);

    // prediction - 400Hz
     _Xp = _X_predict(_Xe, ay, az);

     _Pp = _P_predict(_Pe);

     _Xe = _Xp;

    //  correction - 30Hz / as soon as new lidar data arrives
    if (_prev_data_set != _fs.data.pos.nset)
    {
        _K = _Kk(_Pp);

        _Xe = _X_estimate(_K, _Xp, _fs.data.pos);
        _Pe  = _P_estimate(_Pp, _K);
    }

    _pos.y  = _Xe[0];
    _pos.vy = _Xe[1];
    _pos.z  = _Xe[2];
    _pos.vz = _Xe[3];

    _prev_data_set = _fs.data.pos.nset;

    print_shit();
}

vector<float> AC_PosKalmanFilter::_X_predict(vector<float> Xe, float ay, float az)
{
    vector<float> xp;
    xp.reserve(4);

    // y
    xp.push_back( Xe[0] + Xe[1]*dt_KF + 0.5f*ay*dt_KF2 );
    xp.push_back( Xe[1] + ay*dt_KF                     );

    // z
    xp.push_back( Xe[2] + Xe[3]*dt_KF + 0.5f*az*dt_KF2 );
    xp.push_back( Xe[3] + az*dt_KF                     );

    return xp;
}

vector<float> AC_PosKalmanFilter::_X_estimate(vector<float> K, vector<float> Xp, position_t pos)
{
    vector<float> xe;
    xe.reserve(4);

    // y
    xe.push_back( Xp[0] + K[0]*(pos.y - Xp[0]) );
    xe.push_back( Xp[1] + K[1]*(pos.y - Xp[0]) );

    // z
    xe.push_back( Xp[2] + K[2]*(pos.z - Xp[2]) );
    xe.push_back( Xp[3] + K[3]*(pos.z - Xp[2]) );

    return xe;
}

vector<float> AC_PosKalmanFilter::_P_predict(vector<float> P)
{
    vector<float> pp;
    pp.reserve(8);

    // y
    pp.push_back( P[0] + Q + dt_KF*P[2] + dt_KF*(P[1] + dt_KF*P[3]) );
    pp.push_back( P[1] + dt_KF * P[3]                               );
    pp.push_back( P[2] + dt_KF * P[3]                               );
    pp.push_back( P[3] + Q                                          );

    // z
    pp.push_back( P[4] + Q + dt_KF*P[6] + dt_KF*(P[5] + dt_KF*P[7]) );
    pp.push_back( P[5] + dt_KF * P[7]                               );
    pp.push_back( P[6] + dt_KF * P[7]                               );
    pp.push_back( P[7] + Q                                          );

    return pp;
}

vector<float> AC_PosKalmanFilter::_P_estimate(vector<float> P, vector<float> K)
{
    vector<float> pe;
    pe.reserve(8);

    // y
    pe.push_back( -(K[0] - 1) * (P[0] + Q + dt_KF * P[2] + dt_KF * (P[1] + dt_KF * P[3]))                   );
    pe.push_back( -(P[1] + dt_KF * P[3]) * (K[0] - 1)                                                       );
    pe.push_back( P[2] - K[1] * (P[0] + Q + dt_KF * P[2] + dt_KF * (P[1] + dt_KF * P[3])) + dt_KF * P[3]    );
    pe.push_back( P[3] + Q - K[1] * (P[1] + dt_KF * P[3])                                                   );

    // z
    pe.push_back( -(K[2] - 1) * (P[4] + Q + dt_KF * P[6] + dt_KF * (P[5] + dt_KF * P[7]))                   );
    pe.push_back( -(P[5] + dt_KF * P[7]) * (K[2] - 1)                                                       );
    pe.push_back( P[6] - K[3] * (P[4] + Q + dt_KF * P[6] + dt_KF * (P[5] + dt_KF * P[7])) + dt_KF * P[7]    );
    pe.push_back( P[7] + Q - K[3] * (P[5] + dt_KF * P[7])                                                   );

    return pe;
}

vector<float> AC_PosKalmanFilter::_Kk(vector<float> P)
{
    vector<float> k;
    k.reserve(4);

    // y
    k.push_back( P[0] / (R + P[0]) );
    k.push_back( P[2] / (R + P[0]) );

    // z
    k.push_back( P[4] / (R + P[4]) );
    k.push_back( P[6] / (R + P[4]) );

    return k;
}

position_t AC_PosKalmanFilter::get_pos()
{
    return _pos;
}

void AC_PosKalmanFilter::print_shit()
{
    hal.uartA->printf("alt %f, pos_z %f\n", _fs.data.pos.alt, _pos.z);
}

void AC_PosKalmanFilter::write_log()
{
    // write log to dataflash
    const Vector3f &accel = _ahrs.get_accel_ef_blended();
    const Vector3f &velocity = _inav.get_velocity();

    // accel z
    double accel_z = accel.z/(_ahrs.cos_roll()*_ahrs.cos_pitch());
    double accel_z_fil = _pos_z_filter.apply(accel_z, 0.0025f);

    DataFlash_Class::instance()->Log_Write("PKF", "TimeUS,ALT,VELZ,ACCZ,ACCZF,NSET,POSZ",
                                           "smnoo-m", "F000000", "Qffffif",
                                           AP_HAL::micros64(),
                                           (double) _fs.data.pos.alt,
                                           (double) velocity.z,
                                           (double) accel_z,
                                           (double) accel_z_fil,
                                           _fs.data.pos.nset,
                                            _pos.z);

}
