#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <stdio.h>
#include <cstdio>
#include <vector>
#include <ctype.h>

#define ODROID_BAUDRATE 921600

#define DATA_BUF_SIZE 21 // 21 char in a message
#define FAR_THRESHOLD 2000 // mm

using namespace std;

union float_num
{
    unsigned char buf[4];
    float num;
};

enum FakeSensor_status
{
    Good,
    FarFromOrigin,
    OutOfTunnel,
    Bad,
    NotConnected
};

struct rc_channel_t
{
    int roll  = 1500;
    int pitch = 1500;
    int yaw   = 1500;
    int thr   = 0;
    int aux5  = 0;
    int aux6  = 0;
    int aux7  = 0;
    int aux8  = 0;
    int aux9  = 0;

};

// control output
struct pos_error_t
{
    float ez;
    float dterm_z;
    float iterm_z;
};

struct FakeSensor_data_t
{
    /*
        message structure:

        $ start char
        0 or 1 : is lidar healthy
        + or - : sign of pos y
        5 digits of y position
        + or - : sign of pos z
        5 digits of z position
        + or - : sign of alt
        5 digits of altitude
        # end char
    */
    float pos_y;    // m
    float pos_z;    // m
    float alt;      // m
    int16_t alt_cm; // cm


    // Pixhawk 2 info
    float roll;
    float pitch;
    float yaw;

    rc_channel_t ch;    // rc channels

    uint32_t ts;    // in ms

    enum FakeSensor_status status;

    // controller output
    pos_error_t perr;
    float mthrust_out[3];   // motor thrust output
    float u1;               // Backstepping thrust output
};



class AP_FakeSensor
{
public:
    FakeSensor_data_t data;

    AP_FakeSensor();

    void init();
    void update();
    void get_AHRS(AP_AHRS_View* ahrs);
    void get_motors(AP_MotorsMulticopter* motors);
    bool data_is_ok();

    // get controller info
    void read_controller(pos_error_t perr, float u1);

private:
    AP_HAL::UARTDriver *_uart = nullptr;
    char _linebuf[DATA_BUF_SIZE];
    uint8_t _linebuf_len = 0;

    AP_MotorsMulticopter*   _motors;
    AP_AHRS_View*           _ahrs;

    // messenger
    void _get_pos();
    void _read_radio();
    void _read_AHRS();
    vector<unsigned char> _msg_encoder();
    void _msg_sender(vector<unsigned char>  msg);

    // helper
    float _limit_thr(float thr);    // restrict throttle from 0-1 (mainly noise issue?)
    vector<unsigned char> _int2byte(vector<unsigned char> in, int value);
    vector<unsigned char> _float2byte(vector<unsigned char> in, float value);
};
