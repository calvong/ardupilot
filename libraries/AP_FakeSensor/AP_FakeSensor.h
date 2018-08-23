#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <stdio.h>
#include <cstdio>
#include <vector>

#define ODROID_BAUDRATE 921600

#define DATA_BUF_SIZE 21 // 21 char in a message
#define FAR_THRESHOLD 2000 // mm

using namespace std;

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

    float roll;
    float pitch;
    float yaw;

    rc_channel_t ch;    // rc channels

    uint32_t ts;    // in ms

    enum FakeSensor_status status;

    // controller output
    float u1;   // thrust output

    // TEMP: temp debug variables
    int16_t my_cr;  // alt climb rate
    float my_alt_tar;   // my alt tar

    int16_t ac_cr;  // arducopter alt hold climb rate
    float ac_alt_tar; // arducopter's
};



class AP_FakeSensor
{
public:
    FakeSensor_data_t data;

    AP_FakeSensor();

    void init();
    void update();
    void get_AHRS(AP_AHRS_View* ahrs);
    bool data_is_ok();

    vector<unsigned char> msg_encoder();
    void msg_sender(vector<unsigned char>  msg);

    // get controller info


private:
    AP_HAL::UARTDriver *_uart = nullptr;
    char _linebuf[DATA_BUF_SIZE];
    uint8_t _linebuf_len = 0;

    AP_AHRS_View*    _ahrs;

    // messenger
    vector<unsigned char> _int2byte(vector<unsigned char> in, int value);
};
