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

#define DATA_BUF_SIZE 13 // 13 char in a message
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

struct FakeSensor_data
{
/*
    message structure:

    $ start char
    4 digits of y position
    + or - : sign of pos y
    4 digits of z position
    + or - : sign of pos z
    0 or 1 in tunnel or not
    # end char
*/
    int16_t pos_y;
    int16_t pos_z;
    int16_t alt;

    float roll;
    float pitch;
    float yaw;

    uint32_t ts;    // in ms

    enum FakeSensor_status status;
};



class AP_FakeSensor
{
public:
    AP_FakeSensor();

    void init();
    void update();
    void get_AHRS(AP_AHRS_View* ahrs);
    FakeSensor_data get_data();


    vector<unsigned char> msg_encoder();
    void msg_sender(vector<unsigned char>  msg);

private:
    AP_HAL::UARTDriver *_uart = nullptr;
    FakeSensor_data _data;
    char _linebuf[DATA_BUF_SIZE];
    uint8_t _linebuf_len = 0;

    AP_AHRS_View*    _ahrs;

    // messenger
    vector<unsigned char> _int2byte(vector<unsigned char> in, int value);
};
