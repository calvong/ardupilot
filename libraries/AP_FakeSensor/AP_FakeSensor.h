#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>


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

    enum FakeSensor_status status;
};

class AP_FakeSensor
{
public:
    AP_FakeSensor();

    void init();
    void update();
    FakeSensor_data get_data();

private:
    AP_HAL::UARTDriver *uart = nullptr;
    FakeSensor_data _data;
    char linebuf[DATA_BUF_SIZE];
    uint8_t linebuf_len = 0;
};
