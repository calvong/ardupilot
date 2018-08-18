#include "AP_FakeSensor.h"

extern const AP_HAL::HAL &hal;

AP_FakeSensor::AP_FakeSensor()
{
    _data.pos_y    = 0;
    _data.pos_z    = 0;
    _data.status   = NotConnected;
}

void AP_FakeSensor::get_AHRS(AP_AHRS_View* ahrs)
{
    _ahrs = ahrs;
}

void AP_FakeSensor::init()
{
    _uart = hal.uartC;  // using telem1 port
    if (_uart == nullptr)    {return;}

    _uart->begin(ODROID_BAUDRATE);

    gcs().send_text(MAV_SEVERITY_INFO, "Initialising fake sensor communication with Odroid...\n");
}

void AP_FakeSensor::update()
{
    if (_uart == nullptr)    {return;}


    int16_t nbytes = _uart->available();

    char d[6];      // temp buffer for 6 digits

// position update
    while (nbytes-- > 0)
    {
        char c = _uart->read();

        // decode the message
        if (c == '#')   // end bit: start processing data
        {
            // pos y
            for (size_t i = 0; i < 6; i++)
            {
                d[i] = _linebuf[i+2];
            }
            _data.pos_y = atoi(d);

            // pos z
            for (size_t i = 0; i < 6; i++)
            {
                d[i] = _linebuf[i+8];
            }
            _data.pos_z = atoi(d);

            // pos z
            for (size_t i = 0; i < 6; i++)
            {
                d[i] = _linebuf[i+14];
            }
            _data.alt = atoi(d);

            // assign status
            if (_linebuf[1] == '1')
                _data.status = Good;
            else
                _data.status = Bad;

            _linebuf_len = 0; // reset
        }
        else
        {
            _linebuf[_linebuf_len++] = c;

            if (_linebuf_len == sizeof(_linebuf)) {_linebuf_len = 0;}
        }
    }

// attitude update
    _data.roll = _ahrs->roll;
    _data.pitch = _ahrs->pitch;
    _data.yaw = _ahrs->yaw;

// assign timestamp to data
    _data.ts = AP_HAL::millis();

    vector<unsigned char> msg;
    msg = msg_encoder();
    msg_sender(msg);

    //gcs().send_text(MAV_SEVERITY_INFO, "%d %d alt %d",_data.pos_y, _data.pos_z, _data.alt);
}


vector<unsigned char> AP_FakeSensor::msg_encoder()
{
//=================================
//********  MSG FORMAT  ***********
//=================================
//  variables:
//  roll, pitch, yaw are x1000 and sent as int
//
//  message:
//  $roll pitch yaw ts#

    vector<unsigned char> result;

    int roll        = static_cast<int>(_data.roll*1000);
    int pitch       = static_cast<int>(_data.pitch*1000);
    int yaw         = static_cast<int>(_data.yaw*1000);
    unsigned int ts = _data.ts;

    result.push_back('$');
    result = _int2byte(result, roll);
    result = _int2byte(result, pitch);
    result = _int2byte(result, yaw);
    result = _int2byte(result, ts);

    return result;
}

vector<unsigned char> AP_FakeSensor::_int2byte(vector<unsigned char> in, int value)
{
    in.push_back(value >> 24);
    in.push_back(value >> 16);
    in.push_back(value >>  8);
    in.push_back(value      );
    return in;
}


void AP_FakeSensor::msg_sender(vector<unsigned char> msg)
{
    _uart->set_blocking_writes(false);
    for (int i=0; i < msg.size(); i++)
    {
        _uart->write(msg[i]);
    }
    _uart->set_blocking_writes(true);
}

FakeSensor_data AP_FakeSensor::get_data()
{
    return _data;
}
