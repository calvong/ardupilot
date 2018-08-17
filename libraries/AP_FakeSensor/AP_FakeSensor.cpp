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
    uart = hal.uartC;  // using telem1 port
    if (uart == nullptr)    {return;}

    uart->begin(ODROID_BAUDRATE);

    gcs().send_text(MAV_SEVERITY_INFO, "Initialising fake sensor communication with Odroid...\n");
}

void AP_FakeSensor::update()
{
    if (uart == nullptr)    {return;}


    int16_t nbytes = uart->available();

    char d[4];      // temp buffer for 4 digits


// position update
    while (nbytes-- > 0)
    {
        char c = uart->read();

        // decode the message
        if (c == '#')   // end bit: start processing data
        {
            // pos y
            for (size_t i = 0; i < 4; i++)
            {
                d[i] = _linebuf[i+1];
            }
            _data.pos_y = atoi(d);

            if (_linebuf[5] == '-')  _data.pos_y *= -1;

            // pos z
            for (size_t i = 0; i < 4; i++)
            {
                d[i] = _linebuf[i+6];
            }
            _data.pos_z = atoi(d);

            if (_linebuf[10] == '-')  _data.pos_z *= -1;

            // assign status
            int inT = (int) _linebuf[11] - 48;   // in Tunnel?

            if (inT && _linebuf[0] == '$')
                _data.status = Good;
            else if (inT && _linebuf[0] != '$')    // not a complete msg
                _data.status = Bad;
            else if (abs(_data.pos_y) > FAR_THRESHOLD || abs(_data.pos_z) > FAR_THRESHOLD)
                _data.status = FarFromOrigin;
            else if (!inT)
                _data.status = OutOfTunnel;

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

    gcs().send_text(MAV_SEVERITY_INFO, "roll: %f  %f %ld\n", _ahrs->roll*180/M_PI, _data.pitch*180/M_PI, _data.ts);

    // signal to get another reading
    uart->set_blocking_writes(false);
    uart->write("abcd2345423");
    uart->set_blocking_writes(true);

}

FakeSensor_data AP_FakeSensor::get_data()
{
    return _data;
}
