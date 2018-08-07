#include "AP_FakeSensor.h"

extern const AP_HAL::HAL &hal;

AP_FakeSensor::AP_FakeSensor()
{
    _data.pos_y    = 0;
    _data.pos_z    = 0;
    _data.status   = NotConnected;
}

void AP_FakeSensor::init()
{
    uart = hal.uartE;  // using 2nd GPS port
    if (uart == nullptr)    {return;}

    uart->begin(ODROID_BAUDRATE);

    hal.console->printf("Init fake sensor ...\n");
    //TODO: need to do init handshake with odroid
}

void AP_FakeSensor::update()
{
    if (uart == nullptr)    {return;}

    int16_t nbytes = uart->available();

    char d[4];      // temp buffer for 4 digits

    while (nbytes-- > 0)
    {
        char c = uart->read();

        // decode the message
        if (c == '#')   // end bit: start processing data
        {
            // pos y
            for (size_t i = 0; i < 4; i++)
            {
                d[i] = linebuf[i+1];
            }
            _data.pos_y = atoi(d);

            if (linebuf[5] == '-')  _data.pos_y *= -1;

            // pos z
            for (size_t i = 0; i < 4; i++)
            {
                d[i] = linebuf[i+6];
            }
            _data.pos_z = atoi(d);

            if (linebuf[10] == '-')  _data.pos_z *= -1;

            // assign status
            int inT = (int) linebuf[11] - 48;   // in Tunnel?

            if (inT && linebuf[0] == '$')
                _data.status = Good;
            else if (inT && linebuf[0] != '$')    // not a complete msg
                _data.status = Bad;
            else if (abs(_data.pos_y) > FAR_THRESHOLD || abs(_data.pos_z) > FAR_THRESHOLD)
                _data.status = FarFromOrigin;
            else if (!inT)
                _data.status = OutOfTunnel;

            linebuf_len = 0; // reset
        }
        else
        {
            linebuf[linebuf_len++] = c;

            if (linebuf_len == sizeof(linebuf)) {linebuf_len = 0;}
        }
    }

    // signal to get another reading
    uart->write('g');
}

FakeSensor_data AP_FakeSensor::get_data()
{
    return _data;
}
