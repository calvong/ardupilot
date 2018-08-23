#include "AP_FakeSensor.h"

extern const AP_HAL::HAL &hal;

AP_FakeSensor::AP_FakeSensor()
{
    data.pos_y    = 0;
    data.pos_z    = 0;
    data.status   = NotConnected;
}

void AP_FakeSensor::get_AHRS(AP_AHRS_View* ahrs)
{
    _ahrs = ahrs;
}

void AP_FakeSensor::get_motors(AP_MotorsMulticopter* motors)
{
    _motors = motors;
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
    // get position data from Odroid
    _get_pos();

    // update radio values
    _read_radio();

    // update AHRS
    _read_AHRS();

    // get motor thrust outputs
    _motors->get_motor_thrust_output(data.mthrust_out);

    // assign timestamp to data
    data.ts = AP_HAL::millis();

    // send Pixhawk 2 data back to Oddroid
    vector<unsigned char> msg;
    msg = _msg_encoder();
    _msg_sender(msg);

    //gcs().send_text(MAV_SEVERITY_INFO, "RC %d ", data.ch.roll);
}

void AP_FakeSensor::read_controller(pos_error_t perr, float u1)
{
    data.perr = perr;
    data.u1 = u1;
}

void AP_FakeSensor::_read_AHRS()
{
    data.roll = _ahrs->roll;
    data.pitch = _ahrs->pitch;
    data.yaw = _ahrs->yaw;
}

void AP_FakeSensor::_get_pos()
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
            data.pos_y = (float) atoi(d)*0.001f;   // m

            // pos z
            for (size_t i = 0; i < 6; i++)
            {
                d[i] = _linebuf[i+8];
            }
            data.pos_z = (float) atoi(d)*0.001f;   // m

            // alt
            for (size_t i = 0; i < 6; i++)
            {
                d[i] = _linebuf[i+14];
            }
            data.alt = (float) atoi(d) * 0.001f;  // m
            data.alt_cm = (int16_t) data.alt * 0.1f; // cm

            // assign status
            if (_linebuf[1] == '1')
                data.status = Good;
            else
                data.status = Bad;

            _linebuf_len = 0; // reset
        }
        else
        {
            _linebuf[_linebuf_len++] = c;

            if (_linebuf_len == sizeof(_linebuf)) {_linebuf_len = 0;}
        }
    }
}

vector<unsigned char> AP_FakeSensor::_msg_encoder()
{
    //  message starts with '$'
    //  roll, pitch, yaw are x1000 and sent as int

    vector<unsigned char> result;

    result.push_back('$');
    result = _int2byte(result, data.ts);
    result = _float2byte(result, data.roll);
    result = _float2byte(result, data.pitch);
    result = _float2byte(result, data.yaw);
    result = _int2byte(result, data.ch.roll);
    result = _int2byte(result, data.ch.pitch);
    result = _int2byte(result, data.ch.thr);
    result = _int2byte(result, data.ch.yaw);
    result = _int2byte(result, data.ch.aux5);
    result = _int2byte(result, data.ch.aux6);
    result = _int2byte(result, data.ch.aux7);
    result = _int2byte(result, data.ch.aux8);
    result = _float2byte(result, data.u1);
    result = _float2byte(result, _limit_thr(data.mthrust_out[0]));  // throttle in
    result = _float2byte(result, _limit_thr(data.mthrust_out[1]));  // throttle avg max
    result = _float2byte(result, _limit_thr(data.mthrust_out[2]));  // throttle hover
    result = _float2byte(result, data.perr.ez);
    result = _float2byte(result, data.perr.dterm_z);
    result = _float2byte(result, data.perr.iterm_z);
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

vector<unsigned char> AP_FakeSensor::_float2byte(vector<unsigned char> in, float value)
{
    float_num f;
    f.num = value;  // assign float value to the buffer

    in.push_back(f.buf[0]);
    in.push_back(f.buf[1]);
    in.push_back(f.buf[2]);
    in.push_back(f.buf[3]);
    return in;
}

void AP_FakeSensor::_msg_sender(vector<unsigned char> msg)
{
    _uart->set_blocking_writes(false);
    for (int i=0; i < msg.size(); i++)
    {
        _uart->write(msg[i]);
    }
    _uart->set_blocking_writes(true);
}

bool AP_FakeSensor::data_is_ok()
{
    if (data.status == Bad)    return false;
    else                        return true;
}

float AP_FakeSensor::_limit_thr(float thr)
{
    if (thr > 1) return 1.0f;
    else if (thr < 0) return 0;
    else return thr;
}

void AP_FakeSensor::_read_radio()
{
    data.ch.roll    = rc().channel(CH_1)->get_radio_in();
    data.ch.pitch   = rc().channel(CH_2)->get_radio_in();
    data.ch.thr     = rc().channel(CH_3)->get_radio_in();
    data.ch.yaw     = rc().channel(CH_4)->get_radio_in();
    data.ch.aux5    = rc().channel(CH_5)->get_radio_in();
    data.ch.aux6    = rc().channel(CH_6)->get_radio_in();
    data.ch.aux7    = rc().channel(CH_7)->get_radio_in();
    data.ch.aux8    = rc().channel(CH_8)->get_radio_in();
    data.ch.aux9    = rc().channel(CH_9)->get_radio_in();
}
