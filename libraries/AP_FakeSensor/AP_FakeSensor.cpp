#include "AP_FakeSensor.h"

extern const AP_HAL::HAL &hal;

AP_FakeSensor::AP_FakeSensor()
{
    data.pos.y    = 0;
    data.pos.z    = 0;
    data.status   = NotConnected;

    //hat.uartA->begin(115200);   // debug
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
    _flag_init = false;

    _uart = hal.uartE;  // using GPS2 port
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

    if (nbytes) _prev_pos = data.pos;   // storing the previous data for comparison

    _buflen = 0;

    // position update
    if (nbytes)
    {
        if (nbytes == 17)
        {
            char c = _uart->read();

            if (c == '$')
            {
                for (int i=0;i<16;i++)
                {
                    _linebuf[_buflen++] = _uart->read();
                }

                data.pos.y = (float) _byte2int(_linebuf, 1) * 0.001f;
                data.pos.z = (float) _byte2int(_linebuf, 2) * 0.001f;
                data.pos.nset = _byte2int(_linebuf, 3);

                //hal.uartA->printf("y %f, z %f, n %d\n", data.pos.y, data.pos.z, data.pos.nset);
            }
        }
        else
        {
            while (nbytes-->0)
            {
                _uart->read();//_linebuf[_brokenlen++] = _uart->read();

            }

            if (_brokenlen == 17)
            {
                //int y = _byte2int(_linebuf, 1);
                //int z = _byte2int(_linebuf, 2);
                //int nset = _byte2int(_linebuf, 3);

                //hal.uartA->printf("broken y %d, z %d, n %d\n", y, z, nset);
                _brokenlen = 0;
            }
        }

        _flag_init = true;

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
    result = _float2byte(result, data.perr.iterm_z);
    result = _float2byte(result, data.perr.dterm_z);

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

void AP_FakeSensor::get_KF_pos(position_t p)
{
    data.KF_pos = p;
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

int AP_FakeSensor::_byte2int(char* buffer, int position)
{
    int_num value;
    //return value = (int) (buffer[position*4]<<24|buffer[position*4+1]<<16|buffer[position*4+2]<<8|buffer[position*4+3]);
    value.buf[0] = buffer[position*4+0];
    value.buf[1] = buffer[position*4+1];
    value.buf[2] = buffer[position*4+2];
    value.buf[3] = buffer[position*4+3];

    return value.num;
}

float AP_FakeSensor::_byte2float(char* buffer, int position)
{
    float_num f;

    f.buf[0] = buffer[position*4+0];
    f.buf[1] = buffer[position*4+1];
    f.buf[2] = buffer[position*4+2];
    f.buf[3] = buffer[position*4+3];

    return f.num;
}

void AP_FakeSensor::write_log()
{
    // write log to dataflash
    DataFlash_Class::instance()->Log_Write("FAKE", "TimeUS,ALTD,CR,eDIST,RNGALT",
                                           "smmmm", "F0000", "Qffff",
                                           AP_HAL::micros64(),
                                           (double) data.AC_alt_target,
                                           (double) data.AC_cr,
                                           (double) data.dist_err,
                                           (double) data.target_rangefinder_alt);
}
