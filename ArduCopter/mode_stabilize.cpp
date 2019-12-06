#include "Copter.h"

#define m1 1200
#define m2 1500
#define m3 1500
#define m4 1200

int16_t input;
uint8_t flag_start;

// stabilize_init - initialise stabilize controller
bool Copter::ModeStabilize::init(bool ignore_checks)
{
    input = -1;
    flag_start = 0;
    motors->armed(true);
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeStabilize::run()
{

/*
    if (hal.uartA->available())
    {
        input = hal.uartA->read();

        if (input == 'r')
        {
            motors->armed(true);
            flag_start = 1;
        }

    }
*/
    flag_start = 1;

    motors->armed(true);

    if (flag_start)
    {
        motors->output_test_num(1, m1);
        //motors->output_test_seq(2, m2);
        //motors->output_test_seq(3, m3);
        motors->output_test_num(4, m4);
    }

}
