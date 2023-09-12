#include "loopo.hpp"

using namespace motor;
using namespace encoder;

bool update_callback(repeating_timer_t *rt);
bool command_callback(repeating_timer_t *rt);

void respond();

// void read_command();

// Create a motor and set its direction and speed scale
Motor lp_mot = Motor(LP_MOTOR_PINS, DIRECTION_MOTOR, SPEED_SCALE);

// Create an encoder and set its direction and counts per rev, using PIO 0 and State Machine 0
Encoder lp_enc = Encoder(pio0, 3, LP_ENCODER_PINS, PIN_UNUSED, DIRECTION_ENCODER, COUNTS_PER_REV, true);

// Create the user button
Button user_sw(motor2040::USER_SW);
Button lp_ronout(LP_RUNOUT_PIN);

Analog force = Analog(FORCE_PIN);

// Create PID object for position control

PID lp_pos_pid = PID(POS_KP, POS_KI, POS_KD, UPDATE_RATE);
PID lp_vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);
PID lp_frc_pid = PID(FRC_KP, FRC_KI, FRC_KD, UPDATE_RATE);

loop lp = loop(&lp_mot, &lp_enc, &lp_ronout, &force, &lp_pos_pid, &lp_vel_pid, &lp_frc_pid);

repeating_timer_t update_timer;
repeating_timer_t command_timer;

uint8_t serial_buffer[BUFFER_LENGTH];

bool cmd_flag = 0;

int main()
{
    stdio_init_all();

    add_repeating_timer_ms(UPDATE_RATE * 1000.0f, update_callback, NULL, &update_timer);
    add_repeating_timer_ms(LOG_RATE * 1000.0f, command_callback, NULL, &command_timer);

    while (!user_sw.raw())
    {
        // read_command();
    }
}

bool update_callback(repeating_timer_t *rt)
{
    lp.update();

    if (cmd_flag)
    {
        respond();
        cmd_flag = 0;
    }

    return 1;
}

void respond()
{
    int32_t lp_pos = lp.get_position();
    int lp_sts = lp.get_status();
    int lp_cnt = lp.get_control();
    float frc = lp.get_force();

#ifdef DEBUGGING
    printf("Extension\tpos: %d - sts: %d - cnt: %d\n", ex_pos, ex_sts, ex_cnt);
    printf("Twist\t\tpos: %d - off: %d -sts: %d - cnt: %d\n", tw_pos, tw_off, tw_sts, tw_cnt);
    printf("Loop\t\tpos: %d - frc: %f -sts: %d - cnt: %d\n\n", lp_pos, frc, lp_sts, lp_cnt);
#else
    printf("%d:%f:%d:%d\n", lp_pos, frc, lp_sts, lp_cnt);
#endif
}

bool command_callback(repeating_timer_t *rt)
// void read_command()
{
    uint16_t end = read_line(serial_buffer);
    if (end)
    {

        cmd_flag = 1;

        actuator_command message = interpret_buffer(serial_buffer, end);

#if DEBUGGING
        printf("ID: %d\tCOMMAND: %d\tVALUE: %f\n", message.id, message.command, message.value);
#endif

        switch (message.id)
        {
        case 3:
            lp.execute_command(message.command, message.value);
        default:
            break;
        }
    }
    return 1;
}
