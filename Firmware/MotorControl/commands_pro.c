/*

 */

#include "commands_pro.h"
#include "packet.h"
#include "buffer.h"
#include "terminal.h"
#include "commands.h"
#include "low_level.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


// Private variables
static uint8_t send_buffer[PACKET_MAX_PL_LEN];

static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*send_func_last)(unsigned char *data, unsigned int len) = 0;
static void(*appdata_func)(unsigned char *data, unsigned int len) = 0;

static Motor_t *motor;
static float pos_setpoint, vel_feed_forward, current_feed_forward;

/**
 * Provide a function to use the next time there are packets to be sent.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
    send_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
    if (send_func) {
        send_func(data, len);
    }
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void commands_process_packet(unsigned char *data, unsigned int len) {
    if (!len) {
        return;
    }

    COMM_PACKET_ID packet_id;
    int32_t ind = 0;

    packet_id = data[0];
    data++;
    len--;

    switch (packet_id) {
        case COMM_FW_VERSION:
            ind = 0;
            send_buffer[ind++] = COMM_FW_VERSION;
            send_buffer[ind++] = FW_VERSION_MAJOR;
            send_buffer[ind++] = FW_VERSION_MINOR;

#ifdef HW_NAME
            strcpy((char*)(send_buffer + ind), HW_NAME);
            ind += strlen(HW_NAME);

            strcpy((char*)(send_buffer + ind), __BUILD__);
            ind += strlen(__BUILD__);

            strcpy((char*)(send_buffer + ind), __FOR__);
            ind += strlen(__FOR__);

            strcpy((char*)(send_buffer + ind), __BY__);
            ind += strlen(__BY__);

            strcpy((char*)(send_buffer + ind), __DATE__);
            ind += strlen(__DATE__);

            strcpy((char*)(send_buffer + ind), __AT__);
            ind += strlen(__AT__);

            strcpy((char*)(send_buffer + ind), __TIME__);
            ind += strlen(__TIME__);

            ind += 1;

            memcpy(send_buffer + ind, STM32_UUID_8, 12);
            ind += 12;
#endif

            commands_send_packet(send_buffer, ind);
            break;

        case COMM_GET_VALUES:
            ind = 0;
            send_buffer[ind++] = COMM_GET_VALUES;
            buffer_append_float16(send_buffer, *exposed_floats[2], 1e1, &ind);//T FET
            buffer_append_float16(send_buffer, *exposed_floats[3], 1e1, &ind);//T Motor
            buffer_append_float32(send_buffer, motors[0].current_control.Ibus, 1e2, &ind);//I Motor
            buffer_append_float32(send_buffer, motors[1].current_control.Ibus, 1e2, &ind);//I Batt
            buffer_append_float32(send_buffer, motors[0].current_control.Id_measured, 1e2, &ind);//motor_id
            buffer_append_float32(send_buffer, motors[0].current_control.Iq_measured, 1e2, &ind);//motor_iq
            buffer_append_float16(send_buffer, motors[0].vel_setpoint, 1e1, &ind);//Duty
            buffer_append_float32(send_buffer, motors[0].sensorless.pll_vel, 1e0, &ind);//ERPM
            buffer_append_float16(send_buffer, *exposed_floats[VBUS_VOLTAGE], 1e1, &ind);//Volts In
            buffer_append_float32(send_buffer, 0.0f, 1e1, &ind);//Ah Draw
            buffer_append_float32(send_buffer, motors[0].current_setpoint, 1e1, &ind);//Ah Charge
            buffer_append_float32(send_buffer, motors[0].current_control.Id_measured, 1e4, &ind);//Wh Graw
            buffer_append_float32(send_buffer, motors[0].current_control.Iq_measured, 1e4, &ind);//Wh Charge
            buffer_append_int32(send_buffer, *exposed_ints[M0_ERROR], &ind);//Tac
            buffer_append_int32(send_buffer, *exposed_ints[M1_ERROR], &ind);//Tac ABS
            send_buffer[ind++] = 0;//Fault
            buffer_append_float32(send_buffer, motors[0].sensorless.pll_pos, 1e6, &ind);//pid_pos_mow
            commands_send_packet(send_buffer, ind);
            break;

        case COMM_SET_DUTY://D
            ind = 0;
            float m;
//            mc_interface_set_duty((float)buffer_get_int32(data, &ind) / 100000.0f);
            m = (float)buffer_get_int32(data, &ind) / 100000.0f;
            if (m == 0.0f) {
                motor = &motors[0];
                commands_printf("%-32s : oked\n", "Now motor is M0.");
            } else if (m == 1.0f) {
                motor = &motors[1];
                commands_printf("%-32s : oked\n", "Now motor is M1.");
            } else {
                commands_printf("%-32s : failured\n", "Now motor is NONE.");
            }

            break;

        case COMM_SET_CURRENT://I
            ind = 0;
//            mc_interface_set_current((float)buffer_get_int32(data, &ind) / 1000.0f);
//            s_current_setpoint(motor, (float)buffer_get_int32(data, &ind) / 1000.0f);
            current_feed_forward = (float)buffer_get_int32(data, &ind) / 1000.0f;
            commands_printf("%-32s : %3.3f\n", "SET_CURRENT", current_feed_forward);

            break;

        case COMM_SET_CURRENT_BRAKE://IB
            ind = 0;
            float mode;
//            mc_interface_set_brake_current((float)buffer_get_int32(data, &ind) / 1000.0f);
            mode = (float)buffer_get_int32(data, &ind) / 1000.0f;
            if (mode == 0.0f) {
//                s_mode_setpoint(motor, CTRL_MODE_VOLTAGE_CONTROL);
                for (int i = 0; i < num_motors; i++) {
                    set_vel_setpoint(&motors[i], 0.0f, 0.0f);
                }
            } else if (mode == 1.0f) {
//                s_mode_setpoint(motor, CTRL_MODE_CURRENT_CONTROL);
                set_current_setpoint(motor, current_feed_forward);
            } else if (mode == 2.0f) {
//                s_mode_setpoint(motor, CTRL_MODE_VELOCITY_CONTROL);
                set_vel_setpoint(motor, vel_feed_forward, current_feed_forward);
            } else if (mode == 3.0f) {
//                s_mode_setpoint(motor, CTRL_MODE_POSITION_CONTROL);
                set_pos_setpoint(motor, pos_setpoint, vel_feed_forward, current_feed_forward);
            }

            break;

        case COMM_SET_RPM://V
            ind = 0;
//            mc_interface_set_pid_speed((float)buffer_get_int32(data, &ind));
//            s_vel_setpoint(motor, (float)buffer_get_int32(data, &ind));
//            set_vel_setpoint(motor, (float)buffer_get_int32(data, &ind), 1.0f);//it is oked.
            vel_feed_forward = (float)buffer_get_int32(data, &ind);
            commands_printf("%-32s : %3.3f\n", "SET_VELOCITY", vel_feed_forward);

            break;

        case COMM_SET_POS://P
            ind = 0;
//            mc_interface_set_pid_pos((float)buffer_get_int32(data, &ind) / 1000000.0f);
//            s_pos_setpoint(motor, (float)buffer_get_int32(data, &ind) / 1000000.0f);
            pos_setpoint = (float)buffer_get_int32(data, &ind) / 1000000.0f;
            commands_printf("%-32s : %3.3f\n", "SET_POSITION", pos_setpoint);

            break;

        case COMM_SET_HANDBRAKE://HB
            ind = 0;
//            mc_interface_set_handbrake(buffer_get_float32(data, 1e3, &ind));
            for (int i = 0; i < num_motors; i++) {
                set_vel_setpoint(&motors[i], 0.0f, 0.0f);
            }
            break;

        case COMM_GET_MCCONF:
        case COMM_GET_MCCONF_DEFAULT:
            ind = 0;
            send_buffer[ind++] = packet_id;
            commands_send_packet(send_buffer, ind);
            break;
        case COMM_GET_APPCONF:
        case COMM_GET_APPCONF_DEFAULT:
            ind = 0;
            send_buffer[ind++] = packet_id;
            commands_send_packet(send_buffer, ind);
            break;
        case COMM_TERMINAL_CMD:
            data[len] = '\0';
            terminal_process_string((char*)data);
            break;
        case COMM_REBOOT:
            // Lock the system and enter an infinite loop. The watchdog will reboot.
            __disable_irq();
            // Use the WWDG to reset the MCU

            // Clear pending interrupts
            SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;

            // Disable all interrupts
            for (int i = 0; i < 8; i++) {
                NVIC->ICER[i] = NVIC->IABR[i];
            }
            for (;;) {};
            break;

        default:
            break;
    }
}

void commands_printf(const char* format, ...) {
    va_list arg;
    va_start(arg, format);
    int len;
    static char print_buffer[1023];

    print_buffer[0] = COMM_PRINT;
    len = vsnprintf(print_buffer+1, 1022, format, arg);
    va_end(arg);

    if (len > 0) {
        commands_send_packet((unsigned char*)print_buffer, (len<1022)? len+1: 1023);
    }
}

void commands_send_rotor_pos(float rotor_pos) {
    uint8_t buffer[5];
    int32_t index = 0;

    buffer[index++] = COMM_ROTOR_POSITION;
    buffer_append_int32(buffer, (int32_t)(rotor_pos * 100000.0f), &index);

    commands_send_packet(buffer, index);
}

void commands_send_experiment_samples(float *samples, int len) {
    if ((len * 4 + 1) > 256) {
        return;
    }

    uint8_t buffer[len * 4 + 1];
    int32_t index = 0;

    buffer[index++] = COMM_EXPERIMENT_SAMPLE;

    for (int i = 0; i < len; i++) {
        buffer_append_int32(buffer, (int32_t)(samples[i] * 10000.0f), &index);
    }

    commands_send_packet(buffer, index);
}