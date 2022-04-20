#ifndef COMMANDS_H
#define COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include <low_level.h>
/* Exported types ------------------------------------------------------------*/

typedef enum {
    GPIO_MODE_NONE,
    GPIO_MODE_UART,
    GPIO_MODE_STEP_DIR,
} GpioMode_t;

typedef enum {
    SERIAL_PRINTF_IS_NONE,
    SERIAL_PRINTF_IS_USB,
    SERIAL_PRINTF_IS_UART,
} SerialPrintf_t;

typedef enum {
    VBUS_VOLTAGE = 0, // RO
    NULLED, //ELEC_RAD_PER_ENC, // RO
    M0_POS_SETPOINT, // RW
    M0_POS_GAIN, // RW
    M0_VEL_SETPOINT, // RW
    M0_VEL_GAIN, // RW
    M0_VEL_INTEGRATOR_GAIN, // RW
    M0_VEL_INTEGRATOR_CURRENT, // RW
    M0_VEL_LIMIT, // RW
    M0_CURRENT_SETPOINT, // RW
    M0_CALIBRATION_CURRENT, // RW
    M0_PHASE_INDUCTANCE, // RO
    M0_PHASE_RESISTANCE, // RO
    M0_CURRENT_MEAS_PHB, // RO
    M0_CURRENT_MEAS_PHC, // RO
    M0_DC_CALIB_PHB, // RW
    M0_DC_CALIB_PHC, // RW
    M0_SHUNT_CONDUCTANCE, // RW
    M0_PHASE_CURRENT_REV_GAIN, // RW
    M0_CURRENT_CONTROL_CURRENT_LIM, // RW
    M0_CURRENT_CONTROL_P_GAIN, // RW
    M0_CURRENT_CONTROL_I_GAIN, // RW
    M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D, // RW
    M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q, // RW
    M0_CURRENT_CONTROL_IBUS, // RO
    M0_ENCODER_PHASE, // RO
    M0_ENCODER_PLL_POS, // RW
    M0_ENCODER_PLL_VEL, // RW
    M0_ENCODER_PLL_KP, // RW
    M0_ENCODER_PLL_KI, // RW
    M1_POS_SETPOINT, // RW
    M1_POS_GAIN, // RW
    M1_VEL_SETPOINT, // RW
    M1_VEL_GAIN, // RW
    M1_VEL_INTEGRATOR_GAIN, // RW
    M1_VEL_INTEGRATOR_CURRENT, // RW
    M1_VEL_LIMIT, // RW
    M1_CURRENT_SETPOINT, // RW
    M1_CALIBRATION_CURRENT, // RW
    M1_PHASE_INDUCTANCE, // RO
    M1_PHASE_RESISTANCE, // RO
    M1_CURRENT_MEAS_PHB, // RO
    M1_CURRENT_MEAS_PHC, // RO
    M1_DC_CALIB_PHB, // RW
    M1_DC_CALIB_PHC, // RW
    M1_SHUNT_CONDUCTANCE, // RW
    M1_PHASE_CURRENT_REV_GAIN, // RW
    M1_CURRENT_CONTROL_CURRENT_LIM, // RW
    M1_CURRENT_CONTROL_P_GAIN, // RW
    M1_CURRENT_CONTROL_I_GAIN, // RW
    M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D, // RW
    M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q, // RW
    M1_CURRENT_CONTROL_IBUS, // RO
    M1_ENCODER_PHASE, // RO
    M1_ENCODER_PLL_POS, // RW
    M1_ENCODER_PLL_VEL, // RW
    M1_ENCODER_PLL_KP, // RW
    M1_ENCODER_PLL_KI, // RW
		FLOATS_END
} Exposed_Floats_t;

typedef enum {
    M0_CONTROL_MODE = 0, // RW
    M0_ENCODER_ENCODER_OFFSET, // RW
    M0_ENCODER_ENCODER_STATE, // RO
    M0_ERROR, // RW
    M1_CONTROL_MODE, // RW
    M1_ENCODER_ENCODER_OFFSET, // RW
    M1_ENCODER_ENCODER_STATE, // RO
    M1_ERROR, // RW
		INTS_END
} Exposed_Ints_t;

typedef enum {
    M0_THREAD_READY = 0, // RO
    M0_ENABLE_CONTROL, // RW
    M0_DO_CALIBRATION, // RW
    M0_CALIBRATION_OK, // RO
    M1_THREAD_READY, // RO
    M1_ENABLE_CONTROL, // RW
    M1_DO_CALIBRATION, // RW
    M1_CALIBRATION_OK, // RO
		BOOLS_END
} Exposed_Bools_t;

typedef enum {
    M0_CONTROL_DEADLINE = 0, // RW
    M0_LAST_CPU_TIME, // RO
    M1_CONTROL_DEADLINE, // RW
    M1_LAST_CPU_TIME, // RO
		UINT16_END
} Exposed_Uint16_t;
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern SerialPrintf_t serial_printf_select;

extern float* const exposed_floats[];

extern int* const exposed_ints[];

extern bool* const exposed_bools[];

extern uint16_t* const exposed_uint16[];

extern monitoring_slot monitoring_slots[20];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void init_communication();
void cmd_parse_thread(void const * argument);
void packet_timer_thread(void const * argument);
void motor_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface);

#define ARM_TERMINAL
#if defined ARM_TERMINAL
void commands_process_string(uint8_t* buffer, int len, SerialPrintf_t response_interface);
void commands_register_command_callback(
    const char* command,
    const char *help,
    const char *arg_names,
    void(*cbf)(int argc, const char **argv));
#endif

void print_monitoring(int limit);		
void set_cmd_buffer(uint8_t *buf, uint32_t len);
void usb_update_thread();

#if defined ARM_PRINTF

#endif
#if defined ARM_TERMINAL

#endif
		
#define ARM_COMMMAND_UART		
#if defined ARM_COMMMAND_UART

#endif		

#if defined ARM_PRINTF
void cmd_printf(const char* format, ...);
#endif
#if defined ARM_COMMMAND_UART
#define PACKET_HANDLER 0
#endif			

#endif /* COMMANDS_H */
