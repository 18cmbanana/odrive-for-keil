/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <commands.h>
#include <usart.h>
#include <gpio.h>
#include <freertos_vars.h>
#include <usbd_cdc.h>
#include <utils.h>

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
// This automatically updates to the interface that most
// recently recieved a command. In the future we may want to separate
// debug printf and the main serial comms.
SerialPrintf_t serial_printf_select = SERIAL_PRINTF_IS_NONE;

/* Private constant data -----------------------------------------------------*/
// TODO: make command to switch gpio_mode during run-time
//static const GpioMode_t gpio_mode = GPIO_MODE_NONE;     //GPIO 1,2 is not configured
static const GpioMode_t gpio_mode = GPIO_MODE_UART;     //GPIO 1,2 is UART Tx,Rx
// static const GpioMode_t gpio_mode = GPIO_MODE_STEP_DIR; //GPIO 1,2 is M0 Step,Dir

static uint8_t* usb_buf;
static uint32_t usb_len;
extern USBD_HandleTypeDef hUsbDeviceFS;

static uint32_t command_received_cnt = 0;

// variables exposed to usb/serial interface via set/get/monitor
// Note: this will be depricated soon
float* const exposed_floats[] = {
    &vbus_voltage, // ro
    NULL, //&elec_rad_per_enc, // ro
    &motors[0].pos_setpoint, // rw
    &motors[0].pos_gain, // rw
    &motors[0].vel_setpoint, // rw
    &motors[0].vel_gain, // rw
    &motors[0].vel_integrator_gain, // rw
    &motors[0].vel_integrator_current, // rw
    &motors[0].vel_limit, // rw
    &motors[0].current_setpoint, // rw
    &motors[0].calibration_current, // rw
    &motors[0].phase_inductance, // ro
    &motors[0].phase_resistance, // ro
    &motors[0].current_meas.phB, // ro
    &motors[0].current_meas.phC, // ro
    &motors[0].DC_calib.phB, // rw
    &motors[0].DC_calib.phC, // rw
    &motors[0].shunt_conductance, // rw
    &motors[0].phase_current_rev_gain, // rw
    &motors[0].current_control.current_lim, // rw
    &motors[0].current_control.p_gain, // rw
    &motors[0].current_control.i_gain, // rw
    &motors[0].current_control.v_current_control_integral_d, // rw
    &motors[0].current_control.v_current_control_integral_q, // rw
    &motors[0].current_control.Ibus, // ro
    &motors[0].encoder.phase, // ro
    &motors[0].encoder.pll_pos, // rw
    &motors[0].encoder.pll_vel, // rw
    &motors[0].encoder.pll_kp, // rw
    &motors[0].encoder.pll_ki, // rw
    &motors[1].pos_setpoint, // rw
    &motors[1].pos_gain, // rw
    &motors[1].vel_setpoint, // rw
    &motors[1].vel_gain, // rw
    &motors[1].vel_integrator_gain, // rw
    &motors[1].vel_integrator_current, // rw
    &motors[1].vel_limit, // rw
    &motors[1].current_setpoint, // rw
    &motors[1].calibration_current, // rw
    &motors[1].phase_inductance, // ro
    &motors[1].phase_resistance, // ro
    &motors[1].current_meas.phB, // ro
    &motors[1].current_meas.phC, // ro
    &motors[1].DC_calib.phB, // rw
    &motors[1].DC_calib.phC, // rw
    &motors[1].shunt_conductance, // rw
    &motors[1].phase_current_rev_gain, // rw
    &motors[1].current_control.current_lim, // rw
    &motors[1].current_control.p_gain, // rw
    &motors[1].current_control.i_gain, // rw
    &motors[1].current_control.v_current_control_integral_d, // rw
    &motors[1].current_control.v_current_control_integral_q, // rw
    &motors[1].current_control.Ibus, // ro
    &motors[1].encoder.phase, // ro
    &motors[1].encoder.pll_pos, // rw
    &motors[1].encoder.pll_vel, // rw
    &motors[1].encoder.pll_kp, // rw
    &motors[1].encoder.pll_ki, // rw
};

int* const exposed_ints[] = {
    (int*)&motors[0].control_mode, // rw
    &motors[0].encoder.encoder_offset, // rw
    &motors[0].encoder.encoder_state, // ro
    &motors[0].error, // rw
    (int*)&motors[1].control_mode, // rw
    &motors[1].encoder.encoder_offset, // rw
    &motors[1].encoder.encoder_state, // ro
    &motors[1].error, // rw
};

bool* const exposed_bools[] = {
    &motors[0].thread_ready, // ro
    &motors[0].enable_control, // rw
    &motors[0].do_calibration, // rw
    &motors[0].calibration_ok, // ro
    &motors[1].thread_ready, // ro
    &motors[1].enable_control, // rw
    &motors[1].do_calibration, // rw
    &motors[1].calibration_ok, // ro
};

uint16_t* const exposed_uint16[] = {
    &motors[0].control_deadline, // rw
    &motors[0].last_cpu_time, // ro
    &motors[1].control_deadline, // rw
    &motors[1].last_cpu_time, // ro
};

/* Private variables ---------------------------------------------------------*/
monitoring_slot monitoring_slots[20] = {0};
/* Private function prototypes -----------------------------------------------*/
//static void print_monitoring(int limit);

#if defined ARM_PRINTF

#include <usart.h>
#include <usbd_cdc_if.h>

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define UART_TX_BUFFER_SIZE 1024
static uint8_t uart_tx_buf[UART_TX_BUFFER_SIZE];

int commands_write(char* data, int len) {
    //number of bytes written
    int written = 0;
    switch (serial_printf_select) {
        case SERIAL_PRINTF_IS_USB: {
            // Wait on semaphore for the interface to be available
            // Note that the USB driver will release the interface again when the TX completes
            const uint32_t usb_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_usb_tx, usb_tx_timeout);
            if (sem_stat == osOK) {
                uint8_t status = CDC_Transmit_FS((uint8_t*)data, len);  // transmit over CDC
                written = (status == USBD_OK) ? len : 0;
            } // If the semaphore times out, we simply leave "written" as 0
        }
        break;

        case SERIAL_PRINTF_IS_UART: {
            //Check length
            if (len > UART_TX_BUFFER_SIZE)
                return 0;
            // Wait on semaphore for the interface to be available
            // Note that HAL_UART_TxCpltCallback will release the interface again when the TX completes
            const uint32_t uart_tx_timeout = 100; // ms
            osStatus sem_stat = osSemaphoreWait(sem_uart_dma, uart_tx_timeout);
            if (sem_stat == osOK) {
                memcpy(uart_tx_buf, data, len);                    // memcpy data into uart_tx_buf
                HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, len);  // Start DMA background transfer
            } // If the semaphore times out, we simply leave "written" as 0
        }
        break;

        default: {
            written = 0;
        }
        break;
    }

    return written;
}

void cmd_printf(const char* format, ...) {
    va_list arg;
    va_start(arg, format);
    int len;
    static char print_buffer[1023];

    len = vsnprintf(print_buffer, 1023, format, arg);
    va_end(arg);

    if (len > 0) {
        commands_write((char*)print_buffer, (len<1022) ? len+1 : 1023);
    }
}

#if defined ARM_TERMINAL

#define CALLBACK_LEN 32

// Private types
typedef struct _terminal_callback_struct {
    const char *command;
    const char *help;
    const char *arg_names;
    void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;

// Private variables
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;

void commands_process_string(uint8_t* buffer, int len, SerialPrintf_t response_interface) {
    // Set response interface
    serial_printf_select = response_interface;

    // TODO very hacky way of terminating sscanf at end of buffer:
    // We should do some proper struct packing instead of using sscanf altogether
    if (len) {
        buffer[len-1] = 0;
    }

    //
    enum { kMaxArgs = 64 };
    int argc = 0;
    char *argv[kMaxArgs];

    char *str;
    buffer[len] = '\0';
    str = (char*)buffer;

    char *p2 = strtok(str, " ");
    while (p2 && argc < kMaxArgs) {
        argv[argc++] = p2;
        p2 = strtok(0, " ");
    }

    if (argc == 0) {
        return;
    }

    // check incoming packet type
    if (strcmp(argv[0], "p") == 0) {
        // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;

        sscanf(argv[1], "%u", &motor_number);
        if (argc == 5 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &pos_setpoint);
            sscanf(argv[3], "%f", &vel_feed_forward);
            sscanf(argv[4], "%f", &current_feed_forward);
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        } else {
            cmd_printf("This command requires 5 argument.\n");
        }
    } else if (strcmp(argv[0], "v") == 0) {
        // velocity control
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        sscanf(argv[1], "%u", &motor_number);
        if (argc == 4 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &vel_feed_forward);
            sscanf(argv[3], "%f", &current_feed_forward);
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "c") == 0) {
        // current control
        unsigned motor_number;
        float current_feed_forward;
        sscanf(argv[1], "%u", &motor_number);
        if (argc == 3 && motor_number < num_motors) {
            sscanf(argv[2], "%f", &current_feed_forward);
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        } else {
            cmd_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "i") == 0) {// Dump device info
        // Retrieves the device signature, revision, flash size, and UUID
        cmd_printf("Signature: %#x\n", STM_ID_GetSignature());
        cmd_printf("Revision: %#x\n", STM_ID_GetRevision());
        cmd_printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        cmd_printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));
    } else if (strcmp(argv[0], "g") == 0) { // GET
        // g <0:float,1:int,2:bool,3:uint16> index
        int type = 0;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: {
                    cmd_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 1: {
                    cmd_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 2: {
                    cmd_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 3: {
                    cmd_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "get") == 0) { // GET
        // g <f:float,i:int,b:bool,u:uint16> index
        char type;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%s", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 'f': {
                    cmd_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 'i': {
                    cmd_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 'b': {
                    cmd_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 'u': {
                    cmd_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "h") == 0) { // HALT
        for (int i = 0; i < num_motors; i++) {
            set_vel_setpoint(&motors[i], 0.0f, 0.0f);
        }
    } else if (strcmp(argv[0], "s") == 0) { // SET
        // s <0:float,1:int,2:bool,3:uint16> index value
        int type = 0;
        int index = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: {
                    sscanf(argv[3], "%f", exposed_floats[index]);
                    break;
                };
                case 1: {
                    sscanf(argv[3], "%d", exposed_ints[index]);
                    break;
                };
                case 2: {
                    int btmp = 0;
                    sscanf(argv[3], "%d", &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 3: {
                    sscanf(argv[3], "%hu", exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "set") == 0) { // SET
        // s <0:float,1:int,2:bool,3:uint16> index value
        int type = 0;
        int index = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 'f': {
                    sscanf(argv[3], "%f", exposed_floats[index]);
                    break;
                };
                case 'i': {
                    sscanf(argv[3], "%d", exposed_ints[index]);
                    break;
                };
                case 'b': {
                    int btmp = 0;
                    sscanf(argv[3], "%d", &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 'u': {
                    sscanf(argv[3], "%hu", exposed_uint16[index]);
                    break;
                };
            }
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "m") == 0) { // Setup Monitor
        // m <0:float,1:int,2:bool,3:uint16> index monitoring_slot
        int type = 0;
        int index = 0;
        int slot = 0;
        if (argc == 4) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            sscanf(argv[3], "%u", &slot);
            monitoring_slots[slot].type = type;
            monitoring_slots[slot].index = index;
        } else {
            cmd_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "o") == 0) { // Output Monitor
        int limit = 0;
        if (argc == 2) {
            sscanf(argv[1], "%u", &limit);
            print_monitoring(limit);
        } else {
            cmd_printf("This command requires 2 argument.\n");
        }
    } else if (strcmp(argv[0], "t") == 0) { // Run Anti-Cogging Calibration
        for (int i = 0; i < num_motors; i++) {
            // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }
    } else {
        int i;
        bool found = false;
        for (i = 0; i < callback_write; i++) {
            if (strcmp(argv[0], callbacks[i].command) == 0) {
                callbacks[i].cbf(argc, (const char**)argv);
                found = true;
                break;
            }
        }

        if (found == false) {
            return;
        }
    }
}
/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void commands_register_command_callback(
    const char* command,
    const char *help,
    const char *arg_names,
    void(*cbf)(int argc, const char **argv)) {

    int callback_num = callback_write;

    for (int i = 0; i < callback_write; i++) {
        // First check the address in case the same callback is registered more than once.
        if (callbacks[i].command == command) {
            callback_num = i;
            break;
        }

        // Check by string comparison.
        if (strcmp(callbacks[i].command, command) == 0) {
            callback_num = i;
            break;
        }
    }

    callbacks[callback_num].command = command;
    callbacks[callback_num].help = help;
    callbacks[callback_num].arg_names = arg_names;
    callbacks[callback_num].cbf = cbf;

    if (callback_num == callback_write) {
        callback_write++;
        if (callback_write >= CALLBACK_LEN) {
            callback_write = 0;
        }
    }
}
#endif
#endif

#if defined ARM_COMMMAND_UART

#include "packet.h"
#include "commands_pro.h"

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);
static void init_packet(void);

static void process_packet(unsigned char *data, unsigned int len) {
    commands_set_send_func(send_packet_wrapper);
    commands_process_packet(data, len);
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
    packet_send_packet(data, len, PACKET_HANDLER);
}

static void send_packet(unsigned char *data, unsigned int len) {
    commands_write((char *)data, len);
}
static void init_packet(void) {
    packet_init(send_packet, process_packet, PACKET_HANDLER);
}
#endif


/* Function implementations --------------------------------------------------*/
void init_communication() {
    switch (gpio_mode) {
        case GPIO_MODE_NONE:
            break; //do nothing
        case GPIO_MODE_UART: {
#if defined ARM_COMMMAND_UART
            init_packet();
#endif
            SetGPIO12toUART();
        }
        break;
        case GPIO_MODE_STEP_DIR: {
            SetGPIO12toStepDir();
        }
        break;
        default:
            //TODO: report error unexpected mode
            break;
    }
}

void motor_parse_cmd(uint8_t* buffer, int len, SerialPrintf_t response_interface) {
    // Set response interface
    serial_printf_select = response_interface;

    // TODO very hacky way of terminating sscanf at end of buffer:
    // We should do some proper struct packing instead of using sscanf altogether
    buffer[len-1] = 0;
//	  ((uint8_t *)buffer)[len < buffer_capacity ? len : (buffer_capacity - 1)] = 0;

    // check incoming packet type
    if (buffer[0] == 'p') {
        // position control
        unsigned motor_number;
        float pos_setpoint, vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "p %u %f %f %f", &motor_number, &pos_setpoint, &vel_feed_forward, &current_feed_forward);
        if (numscan == 4 && motor_number < num_motors) {
            set_pos_setpoint(&motors[motor_number], pos_setpoint, vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'v') {
        // velocity control
        unsigned motor_number;
        float vel_feed_forward, current_feed_forward;
        int numscan = sscanf((const char*)buffer, "v %u %f %f", &motor_number, &vel_feed_forward, &current_feed_forward);
        if (numscan == 3 && motor_number < num_motors) {
            set_vel_setpoint(&motors[motor_number], vel_feed_forward, current_feed_forward);
        }
    } else if (buffer[0] == 'c') {
        // current control
        unsigned motor_number;
        float current_feed_forward;
        int numscan = sscanf((const char*)buffer, "c %u %f", &motor_number, &current_feed_forward);
        if (numscan == 2 && motor_number < num_motors) {
            set_current_setpoint(&motors[motor_number], current_feed_forward);
        }
    } else if (buffer[0] == 'i') { // Dump device info
        // Retrieves the device signature, revision, flash size, and UUID
        cmd_printf("Signature: %#x\n", STM_ID_GetSignature());
        cmd_printf("Revision: %#x\n", STM_ID_GetRevision());
        cmd_printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        cmd_printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));
    } else if (buffer[0] == 'g') { // GET
        // g <0:float,1:int,2:bool,3:uint16> index
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char*)buffer, "g %u %u", &type, &index);
        if (numscan == 2) {
            switch (type) {
                case 0: {
                    cmd_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 1: {
                    cmd_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 2: {
                    cmd_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 3: {
                    cmd_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        }
    } else if (buffer[0] == 'h') { // HALT
        for (int i = 0; i < num_motors; i++) {
            set_vel_setpoint(&motors[i], 0.0f, 0.0f);
        }
    } else if (buffer[0] == 's') { // SET
        // s <0:float,1:int,2:bool,3:uint16> index value
        int type = 0;
        int index = 0;
        int numscan = sscanf((const char*)buffer, "s %u %u", &type, &index);
        if (numscan == 2) {
            switch (type) {
                case 0: {
                    sscanf((const char*)buffer, "s %u %u %f", &type, &index, exposed_floats[index]);
                    break;
                };
                case 1: {
                    sscanf((const char*)buffer, "s %u %u %d", &type, &index, exposed_ints[index]);
                    break;
                };
                case 2: {
                    int btmp = 0;
                    sscanf((const char*)buffer, "s %u %u %d", &type, &index, &btmp);
                    *exposed_bools[index] = btmp ? true : false;
                    break;
                };
                case 3: {
                    sscanf((const char*)buffer, "s %u %u %hu", &type, &index, exposed_uint16[index]);
                    break;
                };
            }
        }
    } else if (buffer[0] == 'm') { // Setup Monitor
        // m <0:float,1:int,2:bool,3:uint16> index monitoring_slot
        int type = 0;
        int index = 0;
        int slot = 0;
        int numscan = sscanf((const char*)buffer, "m %u %u %u", &type, &index, &slot);
        if (numscan == 3) {
            monitoring_slots[slot].type = type;
            monitoring_slots[slot].index = index;
        }
    } else if (buffer[0] == 'o') { // Output Monitor
        int limit = 0;
        int numscan = sscanf((const char*)buffer, "o %u", &limit);
        if (numscan == 1) {
            print_monitoring(limit);
        }
    } else if (buffer[0] == 't') { // Run Anti-Cogging Calibration
        for (int i = 0; i < num_motors; i++) {
            // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }
    }
}

void print_monitoring(int limit) {
    for (int i=0; i<limit; i++) {
        switch (monitoring_slots[i].type) {
            case 0:
                cmd_printf("%f\t",*exposed_floats[monitoring_slots[i].index]);
                break;
            case 1:
                cmd_printf("%d\t",*exposed_ints[monitoring_slots[i].index]);
                break;
            case 2:
                cmd_printf("%d\t",*exposed_bools[monitoring_slots[i].index]);
                break;
            case 3:
                cmd_printf("%hu\t",*exposed_uint16[monitoring_slots[i].index]);
                break;
            default:
                i=100;
        }
    }
    cmd_printf("\n");
}

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void cmd_parse_thread(void const * argument) {

    //DMA open loop continous circular buffer
    //1ms delay periodic, chase DMA ptr around, on new data:
    // Check for start char
    // copy into parse-buffer
    // check for end-char
    // checksum, etc.

#define UART_RX_BUFFER_SIZE 64
    static uint8_t dma_circ_buffer[UART_RX_BUFFER_SIZE];
    static uint8_t parse_buffer[UART_RX_BUFFER_SIZE];

    // DMA is set up to recieve in a circular buffer forever.
    // We dont use interrupts to fetch the data, instead we periodically read
    // data out of the circular buffer into a parse buffer, controlled by a state machine
    HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));

    uint32_t last_rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
    // Re-run state-machine forever
    for (;;) {
        //Inialize recieve state machine
        bool reset_read_state = false;
        bool read_active = false;
        uint32_t parse_buffer_idx = 0;
        //Run state machine until reset
        do {
            // Check for UART errors and restart recieve DMA transfer if required
            if (huart4.ErrorCode != HAL_UART_ERROR_NONE) {
                HAL_UART_AbortReceive(&huart4);
                HAL_UART_Receive_DMA(&huart4, dma_circ_buffer, sizeof(dma_circ_buffer));
                break; //reset state machine
            }
            // Fetch the circular buffer "write pointer", where it would write next
            uint32_t rcv_idx = UART_RX_BUFFER_SIZE - huart4.hdmarx->Instance->NDTR;
            // During sleeping, we may have fallen several characters behind, so we keep
            // going until we are caught up, before we sleep again
            while (rcv_idx != last_rcv_idx) {
                // Fetch the next char, rotate read ptr
                uint8_t c = dma_circ_buffer[last_rcv_idx];
#if defined ARM_COMMMAND_UART
                // Set response interface
                serial_printf_select = SERIAL_PRINTF_IS_UART;
                packet_process_byte(c, PACKET_HANDLER);
#else
                {
                    // Wait on semaphore for the interface to be available
                    // Note that HAL_UART_TxCpltCallback will release the interface again when the TX completes
                    const uint32_t uart_tx_timeout = 100; // ms
                    osStatus sem_stat = osSemaphoreWait(sem_uart_dma, uart_tx_timeout);
                    if (sem_stat == osOK) {
                        HAL_UART_Transmit_DMA(&huart4, &c, 1);  // Start DMA background transfer
                    } // If the semaphore times out, we simply leave "written" as 0
                }
#endif
                if (++last_rcv_idx == UART_RX_BUFFER_SIZE)
                    last_rcv_idx = 0;
                // Look for start character
                if (c == '$') {
                    read_active = true;
                    continue; // do not record start char
                }
                // Record into parse buffer when actively reading
                if (read_active) {
                    parse_buffer[parse_buffer_idx++] = c;
                    if (c == '\r' || c == '\n' || c == '!') {
                        // End of command string: exchange end char with terminating null
                        parse_buffer[parse_buffer_idx-1] = '\0';
                        cmd_printf("In [%u] : \n", command_received_cnt++);
#if defined ARM_TERMINAL
                        commands_process_string(parse_buffer, parse_buffer_idx, SERIAL_PRINTF_IS_UART);
#else
                        motor_parse_cmd(parse_buffer, parse_buffer_idx, SERIAL_PRINTF_IS_UART);
#endif

                        // Reset receieve state machine
                        reset_read_state = true;
                        break;
                    } else if (parse_buffer_idx == UART_RX_BUFFER_SIZE - 1) {
                        // We are not at end of command, and receiving another character after this
                        // would go into the last slot, which is reserved for terminating null.
                        // We have effectively overflowed parse buffer: abort.
                        reset_read_state = true;
                        break;
                    }
                }
            }
            // When we reach here, we are out of immediate characters to fetch out of UART buffer
            // Now we check if there is any USB processing to do: we wait for up to 1 ms,
            // before going back to checking UART again.
            const uint32_t usb_check_timeout = 1; // ms
            osStatus sem_stat = osSemaphoreWait(sem_usb_rx, usb_check_timeout);
            if (sem_stat == osOK) {
                cmd_printf("In [%u] : \n", command_received_cnt++);
#if defined ARM_TERMINAL
                commands_process_string(usb_buf, usb_len, SERIAL_PRINTF_IS_USB);
//                commands_process_string(usb_buf, usb_len, SERIAL_PRINTF_IS_UART);// it is oked.
#else
                motor_parse_cmd(usb_buf, usb_len, SERIAL_PRINTF_IS_USB);
#endif
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);  // Allow next packet
            }
        } while (!reset_read_state);
    }
    // If we get here, then this task is done
    vTaskDelete(osThreadGetId());
}

// Called from CDC_Receive_FS callback function, this allows motor_parse_cmd to access the
// incoming USB data
void set_cmd_buffer(uint8_t *buf, uint32_t len) {
    usb_buf = buf;
    usb_len = len;
}

void usb_update_thread() {
    for (;;) {
        // Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
        osStatus semaphore_status = osSemaphoreWait(sem_usb_irq, osWaitForever);
        if (semaphore_status == osOK) {
            // We have a new incoming USB transmission: handle it
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
            // Let the irq (OTG_FS_IRQHandler) fire again.
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
    vTaskDelete(osThreadGetId());
}

void packet_timer_thread(void const * argument) {
    /* 阻塞1ms. 注:宏pdMS_TO_TICKS用于将毫秒转成节拍数,FreeRTOS V8.1.0及
       以上版本才有这个宏,如果使用低版本,可以使用 1 / portTICK_RATE_MS */
    const portTickType xDelay = pdMS_TO_TICKS(1);
    // Re-run state-machine forever
    for (;;) {
        packet_timerfunc();
        vTaskDelay(xDelay);
    }
    // If we get here, then this task is done
    vTaskDelete(osThreadGetId());
}
