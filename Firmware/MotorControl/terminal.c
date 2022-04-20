/*

 */

#include "terminal.h"
#include "commands_pro.h"
#include "utils.h"

#include "low_level.h"
#include "commands.h"

#include "version.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

// Settings
#define CALLBACK_LEN                        32

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
static uint32_t comm_received_cnt = 0;

void terminal_process_string(char *str) {
    enum { kMaxArgs = 64 };
    int argc = 0;
    char *argv[kMaxArgs];

    char *p2 = strtok(str, " ");
    while (p2 && argc < kMaxArgs) {
        argv[argc++] = p2;
        p2 = strtok(0, " ");
    }

    commands_printf("\nIn [%u] : %s", comm_received_cnt++, argv[0]);
    commands_printf("----------------------------------------------------------------");

    if (argc == 0) {
        commands_printf("No command received\n");
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
            commands_printf("This command requires 5 argument.\n");
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
            commands_printf("This command requires 4 argument.\n");
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
            commands_printf("This command requires 3 argument.\n");
        }
    } else if (strcmp(argv[0], "i") == 0) {// Dump device info
        // Retrieves the device signature, revision, flash size, and UUID
        commands_printf("Signature: %#x\n", STM_ID_GetSignature());
        commands_printf("Revision: %#x\n", STM_ID_GetRevision());
        commands_printf("Flash Size: %#x KiB\n", STM_ID_GetFlashSize());
        commands_printf("UUID: 0x%lx%lx%lx\n", STM_ID_GetUUID(2), STM_ID_GetUUID(1), STM_ID_GetUUID(0));
    } else if (strcmp(argv[0], "g") == 0) { // GET
        // g <0:float,1:int,2:bool,3:uint16> index
        int type = 0;
        int index = 0;
        if (argc == 3) {
            sscanf(argv[1], "%u", &type);
            sscanf(argv[2], "%u", &index);
            switch (type) {
                case 0: {
                    commands_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 1: {
                    commands_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 2: {
                    commands_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 3: {
                    commands_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            commands_printf("This command requires 3 argument.\n");
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
                    commands_printf("%f\n",*exposed_floats[index]);
                    break;
                };
                case 'i': {
                    commands_printf("%d\n",*exposed_ints[index]);
                    break;
                };
                case 'b': {
                    commands_printf("%d\n",*exposed_bools[index]);
                    break;
                };
                case 'u': {
                    commands_printf("%hu\n",*exposed_uint16[index]);
                    break;
                };
            }
        } else {
            commands_printf("This command requires 3 argument.\n");
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
            commands_printf("This command requires 4 argument.\n");
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
            commands_printf("This command requires 4 argument.\n");
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
            commands_printf("This command requires 4 argument.\n");
        }
    } else if (strcmp(argv[0], "o") == 0) { // Output Monitor
        int limit = 0;
        if (argc == 2) {
            sscanf(argv[1], "%u", &limit);
            print_monitoring(limit);
        } else {
            commands_printf("This command requires 2 argument.\n");
        }
    } else if (strcmp(argv[0], "t") == 0) { // Run Anti-Cogging Calibration
        for (int i = 0; i < num_motors; i++) {
            // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
            if (motors[i].anticogging.cogging_map != NULL && motors[i].error == ERROR_NO_ERROR) {
                motors[i].anticogging.calib_anticogging = true;
            }
        }
    } else if (strcmp(argv[0], "getf") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %f\n", i++, "VBUS_VOLTAGE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "NULLED", *exposed_floats[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %f\n", i++, "M0_POS_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_POS_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_INTEGRATOR_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_INTEGRATOR_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_VEL_LIMIT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CALIBRATION_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_PHASE_INDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_PHASE_RESISTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_MEAS_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_MEAS_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_DC_CALIB_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_DC_CALIB_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_SHUNT_CONDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_PHASE_CURRENT_REV_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_CURRENT_LIM", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_P_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_I_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_CURRENT_CONTROL_IBUS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PHASE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_POS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_VEL", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_KP", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M0_ENCODER_PLL_KI", *exposed_floats[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %f\n", i++, "M1_POS_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_POS_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_INTEGRATOR_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_INTEGRATOR_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_VEL_LIMIT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_SETPOINT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CALIBRATION_CURRENT", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_PHASE_INDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_PHASE_RESISTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_MEAS_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_MEAS_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_DC_CALIB_PHB", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_DC_CALIB_PHC", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_SHUNT_CONDUCTANCE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_PHASE_CURRENT_REV_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_CURRENT_LIM", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_P_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_I_GAIN", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_D", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_V_CURRENT_CONTROL_INTEGRAL_Q", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_CURRENT_CONTROL_IBUS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PHASE", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_POS", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_VEL", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_KP", *exposed_floats[index++]);
        commands_printf("[%d]%-32s : %f\n", i++, "M1_ENCODER_PLL_KI", *exposed_floats[index++]);
    } else if (strcmp(argv[0], "geti") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %d\n", i++, "M0_CONTROL_MODE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ENCODER_ENCODER_OFFSET", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ENCODER_ENCODER_STATE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ERROR", *exposed_ints[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %d\n", i++, "M1_CONTROL_MODE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ENCODER_ENCODER_OFFSET", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ENCODER_ENCODER_STATE", *exposed_ints[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ERROR", *exposed_ints[index++]);
    } else if (strcmp(argv[0], "getb") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %d\n", i++, "M0_THREAD_READY", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_ENABLE_CONTROL", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_DO_CALIBRATION", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M0_CALIBRATION_OK", *exposed_bools[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %d\n", i++, "M1_THREAD_READY", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_ENABLE_CONTROL", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_DO_CALIBRATION", *exposed_bools[index++]);
        commands_printf("[%d]%-32s : %d\n", i++, "M1_CALIBRATION_OK", *exposed_bools[index++]);
    } else if (strcmp(argv[0], "getu") == 0) {
        int index = 0;
        int i = 0;
        commands_printf("[%d]%-32s : %hu\n", i++, "M0_CONTROL_DEADLINE", *exposed_uint16[index++]);
        commands_printf("[%d]%-32s : %hu\n", i++, "M0_LAST_CPU_TIME", *exposed_uint16[index++]);
        commands_printf("----------------------------------------------------------------\n");
        commands_printf("[%d]%-32s : %hu\n", i++, "M1_CONTROL_DEADLINE", *exposed_uint16[index++]);
        commands_printf("[%d]%-32s : %hu\n", i++, "M1_LAST_CPU_TIME", *exposed_uint16[index++]);
    } else if (strcmp(argv[0], "hw_status") == 0) {
        commands_printf("Firmware: %d.%d.%d", ODRIVE_FW_VERSION_MAJOR, ODRIVE_FW_VERSION_MINOR, ODRIVE_FW_VERSION_PATCH);
#ifdef HW_NAME
        commands_printf("Hardware: %s", HW_NAME);
#endif
        commands_printf("Firmware version: %s %s", __BUILD__, __FOR__);
        commands_printf("Firmware changelog time: %s", FW_CHANGELOG_TIME);

        commands_printf("UUID: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                        STM32_UUID_8[0], STM32_UUID_8[1], STM32_UUID_8[2], STM32_UUID_8[3],
                        STM32_UUID_8[4], STM32_UUID_8[5], STM32_UUID_8[6], STM32_UUID_8[7],
                        STM32_UUID_8[8], STM32_UUID_8[9], STM32_UUID_8[10], STM32_UUID_8[11]);
        // The help command
    } else if (strcmp(argv[0], "help") == 0) {
        commands_printf("Valid commands are:");
        commands_printf("help");
        commands_printf("  Show this help");

        commands_printf("hw_status");
        commands_printf("  Print some hardware status information.");

        for (int i = 0; i < callback_write; i++) {
            if (callbacks[i].arg_names) {
                commands_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
            } else {
                commands_printf(callbacks[i].command);
            }

            if (callbacks[i].help) {
                commands_printf("  %s", callbacks[i].help);
            } else {
                commands_printf("  There is no help available for this command.");
            }
        }

        commands_printf(" ");
    } else {
        bool found = false;
        for (int i = 0; i < callback_write; i++) {
            if (strcmp(argv[0], callbacks[i].command) == 0) {
                callbacks[i].cbf(argc, (const char**)argv);
                found = true;
                break;
            }
        }

        if (!found) {
            commands_printf("Invalid command: %s\n"
                            "type help to list all available commands\n", argv[0]);
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
void terminal_register_command_callback(
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
