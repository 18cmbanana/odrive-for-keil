/*

    */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include "datatypes.h"

// Functions
void terminal_process_string(char *str);
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv));

#endif /* TERMINAL_H_ */
