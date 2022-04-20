/*
    */

#ifndef COMM_H_
#define COMM_H_

#include "datatypes.h"

// Firmware version
#define FW_VERSION_MAJOR		3
#define FW_VERSION_MINOR		102

#define HW_NAME					"34b"

#if defined ARM_MDK
#define __BUILD__					" MDK "
#elif defined ARM_IAR
#define __BUILD__					" IAR "
#else
#define __BUILD__					" GCC "
#endif
#define ARM_APP_NEW
#if defined ARM_APP_NEW
#define __FOR__					"ODrive-fw-v0.3.6"
#elif defined ARM_BOOT_NEW
#define __FOR__					"boot1.0"
#else
#define __FOR__					"none1.0"
#endif

#define __BY__   " by "
#define __AT__   " at "

#define STM32_UUID					((uint32_t*)0x1FFF7A10)
#define STM32_UUID_8				((uint8_t*)0x1FFF7A10)

// Functions
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len));
void commands_send_packet(unsigned char *data, unsigned int len);
void commands_process_packet(unsigned char *data, unsigned int len);
void commands_printf(const char* format, ...);

#endif /* COMM_H_ */
