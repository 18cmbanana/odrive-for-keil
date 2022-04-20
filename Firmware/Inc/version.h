/**
  ******************************************************************************
  * @file           : version.h
  * @brief          : This file implements printf functionality
  ******************************************************************************
*/

#define ODRIVE_FW_VERSION_MAJOR   0
#define ODRIVE_FW_VERSION_MINOR   3
#define ODRIVE_FW_VERSION_PATCH   6

#define HW_NAME					"3.4b"

#if defined ARM_MDK
#define __BUILD__					" MDK "
#elif defined ARM_IAR
#define __BUILD__					" IAR "
#else
#define __BUILD__					" GCC "
#endif

#define __BY__   " by "
#define __AT__   " at "

#define ARM_APP_NEW

#if 1
#if defined ARM_APP_NEW
#define __FOR__					"app2.0"
#elif defined ARM_BOOT_NEW
#define __FOR__					"boot1.0"
#else
#define __FOR__					"none1.0"
#endif
#define FW_CHANGELOG_TIME      "23/05/2018 11:22:59"
/*
备注:
修改:
增加:
删减:
修复:
优化:
计划:
*/
#elif 1
#else
#endif
