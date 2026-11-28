#ifndef CONFIGURATIONS_H
#define CONFIGURATIONS_H

#ifdef __cplusplus
extern "C" {
#endif
/*---------------------------- C Scope ---------------------------*/
#include <stdbool.h>
#include "stdint-gcc.h"

typedef enum configStatus_t
{
    CONFIG_RESTORE = 0,
    CONFIG_OK,
    CONFIG_COMMIT
} configStatus_t;

#define BOARD_CONFIG_VERSION 3U


typedef struct Config_t
{
    configStatus_t configStatus;
    uint32_t canNodeId;
    int32_t encoderHomeOffset;
    uint32_t defaultMode;
    int32_t currentLimit;
    int32_t velocityLimit;
    int32_t velocityAcc;
    int32_t calibrationCurrent;
    int32_t dce_kp;
    int32_t dce_kv;
    int32_t dce_ki;
    int32_t dce_kd;
    bool enableMotorOnBoot;
    bool enableStallProtect;
    int32_t localDamping_mA_per_motorRevS;
    int32_t localCoulomb_mA;
    int32_t localViscous_mA_per_motorRevS;
    int32_t localDeadzone_motorRevS_x1000;
    int32_t localCompLimit_mA;
    int16_t harmonicESin_mA;
    int16_t harmonicECos_mA;
    int16_t harmonicOSin_mA;
    int16_t harmonicOCos_mA;
    uint32_t version;
} BoardConfig_t;

extern BoardConfig_t boardConfig;


#ifdef __cplusplus
}
/*---------------------------- C++ Scope ---------------------------*/

#include <Platform/Memory/eeprom_interface.h>
#include "Motor/motor.h"


#endif
#endif
