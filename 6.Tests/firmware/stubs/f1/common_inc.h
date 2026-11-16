#pragma once
#include "Motor/motor.h"
#include "can.h"
struct EncoderCalibrator { bool isTriggered = false; };
void CAN_Send(CAN_TxHeaderTypeDef*, uint8_t*);
void OnCanCmd(uint8_t, uint8_t*, uint32_t);
