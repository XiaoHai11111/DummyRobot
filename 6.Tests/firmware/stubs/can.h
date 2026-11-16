#pragma once
#include <cstdint>
#define CAN1 reinterpret_cast<void*>(1)
#define CAN2 reinterpret_cast<void*>(2)
constexpr unsigned CAN_ID_STD = 0, CAN_RTR_DATA = 0, DISABLE = 0;
struct CAN_HandleTypeDef { void* Instance = CAN1; };
struct CAN_TxHeaderTypeDef { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; };
using CAN_RxHeaderTypeDef = CAN_TxHeaderTypeDef;
uint32_t HAL_GetTick();
void HAL_NVIC_SystemReset();
