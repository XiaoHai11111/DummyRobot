#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>
#include <Arduino.h>
#include "HAL_Def.h"
#include "FreeRTOS.h"

namespace HAL
{
    void Init();
    void Update();

    /* Serial */
    void Serial_Init();
    void Serial_Update();
    void Serial1_Update();
    void Serial1_GetDummyStatus();

    /* Bluetooth */
    void BT_Init();
    void BT_Update();
    bool BT_IsConnect();
    // void BT_print();

    /* LedRing */
    void LedRing_Init();
    void LedRing_Update();
}

#endif
