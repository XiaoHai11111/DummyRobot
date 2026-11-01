#include "HAL/HAL.h"

void HAL::Init()
{
    HAL::Serial_Init();
    HAL::LedRing_Init();
    HAL::BT_Init();
}

void HAL::Update()
{
    HAL::BT_Update();
    
    HAL::Serial_Update();
    
    HAL::LedRing_Update();
    delay(20);
}
