#include <Arduino.h>
#include "HAL.h"
#include "Configs/Config.h"
#include <Freenove_WS2812_Lib_for_ESP32.h>

static Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);

void HAL::LedRing_Init()
{
    strip.begin();
    strip.setBrightness(8);
}

void HAL::LedRing_Update()
{
    for (int j = 0; j < 255; j += 2) 
    {
        for (int i = 0; i < LEDS_COUNT; i++) 
        {
            uint8_t hue = (i * 256 / LEDS_COUNT + j) & 255;
            uint32_t color = strip.Wheel(hue);
            strip.setLedColorData(i, color);
        }
        strip.show();
        delay(5);
    }  
}