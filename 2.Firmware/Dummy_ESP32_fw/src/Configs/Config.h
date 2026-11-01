#ifndef __CONFIG_H
#define __CONFIG_H

/*=========================
   Application configuration
 *=========================*/

#define CONFIG_SYSTEM_SAVE_FILE_PATH          "S:/SystemSave.json"
#define CONFIG_SYSTEM_LANGUAGE_DEFAULT        {'e','n','-','G','B'}
#define CONFIG_SYSTEM_GMT_OFFSET_DEFAULT      8 // GMT+ 8


/*=========================
   Hardware Configuration
 *=========================*/

#define NULL_PIN                    PD0

/* LedRing */
#define LEDS_COUNT        12    //彩灯数目
#define LEDS_PIN	        13    //ESP32控制ws2812的引脚
#define CHANNEL		        8    //控制通道，最多8路

#endif
