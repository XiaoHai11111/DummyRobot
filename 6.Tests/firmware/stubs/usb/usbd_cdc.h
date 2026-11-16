#pragma once
#include <cstdint>
constexpr uint8_t CDC_OUT_EP = 1, CDC_IN_EP = 0x81, ODRIVE_OUT_EP = 3, ODRIVE_IN_EP = 0x83;
constexpr uint8_t USBD_OK = 0, USBD_BUSY = 1;
struct USBD_HandleTypeDef { uint8_t dev_state = 3; };
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef*, uint8_t);
