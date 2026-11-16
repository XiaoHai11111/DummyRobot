#pragma once
#include "usbd_cdc.h"
constexpr unsigned USB_TX_DATA_SIZE = 64;
uint8_t CDC_Transmit_FS(uint8_t*, uint16_t, uint8_t);
