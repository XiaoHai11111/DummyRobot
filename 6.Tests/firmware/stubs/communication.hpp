#pragma once
#include "can.h"
#include "fibre/protocol.hpp"
#include "ascii_processor.hpp"
extern CAN_HandleTypeDef hcan1;
struct CAN_context {
    CAN_HandleTypeDef* handle;
    uint32_t received_msg_cnt = 0, tx_msg_cnt = 0, unexpected_errors = 0;
    uint32_t TxMailboxAbortCallbackCnt = 0, last_error = 0, bus_error_status = 0;
};
CAN_context* get_can_ctx(CAN_HandleTypeDef* handle);
bool CanSendMessage(CAN_context*, uint8_t*, CAN_TxHeaderTypeDef*);
void OnCanMessage(CAN_context*, CAN_RxHeaderTypeDef*, uint8_t*);
