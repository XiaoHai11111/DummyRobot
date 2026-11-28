#ifndef CTRL_STEP_FW_CAN_COMMAND_FILTER_H
#define CTRL_STEP_FW_CAN_COMMAND_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * New per-node configuration and extended drag commands stay address-only.
 * Existing broadcast shutdown and safety commands remain accepted.
 */
static inline bool CanRxCommandAccepted(uint8_t localNodeId,
                                        bool isStandardFrame,
                                        bool isDataFrame,
                                        uint8_t targetNodeId,
                                        uint8_t command)
{
    if (!isStandardFrame || !isDataFrame) return false;
    if (targetNodeId == localNodeId) return true;
    if (targetNodeId != 0U) return false;

    if (command >= 0x0D && command <= 0x10) return false;
    if (command >= 0x27 && command <= 0x2C) return false;
    return true;
}

#ifdef __cplusplus
}
#endif

#endif
