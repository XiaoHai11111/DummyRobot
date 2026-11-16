#include "common_inc.h"


extern DummyRobot dummy;

void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    // Common CAN message callback, uses ID 32~0x7FF.
    if (canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
        uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg

        /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
        switch (cmd)
        {
            case 0x23:
            {
                if (rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA ||
                    id < 1 || id > 6 || rxHeader->DLC < 5)
                    return;
                float position;
                uint32_t bits;
                memcpy(&position, data, sizeof(position));
                memcpy(&bits, data, sizeof(bits));
                if ((bits & 0x7F800000U) == 0x7F800000U || position < -40000 || position > 40000)
                    return;
                const bool supported = rxHeader->DLC == 8 && data[7] == 0xA1 && data[5] <= 6;
                dummy.motorJ[id]->UpdateAngleCallback(position, data[4] == 1);
                dummy.motorJ[id]->UpdateStatus(supported ? data[5] : 0, supported ? data[6] : 0, supported);
                dummy.UpdateJointAnglesCallback();
                break;
            }
            case 0x25:
            {
                if (rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA ||
                    id < 1 || id > 6 || rxHeader->DLC != 8) return;
                uint16_t last, peak;
                uint32_t overruns;
                memcpy(&last, data, 2);
                memcpy(&peak, data + 2, 2);
                memcpy(&overruns, data + 4, 4);
                dummy.motorJ[id]->UpdateControlTiming(last, peak, overruns);
                break;
            }
            default:
                break;
        }

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}
