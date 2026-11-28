#include "common_inc.h"


extern DummyRobot dummy;

static int16_t ReadInt16LittleEndian(const uint8_t* data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                (static_cast<uint16_t>(data[1]) << 8));
}

void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    // Common CAN message callback, uses ID 32~0x7FF.
    if (canCtx && canCtx->handle && canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
        uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg

        /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
        switch (cmd)
        {
            case 0x21: // Signed controller current command, amperes (not measured current).
            case 0x22: // Encoder-estimated motor velocity, turns/s.
            {
                if (rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA ||
                    id < 1 || id > 6 || rxHeader->DLC != 8 || data[7] != 0xA1 || data[5] > 6)
                    return;
                float value;
                uint32_t bits;
                memcpy(&value, data, sizeof(value));
                memcpy(&bits, data, sizeof(bits));
                const float limit = cmd == 0x21 ? 3.3f : 40000.0f;
                if ((bits & 0x7F800000U) == 0x7F800000U || value < -limit || value > limit)
                    return;
                if (cmd == 0x21) dummy.motorJ[id]->UpdateCurrentCommand(value, data[6]);
                else dummy.motorJ[id]->UpdateVelocity(value, data[6]);
                break;
            }
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
            case 0x28:
            {
                if (rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA ||
                    (id != 4 && id != 5) || rxHeader->DLC != 8)
                    return;
                int16_t values[4]{};
                for (unsigned i = 0; i < 4; ++i)
                {
                    values[i] = ReadInt16LittleEndian(data + i * 2);
                    if (values[i] < -500 || values[i] > 500) return;
                }
                dummy.UpdateWristHarmonicReadback(id, values);
                dummy.motorJ[id]->UpdateHarmonicCompensationReadback(values);
                break;
            }
            case 0x29:
            case 0x2A:
            case 0x2B:
            case 0x2C:
            {
                if (rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA ||
                    id < 1 || id > 6 || rxHeader->DLC != 8)
                    return;
                dummy.motorJ[id]->UpdateCompensationParams(cmd - 0x29, data);
                break;
            }
            case 0x27:
            {
                if (rxHeader->IDE != CAN_ID_STD || rxHeader->RTR != CAN_RTR_DATA ||
                    (id != 4 && id != 5) || rxHeader->DLC != 8 ||
                    data[0] != 0xB4 || data[1] != 2 || (data[2] & ~3U) != 0)
                    return;
                const int8_t direction = static_cast<int8_t>(data[3]);
                const int16_t actualAssistMa = ReadInt16LittleEndian(data + 4);
                if (direction < -1 || direction > 1 ||
                    actualAssistMa < -3300 || actualAssistMa > 3300)
                    return;
                const uint16_t sequence = static_cast<uint16_t>(
                    static_cast<uint16_t>(data[6]) |
                    (static_cast<uint16_t>(data[7]) << 8));
                dummy.motorJ[id]->UpdateWristCompensationStatus(
                    data[2], direction, actualAssistMa, sequence);
                dummy.UpdateWristCompensationStatus(
                    id, data[2], direction, actualAssistMa, sequence);
                break;
            }
            default:
                break;
        }

    } else if (canCtx && canCtx->handle && canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}
