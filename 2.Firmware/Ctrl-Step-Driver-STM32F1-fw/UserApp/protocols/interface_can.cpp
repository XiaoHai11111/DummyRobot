#include "common_inc.h"
#include "configurations.h"
#include <can.h>
#include <cmath>
#include <cstring>
#include "Motor/control_timing.h"


extern Motor motor;
extern EncoderCalibrator encoderCalibrator;

CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0x00,
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };


template<typename T>
static T ReadValue(const uint8_t* data)
{
    T value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static void SendFeedback(uint8_t command, float value)
{
    uint8_t reply[8] = {};
    memcpy(reply, &value, sizeof(value));
    reply[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
    reply[5] = static_cast<uint8_t>(motor.controller->state);
    reply[6] = motor.controller->GetFaultFlags();
    reply[7] = 0xA1; // Extended status v1; old F4 readers still use bytes 0..4.
    txHeader.StdId = (boardConfig.canNodeId << 7) | command;
    CAN_Send(&txHeader, reply);
}

static bool FiniteFloat(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}


void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len)
{
    float tmpF;
    int32_t tmpI;

    // Reject short frames before touching payload; inquiries/hold/clear have no arguments.
    if (((_cmd >= 0x01 && _cmd <= 0x07 && _cmd != 0x02) && _len < 4) ||
        ((_cmd == 0x06 || _cmd == 0x07 || _cmd == 0x0A || _cmd == 0x0B) && _len < 8) ||
        ((_cmd >= 0x11 && _cmd <= 0x1B) && _len < 5))
        return;
    if ((_cmd >= 0x03 && _cmd <= 0x07) || _cmd == 0x0A || _cmd == 0x0B ||
        (_cmd >= 0x12 && _cmd <= 0x14))
    {
        memcpy(&tmpF, _data, sizeof(tmpF));
        if (!FiniteFloat(tmpF) || tmpF < -40000.0f || tmpF > 40000.0f)
            return;
        if ((_cmd == 0x0A || _cmd == 0x0B) &&
            fabsf(tmpF) * 1000.0f > motor.config.ctrlParams.feedForward.maxCurrent)
            return;
        if ((_cmd == 0x12 && (tmpF <= 0 || tmpF > 3.3f)) ||
            ((_cmd == 0x13 || _cmd == 0x14) && tmpF <= 0))
            return;
    }
    if (_cmd == 0x06 || _cmd == 0x07)
    {
        memcpy(&tmpF, _data + 4, sizeof(tmpF));
        if (!FiniteFloat(tmpF) || tmpF < 0 ||
            (_cmd == 0x06 && (tmpF == 0 || tmpF > 3600)) ||
            (_cmd == 0x07 && tmpF > 100))
            return;
    }

    switch (_cmd)
    {
        // 0x00~0x0F No Memory CMDs
        case 0x01:  // Enable Motor
            motor.controller->SetEnable(ReadValue<uint32_t>(_data) == 1);
            break;
        case 0x02:  // Do Calibration
            encoderCalibrator.isTriggered = true;
            break;
        case 0x03:  // Set Current SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
            motor.controller->SetCurrentSetPoint((int32_t) (ReadValue<float>(_data) * 1000));
            break;
        case 0x04:  // Set Velocity SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
            }
            motor.controller->SetVelocitySetPoint(
                (int32_t) (ReadValue<float>(_data) *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            break;
        case 0x05:  // Set Position SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.controller->SetPositionSetPoint(
                (int32_t) (ReadValue<float>(_data) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            if (_len >= 5 && _data[4])
                SendFeedback(0x23, motor.controller->GetPosition());
            break;
        case 0x06:  // Set Position with Time
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            motor.controller->SetPositionSetPointWithTime(
                (int32_t) (ReadValue<float>(_data) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
                ReadValue<float>(_data + 4));
            SendFeedback(0x23, motor.controller->GetPosition());
            break;
        case 0x07:  // Set Position with Velocity-Limit
        {
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            memcpy(&tmpF, _data + 4, sizeof(tmpF));
            if (!FiniteFloat(tmpF) || tmpF < 0 || tmpF > 100)
                break;
            // Never override the configured motor limit with a host target.
            int32_t velocity = static_cast<int32_t>(tmpF * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            if (velocity > boardConfig.velocityLimit) velocity = boardConfig.velocityLimit;
            motor.config.motionParams.ratedVelocity = velocity > 0 ? velocity : 1;
            motor.controller->SetPositionSetPoint(
                (int32_t) (ReadValue<float>(_data) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            SendFeedback(0x23, motor.controller->GetPosition());
        }
            break;

        case 0x08: // Explicit clear: reset in the control tick, leave STOP, discard old goals.
            motor.controller->ClearStallFlag();
            break;
        case 0x09: // Hold the measured position (does not clear a fault).
            motor.controller->RequestHold();
            break;
        case 0x0A: // Set RAM-only current feed-forward, no mode switch.
            motor.controller->RequestCurrentFeedForward(
                ReadValue<float>(_data), ReadValue<uint16_t>(_data + 4),
                ReadValue<uint16_t>(_data + 6));
            break;
        case 0x0B: // Set leased drag current and enter current mode.
            motor.controller->RequestDragCurrent(
                ReadValue<float>(_data), ReadValue<uint16_t>(_data + 4),
                ReadValue<uint16_t>(_data + 6));
            break;

            // 0x10~0x1F CMDs with Memory
        case 0x11:  // Set Node-ID and Store to EEPROM
            boardConfig.canNodeId = ReadValue<uint32_t>(_data);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x12:  // Set Current-Limit and Store to EEPROM
            motor.config.motionParams.ratedCurrent = (int32_t) (ReadValue<float>(_data) * 1000);
            boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x13:  // Set Velocity-Limit and Store to EEPROM
            motor.config.motionParams.ratedVelocity =
                (int32_t) (ReadValue<float>(_data) *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x14:  // Set Acceleration （and Store to EEPROM）
            tmpF = ReadValue<float>(_data) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

            motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
            motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
            motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
            boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x15:  // Apply Home-Position and Store to EEPROM
            motor.controller->ApplyPosAsHomeOffset();
            boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                            motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x16:  // Set Auto-Enable and Store to EEPROM
            boardConfig.enableMotorOnBoot = (ReadValue<uint32_t>(_data) == 1);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x17:  // Set DCE Kp
            motor.config.ctrlParams.dce.kp = ReadValue<int32_t>(_data);
            boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x18:  // Set DCE Kv
            motor.config.ctrlParams.dce.kv = ReadValue<int32_t>(_data);
            boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x19:  // Set DCE Ki
            motor.config.ctrlParams.dce.ki = ReadValue<int32_t>(_data);
            boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1A:  // Set DCE Kd
            motor.config.ctrlParams.dce.kd = ReadValue<int32_t>(_data);
            boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1B:  // Set Enable Stall-Protect
            motor.config.ctrlParams.stallProtectSwitch = (ReadValue<uint32_t>(_data) == 1);
            boardConfig.enableStallProtect = motor.config.ctrlParams.stallProtectSwitch;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;


            // 0x20~0x2F Inquiry CMDs
        case 0x21:
            SendFeedback(0x21, motor.controller->GetFocCurrent());
            break;
        case 0x22:
            SendFeedback(0x22, motor.controller->GetVelocity());
            break;
        case 0x23:
            SendFeedback(0x23, motor.controller->GetPosition());
            break;
        case 0x24: // Get Offset
        {
            uint8_t reply[8] = {};
            tmpI = motor.config.motionParams.encoderHomeOffset;
            memcpy(reply, &tmpI, sizeof(tmpI));
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x24;
            CAN_Send(&txHeader, reply);
        }
            break;

        case 0x25: // Read-only control-loop timing: uint16 last/max cycles, uint32 overruns.
        {
            uint8_t reply[8] = {};
            const uint32_t last = controlTimingStats.lastCycles;
            const uint32_t peak = controlTimingStats.maxCycles;
            const uint16_t last16 = last > UINT16_MAX ? UINT16_MAX : last;
            const uint16_t peak16 = peak > UINT16_MAX ? UINT16_MAX : peak;
            const uint32_t overruns = controlTimingStats.overrunCount;
            memcpy(reply, &last16, 2);
            memcpy(reply + 2, &peak16, 2);
            memcpy(reply + 4, &overruns, 4);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x25;
            CAN_Send(&txHeader, reply);
        }
            break;

        case 0x26: // Read-only feed-forward diagnostics.
        {
            uint8_t reply[8] = {};
            const int16_t feedback = static_cast<int16_t>(
                motor.controller->GetFeedbackCurrent_mA());
            const int16_t feedForward = static_cast<int16_t>(
                motor.controller->GetFeedForwardCurrent_mA());
            const int16_t total = static_cast<int16_t>(
                motor.controller->GetTotalCurrent_mA());
            uint8_t flags = 0;
            if (motor.controller->IsFeedForwardLimited()) flags |= 1;
            if (motor.controller->GetFeedForwardCurrent_mA() != 0) flags |= 2;
            memcpy(reply, &feedback, 2);
            memcpy(reply + 2, &feedForward, 2);
            memcpy(reply + 4, &total, 2);
            reply[6] = flags;
            reply[7] = 0xA2;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x26;
            CAN_Send(&txHeader, reply);
        }
            break;

        case 0x7e:  // Erase Configs
            boardConfig.configStatus = CONFIG_RESTORE;
            break;
        case 0x7f:  // Reboot
            HAL_NVIC_SystemReset();
            break;
        default:
            break;
    }

}

