#include "ctrl_step.hpp"
#include "communication.hpp"


CtrlStepMotor::CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
    nodeID(_id), hcan(_hcan), inverseDirection(_inverse), reduction(_reduction),
    angleLimitMin(_angleLimitMin), angleLimitMax(_angleLimitMax)
{

}


bool CtrlStepMotor::SetEnable(bool _enable)
{
    state = _enable ? FINISH : STOP;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x01;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::DoCalibration()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x02;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x03;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x04;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x05;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need ACK

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x07;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_pos;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    b = (unsigned char*) &_vel;
    for (int i = 4; i < 8; i++)
        canBuf[i] = *(b + i - 4);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x11;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    auto* b = (unsigned char*) &_id;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x13;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAcceleration(float _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 0; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::ApplyPositionAsHome()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x15;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x16;
    txHeader.StdId = nodeID << 7 | mode;

    // Int to Bytes
    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x1B;
    txHeader.StdId = nodeID << 7 | mode;

    uint32_t val = _enable ? 1 : 0;
    auto* b = (unsigned char*) &val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::Reboot()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x7f;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::EraseConfigs()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x7e;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetAngle(float _angle)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionSetPoint(stepMotorCnt);
}


void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);
}


void CtrlStepMotor::UpdateAngle()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x23;
    txHeader.StdId = nodeID << 7 | mode;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;

    float tmp = _pos / (float) reduction * 360;
    angle = inverseDirection ? -tmp : tmp;
}

uint32_t CtrlStepMotor::FeedbackAge() const
{
    // Read the ISR-owned timestamp BEFORE the clock. A CAN interrupt between
    // reading 'now' and the timestamp can otherwise turn a fresh reply into
    // an unsigned age near UINT32_MAX, latching a false timeout.
    const uint32_t receivedAt = lastFeedbackTick;
    return HAL_GetTick() - receivedAt;
}

void CtrlStepMotor::UpdateStatus(uint8_t _state, uint8_t _faults, bool _supported)
{
    driverState = _state;
    faultFlags = _faults;
    statusSupported = _supported;
    lastFeedbackTick = HAL_GetTick();
    ++feedbackCount;
    feedbackReceived = true;
}

bool CtrlStepMotor::ClearFaults()
{
    CAN_TxHeaderTypeDef header = baseHeader;
    header.StdId = nodeID << 7 | 0x08;
    uint8_t data[8] = {};
    return CanSendMessage(get_can_ctx(hcan), data, &header);
}

bool CtrlStepMotor::ReadControlTiming()
{
    const uint32_t previous = timingReplyCount;
    CAN_TxHeaderTypeDef header = baseHeader;
    header.StdId = nodeID << 7 | 0x25;
    uint8_t data[8] = {};
    if (!CanSendMessage(get_can_ctx(hcan), data, &header)) return false;
    const uint32_t started = HAL_GetTick();
    while (timingReplyCount == previous && HAL_GetTick() - started < 50) osDelay(1);
    return timingReplyCount != previous;
}

void CtrlStepMotor::UpdateControlTiming(uint16_t last, uint16_t peak, uint32_t overruns)
{
    controlLastCycles = last;
    controlMaxCycles = peak;
    controlOverruns = overruns;
    ++timingReplyCount;
    // Timing is diagnostic only; it must not mask a missing position/status reply.
}

void CtrlStepMotor::Hold()
{
    CAN_TxHeaderTypeDef header = baseHeader;
    header.StdId = nodeID << 7 | 0x09;
    uint8_t data[8] = {};
    CanSendMessage(get_can_ctx(hcan), data, &header);
}

void CtrlStepMotor::UpdateCurrentCommand(float amps, uint8_t faults)
{
    currentCommandA = amps; // Motor direction, NOT ADC-measured current.
    currentFaults = faults;
    currentTick = HAL_GetTick();
    ++currentReplyCount;
}

void CtrlStepMotor::UpdateVelocity(float turnsPerSecond, uint8_t faults)
{
    velocityTurnsPerSecond = turnsPerSecond;
    velocityFaults = faults;
    velocityTick = HAL_GetTick();
    ++velocityReplyCount;
    // Neither diagnostic reply refreshes position feedback / the safety watchdog.
}

bool CtrlStepMotor::ReadTelemetry(Telemetry& sample)
{
    if (nodeID < 1 || nodeID > 6) return false;
    const uint32_t previousPosition = feedbackCount;
    const uint32_t previousCurrent = currentReplyCount;
    const uint32_t previousVelocity = velocityReplyCount;
    CAN_TxHeaderTypeDef header = baseHeader;
    uint8_t data[8] = {};
    for (uint8_t command : {0x21, 0x22, 0x23})
    {
        header.StdId = nodeID << 7 | command;
        if (!CanSendMessage(get_can_ctx(hcan), data, &header)) return false;
    }
    const uint32_t started = HAL_GetTick();
    while (feedbackCount == previousPosition || currentReplyCount == previousCurrent ||
           velocityReplyCount == previousVelocity)
    {
        if (HAL_GetTick() - started >= 50) return false;
        osDelay(1);
    }
    // Copy only; never send, wait or format text with interrupts masked.
    taskENTER_CRITICAL();
    sample.positionDeg = angle;
    sample.velocityDegS = velocityTurnsPerSecond * (inverseDirection ? -360.0f : 360.0f) / reduction;
    sample.currentCommandA = currentCommandA;
    sample.positionTick = lastFeedbackTick;
    sample.velocityTick = velocityTick;
    sample.currentTick = currentTick;
    sample.driverState = driverState;
    sample.faultFlags = faultFlags | currentFaults | velocityFaults;
    sample.statusSupported = statusSupported;
    taskEXIT_CRITICAL();
    return true;
}


void CtrlStepMotor::SetDceKp(int32_t _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x17;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKv(int32_t _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x18;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKi(int32_t _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x19;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::SetDceKd(int32_t _val)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x1A;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = 1; // Need save to EEPROM or not

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
