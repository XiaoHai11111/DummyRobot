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


bool CtrlStepMotor::SetCurrentFeedForward(float _val, uint16_t _seq, uint16_t _leaseMs)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x0A;

    auto* b = reinterpret_cast<unsigned char*>(&_val);
    for (int i = 0; i < 4; i++) canBuf[i] = *(b + i);
    b = reinterpret_cast<unsigned char*>(&_seq);
    for (int i = 0; i < 2; i++) canBuf[4 + i] = *(b + i);
    b = reinterpret_cast<unsigned char*>(&_leaseMs);
    for (int i = 0; i < 2; i++) canBuf[6 + i] = *(b + i);

    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::SetDragCurrent(float _val, uint16_t _seq, uint16_t _leaseMs)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x0B;

    auto* b = reinterpret_cast<unsigned char*>(&_val);
    for (int i = 0; i < 4; i++) canBuf[i] = *(b + i);
    b = reinterpret_cast<unsigned char*>(&_seq);
    for (int i = 0; i < 2; i++) canBuf[4 + i] = *(b + i);
    b = reinterpret_cast<unsigned char*>(&_leaseMs);
    for (int i = 0; i < 2; i++) canBuf[6 + i] = *(b + i);

    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


static void PutInt16LittleEndian(uint8_t* destination, int16_t value)
{
    const uint16_t bits = static_cast<uint16_t>(value);
    destination[0] = static_cast<uint8_t>(bits);
    destination[1] = static_cast<uint8_t>(bits >> 8);
}


bool CtrlStepMotor::SetExtendedDragCurrent(int16_t baseMa, uint16_t totalLimitMa,
                                           uint16_t seq, uint16_t leaseMs,
                                           bool disablePositiveMotor,
                                           bool disableNegativeMotor)
{
    if ((nodeID != 4 && nodeID != 5) || seq == 0 || totalLimitMa < 1 ||
        totalLimitMa > 3300 || leaseMs < 1 || leaseMs > 0x3FFF)
        return false;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x0F;
    PutInt16LittleEndian(canBuf, baseMa);
    PutInt16LittleEndian(canBuf + 2, static_cast<int16_t>(totalLimitMa));
    PutInt16LittleEndian(canBuf + 4, static_cast<int16_t>(seq));
    uint16_t leaseFlags = leaseMs;
    if (disablePositiveMotor) leaseFlags |= 0x4000U;
    if (disableNegativeMotor) leaseFlags |= 0x8000U;
    PutInt16LittleEndian(canBuf + 6, static_cast<int16_t>(leaseFlags));
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::SetHarmonicCompensation(const int16_t valuesMa[4])
{
    return SetHarmonicCompensation(valuesMa, false);
}


bool CtrlStepMotor::SetHarmonicCompensation(const int16_t valuesMa[4], bool storeToEeprom)
{
    if (nodeID != 4 && nodeID != 5) return false;
    for (unsigned i = 0; i < 4; ++i)
        if (valuesMa[i] < -500 || valuesMa[i] > 500) return false;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | (storeToEeprom ? 0x10 : 0x0D);
    for (unsigned i = 0; i < 4; ++i)
        PutInt16LittleEndian(canBuf + i * 2, valuesMa[i]);
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::RequestCompensationParams()
{
    bool sent = true;
    for (uint8_t page = 0; page < 4; ++page)
    {
        uint8_t canBuf[8] = {};
        canBuf[0] = page;
        CAN_TxHeaderTypeDef txHeader = baseHeader;
        txHeader.StdId = nodeID << 7 | (0x29U + page);
        const bool requestSent = CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
        sent = requestSent && sent;
        if (compensationParamsPageMask != 0x0F)
        {
            osDelay(1);
        }
    }
    return sent;
}


bool CtrlStepMotor::QueryCompensationParams(uint32_t timeoutMs)
{
    compensationParamsPageMask = 0;
    bool sent = RequestCompensationParams();
    if (compensationParamsPageMask == 0x0F) return sent;
    const uint32_t started = HAL_GetTick();
    while (compensationParamsPageMask != 0x0F && HAL_GetTick() - started < timeoutMs)
        osDelay(1);
    return sent && compensationParamsPageMask == 0x0F;
}


void CtrlStepMotor::UpdateCompensationParams(uint8_t page, const uint8_t data[8])
{
    if (page > 3) return;
    uint32_t value32 = 0;
    switch (page)
    {
        case 0:
            memcpy(&value32, data, 4);
            compensationParams[0] = static_cast<int32_t>(value32);
            memcpy(&value32, data + 4, 4);
            compensationParams[1] = static_cast<int32_t>(value32);
            break;
        case 1:
            memcpy(&value32, data, 4);
            compensationParams[2] = static_cast<int32_t>(value32);
            memcpy(&value32, data + 4, 4);
            compensationParams[3] = static_cast<int32_t>(value32);
            break;
        case 2:
            memcpy(&value32, data, 4);
            compensationParams[4] = static_cast<int32_t>(value32);
            break;
        case 3:
            for (unsigned i = 0; i < 4; ++i)
                harmonicParams[i] = static_cast<int16_t>(
                    static_cast<uint16_t>(data[i * 2]) |
                    (static_cast<uint16_t>(data[i * 2 + 1]) << 8));
            break;
        default:
            break;
    }
    compensationParamsPageMask |= static_cast<uint8_t>(1U << page);
    ++compensationParamsReplyCount;
}


bool CtrlStepMotor::RefreshCompensationParams()
{
    return QueryCompensationParams();
}


bool CtrlStepMotor::SetWristCompensation(bool enable, uint8_t gainPercent,
                                         uint16_t maxAssistMa, uint16_t seq)
{
    if ((nodeID != 4 && nodeID != 5) || seq == 0 || gainPercent > 100 ||
        maxAssistMa > 500)
        return false;
    if (!enable && (gainPercent != 0 || maxAssistMa != 0)) return false;

    uint8_t canBuf[8] = {
        0xB4, 2, static_cast<uint8_t>(enable ? 1 : 0), gainPercent
    };
    PutInt16LittleEndian(canBuf + 4, static_cast<int16_t>(seq));
    PutInt16LittleEndian(canBuf + 6, static_cast<int16_t>(maxAssistMa));
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x0E;
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::RequestHarmonicCompensationReadback()
{
    if (nodeID != 4 && nodeID != 5) return false;
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x28;
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::RequestWristCompensationStatus()
{
    if (nodeID != 4 && nodeID != 5) return false;
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x27;
    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::QueryHarmonicCompensation(int16_t valuesMa[4], uint32_t timeoutMs)
{
    if (!valuesMa) return false;
    const uint32_t previous = harmonicReplyCount;
    if (!RequestHarmonicCompensationReadback()) return false;
    const uint32_t started = HAL_GetTick();
    while (harmonicReplyCount == previous)
    {
        if (HAL_GetTick() - started >= timeoutMs) return false;
        osDelay(1);
    }
    for (unsigned i = 0; i < 4; ++i) valuesMa[i] = harmonicReadbackMa[i];
    return true;
}


bool CtrlStepMotor::QueryWristCompensationStatus(WristCompensationStatus& status,
                                                 uint32_t timeoutMs)
{
    const uint32_t previous = wristCompensationReplyCount;
    if (!RequestWristCompensationStatus()) return false;
    const uint32_t started = HAL_GetTick();
    while (wristCompensationReplyCount == previous)
    {
        if (HAL_GetTick() - started >= timeoutMs) return false;
        osDelay(1);
    }
    status.flags = wristStatusFlags;
    status.direction = wristDirection;
    status.actualAssistMa = wristActualAssistMa;
    status.sequence = wristStatusSequence;
    return true;
}


void CtrlStepMotor::UpdateHarmonicCompensationReadback(const int16_t valuesMa[4])
{
    for (unsigned i = 0; i < 4; ++i) harmonicReadbackMa[i] = valuesMa[i];
    ++harmonicReplyCount;
}


void CtrlStepMotor::UpdateWristCompensationStatus(uint8_t flags, int8_t direction,
                                                  int16_t actualAssistMa, uint16_t sequence)
{
    wristStatusFlags = flags;
    wristDirection = direction;
    wristActualAssistMa = actualAssistMa;
    wristStatusSequence = sequence;
    ++wristCompensationReplyCount;
}


void CtrlStepMotor::RequestVelocityFeedback()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x22;
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void CtrlStepMotor::RequestPositionFeedback()
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x23;
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

uint32_t CtrlStepMotor::VelocityAge() const
{
    const uint32_t receivedAt = velocityTick;
    return HAL_GetTick() - receivedAt;
}


float CtrlStepMotor::GetVelocityDegS() const
{
    const float turnsPerSecond = velocityTurnsPerSecond;
    const float deg = turnsPerSecond * 360.0f / static_cast<float>(reduction);
    return inverseDirection ? -deg : deg;
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

bool CtrlStepMotor::Hold()
{
    CAN_TxHeaderTypeDef header = baseHeader;
    header.StdId = nodeID << 7 | 0x09;
    uint8_t data[8] = {};
    return CanSendMessage(get_can_ctx(hcan), data, &header);
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
    canBuf[4] = 1;

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
    canBuf[4] = 1;

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
    canBuf[4] = 1;

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
    canBuf[4] = 1;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


static bool SendLocalCompensationParam(CAN_HandleTypeDef* hcan,
                                       const CAN_TxHeaderTypeDef& baseHeader,
                                       uint8_t nodeID, uint8_t index,
                                       int32_t value, bool storeToEeprom)
{
    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    txHeader.StdId = nodeID << 7 | 0x0C;

    auto* b = reinterpret_cast<unsigned char*>(&value);
    for (int i = 0; i < 4; i++) canBuf[i] = *(b + i);
    canBuf[4] = index;
    canBuf[5] = storeToEeprom ? 1 : 0;

    return CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


bool CtrlStepMotor::SetLocalCompensation(int32_t damping, int32_t coulomb,
                                         int32_t viscous, int32_t deadzone,
                                         int32_t limit, bool storeToEeprom)
{
    bool sent = true;
    sent = SendLocalCompensationParam(
               hcan, baseHeader, nodeID, 0, damping, storeToEeprom) && sent;
    sent = SendLocalCompensationParam(
               hcan, baseHeader, nodeID, 1, coulomb, storeToEeprom) && sent;
    sent = SendLocalCompensationParam(
               hcan, baseHeader, nodeID, 2, viscous, storeToEeprom) && sent;
    sent = SendLocalCompensationParam(
               hcan, baseHeader, nodeID, 3, deadzone, storeToEeprom) && sent;
    sent = SendLocalCompensationParam(
               hcan, baseHeader, nodeID, 4, limit, storeToEeprom) && sent;
    return sent;
}


namespace
{
bool SetOneLocalCompensationParam(CtrlStepMotor* motor, CAN_HandleTypeDef* hcan,
                                  const CAN_TxHeaderTypeDef& baseHeader,
                                  uint8_t index, int32_t value, bool storeToEeprom)
{
    return SendLocalCompensationParam(hcan, baseHeader, motor->nodeID,
                                      index, value, storeToEeprom);
}
}


bool CtrlStepMotor::SetLocalDamping(int32_t value, bool storeToEeprom)
{
    return SetOneLocalCompensationParam(this, hcan, baseHeader, 0, value, storeToEeprom);
}


bool CtrlStepMotor::SetLocalCoulomb(int32_t value, bool storeToEeprom)
{
    return SetOneLocalCompensationParam(this, hcan, baseHeader, 1, value, storeToEeprom);
}


bool CtrlStepMotor::SetLocalViscous(int32_t value, bool storeToEeprom)
{
    return SetOneLocalCompensationParam(this, hcan, baseHeader, 2, value, storeToEeprom);
}


bool CtrlStepMotor::SetLocalDeadzone(int32_t value, bool storeToEeprom)
{
    return SetOneLocalCompensationParam(this, hcan, baseHeader, 3, value, storeToEeprom);
}


bool CtrlStepMotor::SetLocalLimit(int32_t value, bool storeToEeprom)
{
    return SetOneLocalCompensationParam(this, hcan, baseHeader, 4, value, storeToEeprom);
}


bool CtrlStepMotor::SetStoredHarmonics(int32_t eSin, int32_t eCos,
                                       int32_t oSin, int32_t oCos)
{
    if (eSin < -500 || eSin > 500 || eCos < -500 || eCos > 500 ||
        oSin < -500 || oSin > 500 || oCos < -500 || oCos > 500)
        return false;
    const int16_t values[4] = {
        static_cast<int16_t>(eSin), static_cast<int16_t>(eCos),
        static_cast<int16_t>(oSin), static_cast<int16_t>(oCos)};
    return SetHarmonicCompensation(values, true);
}
