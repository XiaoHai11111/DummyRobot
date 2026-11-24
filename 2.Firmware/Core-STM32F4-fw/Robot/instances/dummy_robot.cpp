#include "communication.hpp"
#include "dummy_robot.h"
#include "motion_command.h"

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (fabsf(_joints.a[i]) > max)
        {
            max = fabsf(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true, 30, -75, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true, 30, 35, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, false, 30, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 30, -120, 120);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, -720, 720);
    hand = new DummyHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(0.114f, 0.035f, 0.146f, 0.123f, 0.052f, 0.151f);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    safetyStarted = HAL_GetTick();
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    if (!IsEnabled() || HasFault()) return;
    const uint32_t generation = commandHandler.GetGeneration();
    for (int j = 1; j <= 6; j++)
    {
        if (!IsEnabled() || HasFault() || recovering || generation != commandHandler.GetGeneration()) return;
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}


bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    return MoveJAtGeneration(_j1, _j2, _j3, _j4, _j5, _j6, commandHandler.GetGeneration());
}

bool DummyRobot::MoveJAtGeneration(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,
                                   uint32_t generation)
{
    if (!IsEnabled() || HasFault() || recovering) return false;
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (!MotionFinite(targetJointsTmp.a[j - 1]) ||
            targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        DOF6Kinematic::Joint6D_t speeds{};
        // Joint speed is deg/s; CAN velocity is motor rev/s. Avoid 0/0 at rest.
        const float time = maxAngle / jointSpeed;
        for (int j = 1; j <= 6; j++)
            speeds.a[j - 1] = maxAngle > 0.0001f ?
                fmaxf(0.01f, fabsf(deltaJoints.a[j - 1] * motorJ[j]->reduction / (360.0f * time))) : 0.01f;

        taskENTER_CRITICAL();
        if (!isEnabled || HasFault() || recovering || generation != commandHandler.GetGeneration())
        {
            taskEXIT_CRITICAL();
            return false;
        }
        dynamicJointSpeeds = speeds;
        motionTimeoutMs = static_cast<uint32_t>(fminf(600000.0f, fmaxf(30000.0f, time * 2000 + 5000)));
        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;
        taskEXIT_CRITICAL();

        return true;
    }

    return false;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    return MoveLAtGeneration(_x, _y, _z, _a, _b, _c, commandHandler.GetGeneration());
}

bool DummyRobot::MoveLAtGeneration(float _x, float _y, float _z, float _a, float _b, float _c,
                                   uint32_t generation)
{
    if (!IsEnabled() || HasFault() || !MotionFinite(_x) || !MotionFinite(_y) ||
        !MotionFinite(_z) || !MotionFinite(_a) || !MotionFinite(_b) || !MotionFinite(_c))
        return false;
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (!MotionFinite(ikSolves.config[i].a[j - 1]) ||
                ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJAtGeneration(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5], generation);
    }

    return false;
}

void DummyRobot::UpdateJointAngles()
{
    // Address nodes individually to reduce simultaneous reply contention.
    for (unsigned j = 1; j <= 6; ++j) motorJ[j]->UpdateAngle();
}


void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH &&
            fabsf(currentJoints.a[i - 1] - targetJoints.a[i - 1]) <= 0.5f)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}


void DummyRobot::SetJointSpeed(float _speed)
{
    if (!MotionFinite(_speed) || _speed <= 0) return;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    if (!MotionFinite(_acc) || _acc <= 0) return;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}


void DummyRobot::  CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    SetEnable(true);
    if (!IsEnabled()) return;
    Resting();
    if (!IsEnabled() || HasFault()) return;
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    WaitForMotion();

    SetJointSpeed(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    WaitForMotion();

    SetJointSpeed(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    if (_enable)
    {
        if (recovering) return;
        CheckSafety();
        for (int j = 1; j <= 6; ++j)
            if (!motorJ[j]->feedbackReceived || !motorJ[j]->statusSupported ||
                motorJ[j]->FeedbackAge() > 250)
                return;
        if (HasFault()) return;
        if (isEnabled) return;
        commandHandler.ClearFifo();
        const uint32_t generation = commandHandler.GetGeneration();
        if (!motorJ[ALL]->SetEnable(true)) return;
        taskENTER_CRITICAL();
        if (generation != commandHandler.GetGeneration() || HasFault() || recovering)
        {
            taskEXIT_CRITICAL();
            if (holdOnStop) motorJ[ALL]->Hold();
            else motorJ[ALL]->SetEnable(false);
            return;
        }
        targetJoints = currentJoints;
        dynamicJointSpeeds = DOF6Kinematic::Joint6D_t(0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f);
        holdOnStop = false;
        enabledAt = HAL_GetTick();
        isEnabled = true;
        taskEXIT_CRITICAL();
    } else
    {
        isEnabled = false;
        holdOnStop = false;
        commandHandler.ClearFifo();
        motorJ[ALL]->SetEnable(false);
    }
}

bool DummyRobot::HasFault() const
{
    return GetFaultJointMask() != 0;
}

uint8_t DummyRobot::GetFaultJointMask() const
{
    uint8_t mask = 0;
    for (unsigned j = 1; j <= 6; ++j)
        if (jointFaults[j]) mask |= 1U << (j - 1);
    return mask;
}

void DummyRobot::CheckSafety()
{
    if (recovering) return;
    const uint32_t now = HAL_GetTick();
    for (unsigned j = 1; j <= 6; ++j)
    {
        const auto motor = motorJ[j];
        if (motor->feedbackReceived)
        {
            jointFaults[j] |= motor->faultFlags;
            if (!motor->statusSupported) jointFaults[j] |= 16;
            const uint32_t age = motor->FeedbackAge();
            if (age > 250)
            {
                if (!(jointFaults[j] & 8)) faultFeedbackAge[j] = age;
                jointFaults[j] |= 8;
            }
            if (isEnabled && now - enabledAt > 100 && motor->statusSupported && motor->driverState == 0)
                jointFaults[j] |= 64; // Driver stopped/rebooted despite an enabled F4.
        } else if (now - safetyStarted > 1000)
        {
            if (!(jointFaults[j] & 8)) faultFeedbackAge[j] = UINT32_MAX;
            jointFaults[j] |= 8;
        }
    }
    if (HasFault() && isEnabled) StopMotion();
}

void DummyRobot::StopMotion()
{
    isEnabled = false;
    holdOnStop = true;
    commandHandler.ClearFifo();
    targetJoints = currentJoints;
    // Each driver uses its own measured position, not a potentially stale F4 angle.
    motorJ[ALL]->Hold();
}

bool DummyRobot::ClearFaults()
{
    taskENTER_CRITICAL();
    if (recovering || isEnabled)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    recovering = true;
    taskEXIT_CRITICAL();
    SetEnable(false);
    bool sent = true;
    for (unsigned j = 1; j <= 6; ++j)
        sent = motorJ[j]->ClearFaults() && sent;
    const uint32_t start = HAL_GetTick();
    bool ready = false;
    while (sent && HAL_GetTick() - start < 500)
    {
        osDelay(10);
        ready = HAL_GetTick() - start >= 20;
        for (unsigned j = 1; j <= 6; ++j)
        {
            const auto motor = motorJ[j];
            ready = ready && motor->feedbackReceived && motor->statusSupported &&
                    static_cast<int32_t>(motor->lastFeedbackTick - start) >= 20 &&
                    motor->FeedbackAge() <= 250 &&
                    motor->faultFlags == 0 && motor->driverState == 0;
        }
        if (ready) break;
    }
    if (ready)
    {
        for (unsigned j = 1; j <= 6; ++j)
        {
            jointFaults[j] = 0;
            faultFeedbackAge[j] = 0;
        }
        targetJoints = currentJoints;
    }
    recovering = false;
    CheckSafety();
    return ready && !HasFault();
}

bool DummyRobot::WaitForMotion(uint32_t timeoutMs)
{
    if (!timeoutMs) timeoutMs = motionTimeoutMs;
    const uint32_t start = HAL_GetTick();
    const uint32_t generation = commandHandler.GetGeneration();
    // Give the control task time to transmit the new target and receive its status.
    osDelay(10);
    while (IsEnabled() && !HasFault() && IsMoving() && generation == commandHandler.GetGeneration())
    {
        if (HAL_GetTick() - start >= timeoutMs)
        {
            for (unsigned j = 1; j <= 6; ++j)
                if (!(jointsStateFlag & (1U << j))) jointFaults[j] |= 32;
            StopMotion();
            return false;
        }
        osDelay(5);
    }
    return IsEnabled() && !HasFault() && !IsMoving() && generation == commandHandler.GetGeneration();
}


void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_DRAG)
        return;

    const bool leavingDrag = commandMode == COMMAND_DRAG && _mode != COMMAND_DRAG;
    commandHandler.ClearFifo();
    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
        case COMMAND_DRAG:
            for (float& velocity : dragVelocityFiltered) velocity = 0;
            break;
    }
    if (leavingDrag) motorJ[ALL]->Hold();
}


static uint16_t NextSequence(uint16_t& sequence)
{
    ++sequence;
    if (sequence == 0) sequence = 1;
    return sequence;
}


void DummyRobot::SetDragGains(float _damping, float _frictionAlpha, float _currentLimit)
{
    if (!std::isfinite(_damping) || !std::isfinite(_frictionAlpha) ||
        !std::isfinite(_currentLimit) ||
        _damping < 0 || _damping > 0.2f ||
        _frictionAlpha < 0 || _frictionAlpha > 1 ||
        _currentLimit < 0.1f || _currentLimit > 1.5f)
        return;
    dragDamping = _damping;
    dragFrictionAlpha = _frictionAlpha;
    dragCurrentLimit = _currentLimit;
}


void DummyRobot::GetDragGains(float* _gains) const
{
    _gains[0] = dragDamping;
    _gains[1] = dragFrictionAlpha;
    _gains[2] = dragCurrentLimit;
}


void DummyRobot::UpdateGravityFeedForward(uint32_t tick)
{
    if (tick % 4) return; // 50 Hz from the 200 Hz control task.
    const uint16_t sequence = NextSequence(feedForwardSequence);
    float gravityCurrentA[7]{};
    if (gravityFeedForwardEnabled && IsEnabled() && !HasFault())
        gravityModel.CalculateGravityCurrentA(currentJoints.a, gravityCurrentA);
    for (unsigned j = 1; j <= 6; ++j)
    {
        const float jointCurrentA = gravityCurrentA[j];
        const float motorCurrentA = motorJ[j]->inverseDirection ? -jointCurrentA : jointCurrentA;
        motorJ[j]->SetCurrentFeedForward(motorCurrentA, sequence, 100);
    }
}


void DummyRobot::DragTick(uint32_t tick)
{
    if (!IsEnabled() || HasFault())
    {
        motorJ[ALL]->Hold();
        return;
    }
    if (tick % 20 == 0)
    {
        for (unsigned j = 1; j <= 6; ++j)
        {
            motorJ[j]->RequestPositionFeedback();
            motorJ[j]->RequestVelocityFeedback();
        }
    }

    const uint16_t sequence = NextSequence(dragSequence);
    float gravityCurrentA[7]{};
    gravityModel.CalculateGravityCurrentA(currentJoints.a, gravityCurrentA);
    for (unsigned j = 1; j <= 6; ++j)
    {
        const float measuredVelocity = motorJ[j]->VelocityAge() < 250 ?
                                       motorJ[j]->GetVelocityDegS() : 0;
        dragVelocityFiltered[j] += 0.2f * (measuredVelocity - dragVelocityFiltered[j]);
        const float velocity = dragVelocityFiltered[j];
        float jointCurrentA = gravityCurrentA[j];
        const float loss = gravityModel.FrictionLossA(j, velocity);
        const float smoothSign = velocity / (fabsf(velocity) + 2.0f);
        jointCurrentA += dragFrictionAlpha * smoothSign * loss - dragDamping * velocity;

        const float q = currentJoints.a[j - 1];
        const float maxAngle = motorJ[j]->angleLimitMax;
        const float minAngle = motorJ[j]->angleLimitMin;
        if (q >= maxAngle - 2 && jointCurrentA > 0) jointCurrentA = 0;
        if (q <= minAngle + 2 && jointCurrentA < 0) jointCurrentA = 0;
        if (fabsf(velocity) > 30 &&
            ((velocity > 0 && jointCurrentA > 0) || (velocity < 0 && jointCurrentA < 0)))
            jointCurrentA = 0;

        jointCurrentA = GravityCompensation::Model::Clamp(jointCurrentA, dragCurrentLimit);
        const float motorCurrentA = motorJ[j]->inverseDirection ?
                                    -jointCurrentA : jointCurrentA;
        motorJ[j]->SetDragCurrent(motorCurrentA, sequence, 200);
    }
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
{

}


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 45)_angle = 45;
    if (_angle < 0)_angle = 0;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t canBuf[8] = {};
    CAN_TxHeaderTypeDef txHeader = baseHeader;
    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd, StreamSink* response)
{
    Entry entry;
    MotionCommand parsed;
    if (_cmd.empty() || _cmd.size() >= sizeof(entry.text) || !parsed.Parse(_cmd.c_str()) ||
        context->commandMode == COMMAND_MOTOR_TUNING)
        return 0xFF;
    if (parsed.kind == '>')
        for (unsigned j = 1; j <= 6; ++j)
            if (parsed.values[j - 1] < context->motorJ[j]->angleLimitMin ||
                parsed.values[j - 1] > context->motorJ[j]->angleLimitMax)
                return 0xFF;
    memcpy(entry.text, _cmd.c_str(), _cmd.size());
    entry.response = response;
    taskENTER_CRITICAL();
    if (!context->IsEnabled() || context->HasFault() || context->recovering)
    {
        taskEXIT_CRITICAL();
        return 0xFF;
    }
    if (context->commandMode == COMMAND_TARGET_POINT_INTERRUPTABLE)
        ClearFifo(); // INT: only the newest pending target is useful.
    entry.generation = generation;
    const osStatus_t status = osMessageQueuePut(commandFifo, &entry, 0U, 0U);
    taskEXIT_CRITICAL();
    return status == osOK ? osMessageQueueGetSpace(commandFifo) : 0xFF;
}

void DummyRobot::CommandHandler::EmergencyStop()
{
    context->StopMotion();
}

DummyRobot::CommandHandler::Entry DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    Entry entry;
    if (osMessageQueueGet(commandFifo, &entry, nullptr, timeout) != osOK)
        entry.text[0] = 0;
    return entry;
}

uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}

uint32_t DummyRobot::CommandHandler::ParseCommand(const Entry &entry)
{
    if (!entry.text[0]) return GetSpace();
    MotionCommand parsed;
    bool accepted = false;
    taskENTER_CRITICAL();
    const bool active = entry.generation == generation && context->IsEnabled() && !context->HasFault();
    taskEXIT_CRITICAL();
    if (active && parsed.Parse(entry.text))
    {
        if (parsed.count >= 7) context->SetJointSpeed(parsed.values[6]);
        // Commit only while this entry is still current; IK can take time.
        if (entry.generation == generation)
        {
            const float* v = parsed.values;
            accepted = parsed.kind == '>' ? context->MoveJAtGeneration(v[0], v[1], v[2], v[3], v[4], v[5], entry.generation) :
                                           context->MoveLAtGeneration(v[0], v[1], v[2], v[3], v[4], v[5], entry.generation);
        }
        if (accepted && context->IsEnabled() && entry.generation == generation)
        {
            if (parsed.count == 9) context->hand->SetMaxCurrent(parsed.values[8]);
            if (parsed.count >= 8) context->hand->SetAngle(parsed.values[7]);
            if (context->commandMode == COMMAND_TARGET_POINT_SEQUENTIAL ||
                context->commandMode == COMMAND_CONTINUES_TRAJECTORY)
                accepted = context->WaitForMotion();
        }
    }
    if (entry.response)
        Respond(*entry.response, accepted ? "ok" : "error motion rejected or interrupted");
    return GetSpace();
}

void DummyRobot::CommandHandler::ClearFifo()
{
    taskENTER_CRITICAL();
    ++generation;
    osMessageQueueReset(commandFifo);
    taskEXIT_CRITICAL();
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}
