#include "communication.hpp"
#include "dummy_robot.h"
#include "motion_command.h"

static float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
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


DummyRobot::Config_t::Config_t()
{
    controlFrequencyHz = 200;
    gravityFeedForwardHz = 50;
    feedForwardLeaseMs = 100;
    dragFeedbackHz = 10;
    dragLeaseMs = 200;
    dragVelocityValidMs = 250;
    dragVelocityFilterAlpha = 0.2f;
    dragVelocitySmoothingDegS = 2.0f;
    dragVelocityCutoffDegS = 30.0f;
    dragEndStopMarginDeg = 2.0f;
    dragDamping = 0.0f;
    dragFrictionAlpha = 0.0f;
    dragCurrentLimit = 1.0f;
    dragDampingMax = 0.2f;
    dragFrictionAlphaMax = 1.0f;
    dragCurrentLimitMin = 0.1f;
    dragCurrentLimitMax = 1.5f;
    gravityFeedForwardEnabled = false;
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

    for (unsigned j = 4; j <= 5; ++j)
    {
        WristAxisConfig_t& state = wrist[j];
        const unsigned index = j - 4;
        memcpy(state.harmonicBaseMa, WRIST_BASE_HARMONIC_MA[index],
               sizeof(state.harmonicBaseMa));
    }

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
        if (!motorJ[ALL]->SetEnable(false)) motorJ[ALL]->Hold();
        DisableWristCompensationNow();
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
    if (!motorJ[ALL]->Hold())
        for (unsigned j = 1; j <= 6; ++j) motorJ[j]->Hold();
    SuspendWristCompensationNow();
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
            if (!leavingDrag && !isEnabled && !recovering)
            {
                for (unsigned joint = 4; joint <= 5; ++joint)
                {
                    WristAxisConfig_t& state = wrist[joint];
                    state.dragAutoEnabled = false;
                    state.dragLocalLimitMa = state.frictionLimitMa;
                    state.dragFrictionScale = state.frictionScale;
                    memcpy(state.dragHarmonicMa, state.harmonicBaseMa,
                           sizeof(state.dragHarmonicMa));
                    if (state.dragLocalLimitMa <= 0)
                        state.dragLocalLimitMa = 55;
                    if (state.dragFrictionScale <= 0)
                        state.dragFrictionScale = 0.3f;
                    state.frictionLimitMa = state.dragLocalLimitMa;
                    state.frictionScale = state.dragFrictionScale;
                    state.frictionRequested = true;
                    state.speedRequested = true;
                    if (!motorJ[joint]->SetHarmonicCompensation(
                            state.dragHarmonicMa, false) ||
                        !ApplyWristCompensation(joint, true))
                    {
                        state.dragAutoEnabled = false;
                        state.frictionRequested = false;
                        state.speedRequested = false;
                        DisableWristCompensationNow();
                    } else
                    {
                        state.dragAutoEnabled = true;
                    }
                }
            }
            break;
    }
    if (leavingDrag)
    {
        if (!motorJ[ALL]->Hold())
            for (unsigned j = 1; j <= 6; ++j) motorJ[j]->Hold();
        SuspendWristCompensationNow();
    }
}


static uint16_t NextSequence(uint16_t& sequence)
{
    ++sequence;
    if (sequence == 0) sequence = 1;
    return sequence;
}


void DummyRobot::SetGravityFeedForwardEnabled(bool _enabled)
{
    config.gravityFeedForwardEnabled = _enabled;
}


bool DummyRobot::IsGravityFeedForwardEnabled() const
{
    return config.gravityFeedForwardEnabled;
}


uint16_t DummyRobot::GetFeedForwardSequence() const
{
    return feedForwardSequence;
}


uint16_t DummyRobot::GetDragSequence() const
{
    return dragSequence;
}


void DummyRobot::SetDragGains(float _damping, float _frictionAlpha, float _currentLimit)
{
    if (!MotionFinite(_damping) || !MotionFinite(_frictionAlpha) ||
        !MotionFinite(_currentLimit) ||
        _damping < 0 || _damping > config.dragDampingMax ||
        _frictionAlpha < 0 || _frictionAlpha > config.dragFrictionAlphaMax ||
        _currentLimit < config.dragCurrentLimitMin || _currentLimit > config.dragCurrentLimitMax)
        return;
    config.dragDamping = _damping;
    config.dragFrictionAlpha = _frictionAlpha;
    config.dragCurrentLimit = _currentLimit;
}


void DummyRobot::GetDragGains(float* _gains) const
{
    _gains[0] = config.dragDamping;
    _gains[1] = config.dragFrictionAlpha;
    _gains[2] = config.dragCurrentLimit;
}


uint16_t DummyRobot::NextWristSequence()
{
    ++wristSequence;
    if (wristSequence == 0) wristSequence = 1;
    return wristSequence;
}


static bool IsWristJoint(uint32_t joint)
{
    return joint == 4 || joint == 5;
}


static uint8_t WristGainPercent(float scale)
{
    const long percent = lroundf(scale * 100.0f);
    if (percent < 0) return 0;
    if (percent > 100) return 100;
    return static_cast<uint8_t>(percent);
}


bool DummyRobot::SetWristGravity(uint32_t joint, float scale)
{
    if (!IsWristJoint(joint) || isEnabled || recovering || !MotionFinite(scale) ||
        scale < 0 || scale > 1)
        return false;
    WristAxisConfig_t& state = wrist[joint];
    state.gravityScale = scale;
    state.gravityRequested = scale > 0;
    return SetGravity(joint, scale);
}


bool DummyRobot::SetWristGravityModel(uint32_t joint, float aPerNm,
                                      float biasA, float limitA)
{
    if (!IsWristJoint(joint) || isEnabled || recovering ||
        !MotionFinite(aPerNm) || !MotionFinite(biasA) || !MotionFinite(limitA))
        return false;
    if (!SetGravityModel(joint, aPerNm, biasA, limitA)) return false;
    WristAxisConfig_t& state = wrist[joint];
    state.gravityAPerNm = aPerNm;
    state.gravityBiasA = biasA;
    state.gravityLimitA = limitA;
    return true;
}


bool DummyRobot::SetGravityModel(uint32_t joint, float aPerNm,
                                 float biasA, float limitA)
{
    if (joint < 1 || joint > 6 || isEnabled || recovering ||
        !MotionFinite(aPerNm) || !MotionFinite(biasA) || !MotionFinite(limitA))
        return false;
    return gravityModel.SetJointGravityModel(joint, aPerNm, biasA, limitA);
}


bool DummyRobot::SetGravity(uint32_t joint, float scale)
{
    if (joint < 1 || joint > 6 || isEnabled || recovering ||
        !MotionFinite(scale) || scale < 0 || scale > 1)
        return false;
    return gravityModel.SetJointGravityScale(joint, scale);
}


bool DummyRobot::SetWristHarmonics(uint32_t joint, int32_t eSin, int32_t eCos,
                                   int32_t oSin, int32_t oCos)
{
    if (!IsWristJoint(joint) || isEnabled || recovering) return false;
    const int32_t values[4] = {eSin, eCos, oSin, oCos};
    for (int32_t value : values)
        if (value < -500 || value > 500) return false;
    const int16_t valuesMa[4] = {
        static_cast<int16_t>(eSin), static_cast<int16_t>(eCos),
        static_cast<int16_t>(oSin), static_cast<int16_t>(oCos)};
    if (!motorJ[joint]->SetHarmonicCompensation(valuesMa, false)) return false;
    WristAxisConfig_t& state = wrist[joint];
    memcpy(state.harmonicBaseMa, valuesMa, sizeof(state.harmonicBaseMa));
    memcpy(state.harmonicMa, valuesMa, sizeof(state.harmonicMa));
    state.harmonicVerified = true;
    return true;
}


bool DummyRobot::SetWristHarmonicsStored(uint32_t joint, int32_t eSin, int32_t eCos,
                                         int32_t oSin, int32_t oCos)
{
    if (!IsWristJoint(joint) || isEnabled || recovering) return false;
    const int32_t inputs[4] = {eSin, eCos, oSin, oCos};
    for (int32_t value : inputs)
        if (value < -500 || value > 500) return false;
    const int16_t valuesMa[4] = {
        static_cast<int16_t>(eSin), static_cast<int16_t>(eCos),
        static_cast<int16_t>(oSin), static_cast<int16_t>(oCos)};
    const bool storeToEeprom = true;
    if (!motorJ[joint]->SetHarmonicCompensation(valuesMa, storeToEeprom)) return false;
    WristAxisConfig_t& state = wrist[joint];
    memcpy(state.harmonicBaseMa, valuesMa, sizeof(state.harmonicBaseMa));
    memcpy(state.harmonicMa, valuesMa, sizeof(state.harmonicMa));
    state.harmonicVerified = true;
    return true;
}


bool DummyRobot::SetLocalCompensation(uint32_t joint, const int32_t values[5],
                                      bool storeToEeprom)
{
    if (joint < 1 || joint > 6 || isEnabled || recovering || !values) return false;
    if (values[0] < 0 || values[0] > 1000000 ||
        values[1] < 0 || values[1] > 3300 ||
        values[2] < 0 || values[2] > 1000000 ||
        values[3] < LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000[joint] ||
        values[3] > 1000000 ||
        values[4] < 0 || values[4] > 3300)
        return false;
    return motorJ[joint]->SetLocalCompensation(
        values[0], values[1], values[2], values[3], values[4], storeToEeprom);
}


bool DummyRobot::SetAllJointParams()
{
    if (isEnabled || recovering) return false;

    for (uint32_t joint = 1; joint <= 6; ++joint)
    {
        CtrlStepMotor* motor = motorJ[joint];

        motor->SetDceKp(DCE_KP[joint]);
        motor->SetDceKv(DCE_KV[joint]);
        motor->SetDceKi(DCE_KI[joint]);
        motor->SetDceKd(DCE_KD[joint]);

        if (!motor->SetLocalCompensation(LOCAL_COMP[joint][0],
                                         LOCAL_COMP[joint][1],
                                         LOCAL_COMP[joint][2],
                                         LOCAL_COMP[joint][3],
                                         LOCAL_COMP[joint][4],
                                         true))
            return false;

        const int16_t* harmonics = ALL_HARMONICS_MA[joint];
        if (IsWristJoint(joint) &&
            !motor->SetHarmonicCompensation(harmonics, true))
            return false;

        if (!IsWristJoint(joint)) continue;

        WristAxisConfig_t& state = wrist[joint];
        memcpy(state.harmonicBaseMa, harmonics, sizeof(state.harmonicBaseMa));
        memcpy(state.harmonicMa, harmonics, sizeof(state.harmonicMa));
        memcpy(state.dragHarmonicMa, harmonics, sizeof(state.dragHarmonicMa));
        state.harmonicVerified = true;
        state.frictionLimitMa = LOCAL_COMP[joint][4];
        state.frictionScale = LOCAL_COMP[joint][1] > 0 ? 0.3f : 0.0f;
        state.dragLocalLimitMa = state.frictionLimitMa;
        state.dragFrictionScale = state.frictionScale;
    }

    return true;
}


bool DummyRobot::SetWristLocalLimit(uint32_t joint, int32_t limitMa)
{
    if (!IsWristJoint(joint) || isEnabled || recovering ||
        limitMa < 0 || limitMa > 500)
        return false;
    wrist[joint].frictionLimitMa = limitMa;
    return true;
}


void DummyRobot::SetDragLocalLimit(uint32_t joint, int32_t limitMa)
{
    if (!IsWristJoint(joint)) return;
    wrist[joint].dragLocalLimitMa = limitMa < 0 ? 0 : (limitMa > 500 ? 500 : limitMa);
}


float DummyRobot::CalculateWristGravityCurrentA(uint8_t joint) const
{
    if (!IsWristJoint(joint)) return 0;
    return gravityModel.CalculateCalibratedJointCurrentA(joint, currentJoints.a);
}


bool DummyRobot::ConfirmWristStatus(unsigned joint, uint16_t expectedSequence,
                                    bool expectedEnabled)
{
    WristAxisConfig_t& state = wrist[joint];
    CtrlStepMotor::WristCompensationStatus status{};
    bool acknowledged = false;
    for (unsigned attempt = 0; attempt < 3; ++attempt)
    {
        if (!motorJ[joint]->QueryWristCompensationStatus(status)) break;
        state.protocolVerified = true;
        if (status.sequence == expectedSequence &&
            ((status.flags & 1U) != 0) == expectedEnabled)
        {
            acknowledged = true;
            break;
        }
    }
    if (!acknowledged) return false;

    state.statusFlags = status.flags;
    state.direction = status.direction;
    state.actualAssistMa = status.actualAssistMa;
    state.statusSequence = status.sequence;
    state.statusTick = HAL_GetTick();
    state.speedEnabled = (status.flags & 1U) != 0;
    state.frictionEnabled =
        state.speedEnabled && state.requestedSequence == status.sequence &&
        state.frictionRequested && state.appliedGainPercent > 0 &&
        state.appliedCapMa > 0 && (status.flags & 2U) != 0;
    state.controlVerified = true;
    state.safeOffVerified = state.appliedGainPercent == 0 &&
                            state.appliedCapMa == 0;
    return true;
}


bool DummyRobot::SendWristControl(unsigned joint, bool assistEnabled,
                                  bool keepSpeedMeasurement, bool verify)
{
    if (!IsWristJoint(joint)) return false;
    WristAxisConfig_t& state = wrist[joint];
    const bool enable = assistEnabled || keepSpeedMeasurement;
    const uint8_t gainPercent = assistEnabled && state.frictionRequested ?
                                WristGainPercent(state.frictionScale) : 0;
    const uint16_t capMa = assistEnabled && state.frictionRequested ?
                           static_cast<uint16_t>(state.frictionLimitMa) : 0;
    const uint16_t sequence = NextWristSequence();
    state.requestedSequence = sequence;
    state.controlVerified = false;
    state.safeOffVerified = false;
    state.appliedGainPercent = gainPercent;
    state.appliedCapMa = capMa;

    if (!motorJ[joint]->SetWristCompensation(enable, gainPercent, capMa, sequence))
    {
        motorJ[joint]->Hold();
        state.speedEnabled = false;
        state.frictionEnabled = false;
        return false;
    }
    if (enable)
    {
        state.speedEnabled = true;
        state.frictionEnabled = assistEnabled;
    }
    else
    {
        state.speedEnabled = false;
        state.frictionEnabled = false;
    }
    if (!verify)
    {
        state.safeOffVerified = gainPercent == 0 && capMa == 0;
        return true;
    }
    if (ConfirmWristStatus(joint, sequence, enable)) return true;

    // The write may have reached the board even though its acknowledgement did
    // not. Supersede it with an unacknowledged zero-assist frame, then HOLD
    // locally so neither this axis nor a later action can reuse the old lease.
    const uint16_t rollbackSequence = NextWristSequence();
    state.requestedSequence = rollbackSequence;
    state.appliedGainPercent = 0;
    state.appliedCapMa = 0;
    motorJ[joint]->SetWristCompensation(false, 0, 0, rollbackSequence);
    motorJ[joint]->Hold();
    state.controlVerified = false;
    state.safeOffVerified = false;
    state.speedEnabled = false;
    state.frictionEnabled = false;
    return false;
}


bool DummyRobot::ApplyWristCompensation(unsigned joint, bool enable)
{
    if (!IsWristJoint(joint) || enable && (isEnabled || recovering)) return false;
    auto* motor = motorJ[joint];
    WristAxisConfig_t& state = wrist[joint];
    state.oldProtocol = false;
    state.harmonicVerified = false;
    state.controlVerified = false;
    state.safeOffVerified = false;
    state.speedEnabled = false;
    state.frictionEnabled = false;

    // Establish both halves of the v2 handshake before sending any assist
    // configuration. Old firmware must never be followed by a 0x0E opt-in.
    int16_t existing[4]{};
    if (!motor->QueryHarmonicCompensation(existing))
    {
        state.oldProtocol = true;
        state.protocolVerified = false;
        motor->Hold();
        return false;
    }

    CtrlStepMotor::WristCompensationStatus status{};
    if (!motor->QueryWristCompensationStatus(status))
    {
        state.oldProtocol = true;
        state.protocolVerified = false;
        motor->Hold();
        return false;
    }
    state.protocolVerified = true;

    // If another producer currently owns the board, first prove a zero-gain
    // speed-only state before touching RAM parameters.
    if ((status.flags & 1U) &&
        !SendWristControl(joint, false, false, true))
        return false;

    if (enable && state.frictionRequested)
    {
        if (state.frictionLimitMa <= 0)
        {
            SendWristControl(joint, false, false, true);
            return false;
        }
        const int16_t* target = state.harmonicBaseMa;
        if (!motor->SetHarmonicCompensation(target))
        {
            SendWristControl(joint, false, false, true);
            return false;
        }
        int16_t readback[4]{};
        if (!motor->QueryHarmonicCompensation(readback) ||
            memcmp(readback, target, sizeof(state.harmonicBaseMa)) != 0)
        {
            SendWristControl(joint, false, false, true);
            return false;
        }
        memcpy(state.harmonicMa, target, sizeof(state.harmonicBaseMa));
        memcpy(state.harmonicReadbackMa, readback, sizeof(readback));
        state.harmonicVerified = true;
    }

    const bool assist = enable && state.frictionRequested;
    const bool keepSpeed = enable && state.speedRequested;
    if (!SendWristControl(joint, assist, keepSpeed, true))
        return false;
    return enable ? state.controlVerified : state.safeOffVerified;
}


bool DummyRobot::SetWristSpeed(uint32_t joint, bool enable)
{
    if (!IsWristJoint(joint)) return false;
    WristAxisConfig_t& state = wrist[joint];
    if (!enable)
    {
        if (!SendWristControl(joint, false, false, true)) return false;
        state.speedRequested = false;
        state.frictionRequested = false;
        state.speedEnabled = false;
        state.frictionEnabled = false;
        state.controlVerified = false;
        state.safeOffVerified = true;
        return true;
    }
    if (isEnabled || recovering) return false;
    state.speedRequested = true;
    const bool applied = ApplyWristCompensation(joint, true);
    if (!applied) state.controlVerified = false;
    return applied;
}


bool DummyRobot::SetWristFriction(uint32_t joint, float scale)
{
    if (!IsWristJoint(joint) || isEnabled || recovering || !MotionFinite(scale) ||
        scale < 0 || scale > 1)
        return false;
    WristAxisConfig_t& state = wrist[joint];
    state.frictionScale = scale;
    state.frictionRequested = scale > 0;
    state.speedRequested = true;
    const bool applied = ApplyWristCompensation(joint, true);
    if (!applied) state.controlVerified = false;
    return applied;
}


bool DummyRobot::RefreshWristStatus(uint32_t joint)
{
    if (!IsWristJoint(joint)) return false;
    WristAxisConfig_t& state = wrist[joint];
    int16_t values[4]{};
    if (!motorJ[joint]->QueryHarmonicCompensation(values)) return false;
    UpdateWristHarmonicReadback(joint, values);
    CtrlStepMotor::WristCompensationStatus status{};
    if (!motorJ[joint]->QueryWristCompensationStatus(status)) return false;
    state.protocolVerified = true;
    state.statusFlags = status.flags;
    state.direction = status.direction;
    state.actualAssistMa = status.actualAssistMa;
    state.statusSequence = status.sequence;
    state.statusTick = HAL_GetTick();
    const bool unexpectedEnable = (status.flags & 1U) && !state.speedRequested;
    if (unexpectedEnable || status.sequence != state.requestedSequence)
    {
        if (!SendWristControl(joint, false, false, true)) return false;
        state.speedEnabled = false;
        state.frictionEnabled = false;
        return true;
    }
    state.speedEnabled = (status.flags & 1U) != 0;
    state.frictionEnabled =
        state.speedEnabled && state.frictionRequested &&
        state.appliedGainPercent > 0 && state.appliedCapMa > 0 &&
        (status.flags & 2U);
    state.controlVerified = true;
    state.safeOffVerified = state.appliedGainPercent == 0 &&
                            state.appliedCapMa == 0;
    return state.speedEnabled || state.safeOffVerified;
}


bool DummyRobot::ResetWristCompensation(uint32_t joint)
{
    if (!IsWristJoint(joint) || isEnabled || recovering) return false;
    WristAxisConfig_t& state = wrist[joint];
    if (!SendWristControl(joint, false, false, true)) return false;
    state.gravityRequested = false;
    state.gravityScale = 0;
    state.frictionRequested = false;
    state.frictionScale = 1;
    state.frictionLimitMa = 0;
    state.speedRequested = false;
    state.speedEnabled = false;
    state.frictionEnabled = false;
    state.protocolVerified = false;
    state.controlVerified = false;
    state.safeOffVerified = true;
    state.harmonicVerified = false;
    state.oldProtocol = false;
    state.appliedGainPercent = 0;
    state.appliedCapMa = 0;
    state.requestedSequence = 0;
    state.statusSequence = 0;
    state.statusFlags = 0;
    state.direction = 0;
    state.actualAssistMa = 0;
    state.statusTick = 0;
    const unsigned index = joint - 4;
    gravityModel.ResetJointGravityModel(joint);
    memcpy(state.harmonicBaseMa, WRIST_BASE_HARMONIC_MA[index],
           sizeof(state.harmonicBaseMa));
    return true;
}


void DummyRobot::ClearWristRuntimeState(unsigned joint, bool clearConfiguration)
{
    WristAxisConfig_t& state = wrist[joint];
    state.speedEnabled = false;
    state.frictionEnabled = false;
    state.controlVerified = false;
    state.safeOffVerified = false;
    state.harmonicVerified = false;
    state.requestedSequence = 0;
    state.statusSequence = 0;
    state.statusFlags = 0;
    state.actualAssistMa = 0;
    state.direction = 0;
    state.statusTick = 0;
    state.appliedGainPercent = 0;
    state.appliedCapMa = 0;
    if (clearConfiguration)
    {
        state.speedRequested = false;
        state.frictionRequested = false;
        state.protocolVerified = false;
        state.oldProtocol = false;
    }
}


void DummyRobot::SuspendWristCompensationNow()
{
    for (unsigned joint = 4; joint <= 5; ++joint)
    {
        WristAxisConfig_t& state = wrist[joint];
        if (!state.speedRequested && !state.protocolVerified &&
            state.safeOffVerified)
            continue;
        const bool keepSpeed = state.speedRequested && state.protocolVerified;
        if (!SendWristControl(joint, false, keepSpeed, false))
            motorJ[joint]->Hold();
        state.frictionEnabled = false;
        state.controlVerified = false;
        state.safeOffVerified = false;
    }
}


void DummyRobot::DisableWristCompensationNow()
{
    for (unsigned joint = 4; joint <= 5; ++joint)
    {
        WristAxisConfig_t& state = wrist[joint];
        if (!state.speedRequested && !state.protocolVerified &&
            state.safeOffVerified)
            continue;
        if (!SendWristControl(joint, false, false, true))
        {
            motorJ[joint]->Hold();
            // Keep the axis on the v2 path until a fresh handshake confirms
            // safe-off. Falling back to 0x0B could revive a stale old lease.
            state.speedRequested = true;
            state.protocolVerified = false;
            state.controlVerified = false;
            state.safeOffVerified = false;
            state.speedEnabled = false;
            state.frictionEnabled = false;
            continue;
        }
        ClearWristRuntimeState(joint, true);
    }
}


void DummyRobot::UpdateWristHarmonicReadback(unsigned joint, const int16_t valuesMa[4])
{
    if (!IsWristJoint(joint) || !valuesMa) return;
    memcpy(wrist[joint].harmonicReadbackMa, valuesMa,
           sizeof(wrist[joint].harmonicReadbackMa));
}


void DummyRobot::UpdateWristCompensationStatus(unsigned joint, uint8_t flags, int8_t direction,
                                               int16_t actualAssistMa, uint16_t sequence)
{
    if (!IsWristJoint(joint)) return;
    WristAxisConfig_t& state = wrist[joint];
    state.statusFlags = flags;
    state.direction = direction;
    state.actualAssistMa = actualAssistMa;
    state.statusSequence = sequence;
    state.statusTick = HAL_GetTick();
    if (sequence == state.requestedSequence)
    {
        state.speedEnabled = (flags & 1U) != 0;
        state.frictionEnabled =
            state.speedEnabled && state.frictionRequested &&
            state.appliedGainPercent > 0 && state.appliedCapMa > 0 &&
            (flags & 2U);
        state.controlVerified = true;
        state.safeOffVerified = state.appliedGainPercent == 0 &&
                                state.appliedCapMa == 0;
        if (!state.speedEnabled)
        {
            state.frictionEnabled = false;
        }
    }
    else
    {
        state.controlVerified = false;
        state.safeOffVerified = false;
        state.speedEnabled = false;
        state.frictionEnabled = false;
    }
}


void DummyRobot::PollWristStatus(uint32_t tick)
{
    bool resyncRequired = false;
    for (unsigned joint = 4; joint <= 5; ++joint)
        resyncRequired |= wrist[joint].speedRequested && !wrist[joint].controlVerified;
    if (tick % 10U && !resyncRequired) return;
    const uint32_t now = HAL_GetTick();
    for (unsigned joint = 4; joint <= 5; ++joint)
    {
        WristAxisConfig_t& state = wrist[joint];
        if (!state.protocolVerified && !state.speedRequested) continue;
        if (!motorJ[joint]->RequestWristCompensationStatus())
        {
            motorJ[joint]->Hold();
            state.controlVerified = false;
            state.safeOffVerified = false;
            state.protocolVerified = false;
            state.speedEnabled = false;
            state.frictionEnabled = false;
            continue;
        }
        if (state.statusTick &&
            now - state.statusTick > config.dragVelocityValidMs)
        {
            if (!SendWristControl(joint, false, state.speedRequested, true))
            {
                motorJ[joint]->Hold();
                state.protocolVerified = false;
            }
            state.frictionEnabled = false;
        }
    }
}


static uint32_t ControlIntervalTicks(uint32_t controlFrequencyHz, uint32_t taskFrequencyHz)
{
    if (controlFrequencyHz == 0 || taskFrequencyHz == 0 || controlFrequencyHz < taskFrequencyHz)
        return 0;
    return controlFrequencyHz / taskFrequencyHz;
}


void DummyRobot::UpdateGravityFeedForward(uint32_t tick)
{
    const uint32_t intervalTicks = ControlIntervalTicks(config.controlFrequencyHz,
                                                        config.gravityFeedForwardHz);
    if (intervalTicks == 0 || tick % intervalTicks) return;
    PollWristStatus(tick);
    const uint16_t sequence = NextSequence(feedForwardSequence);
    float gravityCurrentA[7]{};
    if (IsEnabled() && !HasFault())
    {
        gravityModel.CalculateGravityCurrentA(currentJoints.a, gravityCurrentA);
    }
    for (unsigned j = 1; j <= 6; ++j)
    {
        float jointCurrentA = 0;
        if (config.gravityFeedForwardEnabled)
        {
            jointCurrentA = gravityCurrentA[j];
            if ((j == 4 || j == 5) && wrist[j].gravityRequested)
            {
                const float wristCurrentA = CalculateWristGravityCurrentA(j);
                jointCurrentA +=
                    (wristCurrentA - jointCurrentA) * wrist[j].gravityScale;
            }
        }
        jointCurrentA = GravityCompensation::Model::Clamp(jointCurrentA, 3.3f);
        const float motorCurrentA = motorJ[j]->inverseDirection ? -jointCurrentA : jointCurrentA;
        if (!motorJ[j]->SetCurrentFeedForward(
                motorCurrentA, sequence, config.feedForwardLeaseMs))
        {
            StopMotion();
            return;
        }
    }
}


void DummyRobot::DragTick(uint32_t tick)
{
    if (!IsEnabled() || HasFault())
    {
        motorJ[ALL]->Hold();
        return;
    }
    const uint32_t feedbackIntervalTicks = ControlIntervalTicks(config.controlFrequencyHz,
                                                                config.dragFeedbackHz);
    PollWristStatus(tick);
    if (feedbackIntervalTicks && tick % feedbackIntervalTicks == 0)
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
    const long configuredLimitMa = lroundf(config.dragCurrentLimit * 1000.0f);
    const uint16_t totalLimitMa = static_cast<uint16_t>(
        configuredLimitMa < 1 ? 1 : configuredLimitMa > 3300 ? 3300 :
        configuredLimitMa);
    bool safetyFailure = false;
    for (unsigned j = 1; j <= 6; ++j)
    {
        const float measuredVelocity = motorJ[j]->VelocityAge() < config.dragVelocityValidMs ?
                                       motorJ[j]->GetVelocityDegS() : 0;
        dragVelocityFiltered[j] += config.dragVelocityFilterAlpha *
                                   (measuredVelocity - dragVelocityFiltered[j]);
        const float velocity = dragVelocityFiltered[j];
        float jointCurrentA = gravityCurrentA[j];
        if ((j == 4 || j == 5) && wrist[j].gravityRequested)
        {
            const float wristCurrentA = CalculateWristGravityCurrentA(j);
            jointCurrentA +=
                (wristCurrentA - jointCurrentA) * wrist[j].gravityScale;
        }
        const bool localFrictionOwnsAxis =
            (j == 4 || j == 5) &&
            (wrist[j].speedRequested || wrist[j].speedEnabled);
        if (!localFrictionOwnsAxis)
        {
            const float loss = gravityModel.FrictionLossA(j, velocity);
            const float smoothSign = velocity /
                                     (fabsf(velocity) + config.dragVelocitySmoothingDegS);
            jointCurrentA += config.dragFrictionAlpha * smoothSign * loss -
                             config.dragDamping * velocity;
        }

        const float q = currentJoints.a[j - 1];
        const float maxAngle = motorJ[j]->angleLimitMax;
        const float minAngle = motorJ[j]->angleLimitMin;
        const bool overPositive = velocity > config.dragVelocityCutoffDegS;
        const bool overNegative = velocity < -config.dragVelocityCutoffDegS;
        const bool disableJointPositive =
            q >= maxAngle - config.dragEndStopMarginDeg || overPositive;
        const bool disableJointNegative =
            q <= minAngle + config.dragEndStopMarginDeg || overNegative;
        if (disableJointPositive && jointCurrentA > 0) jointCurrentA = 0;
        if (disableJointNegative && jointCurrentA < 0) jointCurrentA = 0;

        jointCurrentA = GravityCompensation::Model::Clamp(
            jointCurrentA, config.dragCurrentLimit);
        const float motorCurrentA = motorJ[j]->inverseDirection ?
                                    -jointCurrentA : jointCurrentA;
        if ((j == 4 || j == 5) && wrist[j].speedRequested)
        {
            if (!wrist[j].protocolVerified || !wrist[j].controlVerified ||
                !wrist[j].speedEnabled)
            {
                motorJ[j]->Hold();
                safetyFailure = true;
                break;
            }
            const long motorMaLong = lroundf(motorCurrentA * 1000.0f);
            const int16_t motorMa = static_cast<int16_t>(
                motorMaLong < -3300 ? -3300 : motorMaLong > 3300 ? 3300 :
                motorMaLong);
            bool disablePositiveMotor = disableJointPositive;
            bool disableNegativeMotor = disableJointNegative;
            if (motorJ[j]->inverseDirection)
                std::swap(disablePositiveMotor, disableNegativeMotor);
            if (!motorJ[j]->SetExtendedDragCurrent(
                    motorMa, totalLimitMa, sequence, config.dragLeaseMs,
                    disablePositiveMotor, disableNegativeMotor))
            {
                motorJ[j]->Hold();
                safetyFailure = true;
                break;
            }
        }
        else if (!motorJ[j]->SetDragCurrent(
                     motorCurrentA, sequence, config.dragLeaseMs))
        {
            motorJ[j]->Hold();
            safetyFailure = true;
            break;
        }
    }
    if (safetyFailure) StopMotion();
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
