#include "configurations.h"
#include "motor.h"
#include "control_math.h"

#include <cmath>
#include <cstring>


bool Motor::Controller::IsNewerSequence(uint16_t sequence, uint16_t last)
{
    return sequence != 0 && static_cast<int16_t>(sequence - last) > 0;
}


void Motor::Tick20kHz()
{
    // 1.Encoder data Update
    encoder->UpdateAngle();

    // 2.Motor Control Update
    CloseLoopControlTick();
}


void Motor::AttachEncoder(EncoderBase* _encoder)
{
    encoder = _encoder;
}


void Motor::AttachDriver(DriverBase* _driver)
{
    driver = _driver;
}


void Motor::CloseLoopControlTick()
{
    /************************************ First Called ************************************/
    static bool isFirstCalled = true;
    if (isFirstCalled)
    {
        int32_t angle;
        if (config.motionParams.encoderHomeOffset < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2)
        {
            angle =
                encoder->angleData.rectifiedAngle >
                config.motionParams.encoderHomeOffset + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2 ?
                encoder->angleData.rectifiedAngle - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS :
                encoder->angleData.rectifiedAngle;
        } else
        {
            angle =
                encoder->angleData.rectifiedAngle <
                config.motionParams.encoderHomeOffset - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2 ?
                encoder->angleData.rectifiedAngle + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS :
                encoder->angleData.rectifiedAngle;
        }

        controller->realLapPosition = angle;
        controller->realLapPositionLast = angle;
        controller->realPosition = angle;
        controller->realPositionLast = angle;

        isFirstCalled = false;
        return;
    }

    /********************************* Update Data *********************************/
    int32_t deltaLapPosition;

    // Read Encoder data
    controller->realLapPositionLast = controller->realLapPosition;
    controller->realLapPosition = encoder->angleData.rectifiedAngle;

    // Lap-Position calculate
    deltaLapPosition = controller->realLapPosition - controller->realLapPositionLast;
    if (deltaLapPosition > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        deltaLapPosition -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    else if (deltaLapPosition < -MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        deltaLapPosition += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

    // Naive-Position calculate
    controller->realPositionLast = controller->realPosition;
    controller->realPosition += deltaLapPosition;

    /********************************* Estimate Data *********************************/
    // Estimate Velocity
    // 32-tick moving average. The residual keeps the fractional part; adding
    // back 31 * estVelocity here would turn the filter into positive feedback.
    controller->estVelocityIntegral +=
        (controller->realPosition - controller->realPositionLast) * motionPlanner.CONTROL_FREQUENCY;
    controller->estVelocity = controller->estVelocityIntegral >> 5;
    controller->estVelocityIntegral -= (controller->estVelocity << 5);

    // Estimate Position
    controller->estLeadPosition = Controller::CompensateAdvancedAngle(controller->estVelocity);
    controller->estPosition = controller->realPosition + controller->estLeadPosition;

    // Estimate Error
    controller->estError = controller->softPosition - controller->estPosition;

    if (controller->clearFaultsRequested || controller->holdRequested)
    {
        const bool clear = controller->clearFaultsRequested;
        controller->clearFaultsRequested = false;
        controller->holdRequested = false;
        controller->ClearFeedForward();
        controller->goalPosition = controller->realPosition;
        controller->softPosition = controller->realPosition;
        controller->goalVelocity = controller->softVelocity = 0;
        controller->goalCurrent = controller->softCurrent = 0;
        controller->ClearIntegral();
        motionPlanner.positionTracker.NewTask(controller->realPosition, 0);
        controller->requestMode = clear ? MODE_STOP : MODE_COMMAND_POSITION;
        controller->modeRunning = controller->requestMode;
        controller->softNewCurve = false;
        controller->holding = !clear;
        if (clear)
        {
            controller->stalledTime = controller->overloadTime = 0;
            controller->isStalled = controller->overloadFlag = false;
            controller->controlOverrunFlag = false;
        }
    }

    controller->ConsumeFeedForwardRequest();
    controller->UpdateFeedForwardLease();

    /************************************ Ctrl Loop ************************************/
    if (controller->isStalled ||
        controller->softDisable ||
        !encoder->IsCalibrated())
    {
        controller->ClearIntegral();    // clear integrals
        controller->ClearFeedForward();
        controller->focPosition = 0;    // clear outputs
        controller->focCurrent = 0;
        driver->Sleep();
    } else if (controller->softBrake)
    {
        controller->ClearIntegral();
        controller->ClearFeedForward();
        controller->focPosition = 0;
        controller->focCurrent = 0;
        driver->Brake();
    } else
    {
        switch (controller->modeRunning)
        {
            case MODE_STEP_DIR:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            case MODE_STOP:
                controller->focCurrent = 0;
                controller->ClearIntegral();
                driver->Sleep();
                break;
            case MODE_COMMAND_Trajectory:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            case MODE_COMMAND_CURRENT:
                controller->CalcCurrentToOutput(controller->ApplyLeaseLocalCompensation(controller->softCurrent));
                break;
            case MODE_COMMAND_VELOCITY:
                controller->CalcPidToOutput(controller->softVelocity);
                break;
            case MODE_COMMAND_POSITION:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            case MODE_PWM_CURRENT:
                controller->CalcCurrentToOutput(controller->softCurrent);
                break;
            case MODE_PWM_VELOCITY:
                controller->CalcPidToOutput(controller->softVelocity);
                break;
            case MODE_PWM_POSITION:
                controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
                break;
            default:
                break;
        }
    }

    /******************************* Mode Change Handle *******************************/
    if (controller->modeRunning != controller->requestMode)
    {
        controller->modeRunning = controller->requestMode;
        controller->softNewCurve = true;
    }

    /******************************* Update Hard-Goal *******************************/
    if (controller->goalVelocity > config.motionParams.ratedVelocity)
        controller->goalVelocity = config.motionParams.ratedVelocity;
    else if (controller->goalVelocity < -config.motionParams.ratedVelocity)
        controller->goalVelocity = -config.motionParams.ratedVelocity;
    if (controller->goalCurrent > config.motionParams.ratedCurrent)
        controller->goalCurrent = config.motionParams.ratedCurrent;
    else if (controller->goalCurrent < -config.motionParams.ratedCurrent)
        controller->goalCurrent = -config.motionParams.ratedCurrent;

    /******************************** Motion Plan *********************************/
    if ((controller->softDisable && !controller->goalDisable) ||
        (controller->softBrake && !controller->goalBrake))
    {
        controller->softNewCurve = true;
    }

    if (controller->softNewCurve)
    {
        controller->softNewCurve = false;
        controller->ClearIntegral();

        switch (controller->modeRunning)
        {
            case MODE_STOP:
                break;
            case MODE_COMMAND_POSITION:
                motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            case MODE_COMMAND_VELOCITY:
                motionPlanner.velocityTracker.NewTask(controller->estVelocity);
                break;
            case MODE_COMMAND_CURRENT:
                motionPlanner.currentTracker.NewTask(controller->focCurrent);
                break;
            case MODE_COMMAND_Trajectory:
                motionPlanner.trajectoryTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            case MODE_PWM_POSITION:
                motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
                break;
            case MODE_PWM_VELOCITY:
                motionPlanner.velocityTracker.NewTask(controller->estVelocity);
                break;
            case MODE_PWM_CURRENT:
                motionPlanner.currentTracker.NewTask(controller->focCurrent);
                break;
            case MODE_STEP_DIR:
                motionPlanner.positionInterpolator.NewTask(controller->estPosition, controller->estVelocity);
                // step/dir mode uses delta-position, so stay where we are
                controller->goalPosition = controller->estPosition;
                break;
            default:
                break;
        }
    }

    /******************************* Update Soft Goal *******************************/
    switch (controller->modeRunning)
    {
        case MODE_STOP:
            break;
        case MODE_COMMAND_POSITION:
            motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionTracker.go_location;
            controller->softVelocity = motionPlanner.positionTracker.go_velocity;
            break;
        case MODE_COMMAND_VELOCITY:
            motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
            controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
            break;
        case MODE_COMMAND_CURRENT:
            motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
            controller->softCurrent = motionPlanner.currentTracker.goCurrent;
            break;
        case MODE_COMMAND_Trajectory:
            motionPlanner.trajectoryTracker.CalcSoftGoal(controller->goalPosition, controller->goalVelocity);
            controller->softPosition = motionPlanner.trajectoryTracker.goPosition;
            controller->softVelocity = motionPlanner.trajectoryTracker.goVelocity;
            break;
        case MODE_PWM_POSITION:
            motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionTracker.go_location;
            controller->softVelocity = motionPlanner.positionTracker.go_velocity;
            break;
        case MODE_PWM_VELOCITY:
            motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
            controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
            break;
        case MODE_PWM_CURRENT:
            motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
            controller->softCurrent = motionPlanner.currentTracker.goCurrent;
            break;
        case MODE_STEP_DIR:
            motionPlanner.positionInterpolator.CalcSoftGoal(controller->goalPosition);
            controller->softPosition = motionPlanner.positionInterpolator.goPosition;
            controller->softVelocity = motionPlanner.positionInterpolator.goVelocity;
            break;
        default:
            break;
    }

    controller->softDisable = controller->goalDisable;
    controller->softBrake = controller->goalBrake;

    /******************************** State Check ********************************/
    int32_t current = abs(controller->focCurrent);

    // Require consecutive saturation AND low speed. Recovery of either resets the timer.
    if (controller->config->stallProtectSwitch)
    {
        if ((// Current Mode
            ((controller->modeRunning == MODE_COMMAND_CURRENT ||
              controller->modeRunning == MODE_PWM_CURRENT) &&
             (current != 0))
            || // Other Mode
            current >= config.motionParams.ratedCurrent) &&
            config.motionParams.ratedCurrent > 0 &&
            abs(controller->estVelocity) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5)
        {
                if (controller->stalledTime >= 1000 * 1000)
                    controller->isStalled = true;
                else
                    controller->stalledTime += motionPlanner.CONTROL_PERIOD;
        } else // can ONLY clear stall flag  MANUALLY
        {
            controller->stalledTime = 0;
        }
    }

    // Overload detect
    if ((controller->modeRunning != MODE_COMMAND_CURRENT) &&
        (controller->modeRunning != MODE_PWM_CURRENT) &&
        controller->modeRunning != MODE_STOP &&
        config.motionParams.ratedCurrent > 0 &&
        (current >= config.motionParams.ratedCurrent))
    {
        if (controller->overloadTime >= 1000 * 1000)
        {
            if (!controller->overloadFlag) controller->RequestHold();
            controller->overloadFlag = true;
        }
        else
            controller->overloadTime += motionPlanner.CONTROL_PERIOD;
    } else // Keep the event latched until an explicit clear, including after STOP.
    {
        controller->overloadTime = 0;
    }

    /******************************** Update State ********************************/
    if (!encoder->IsCalibrated())
        controller->state = STATE_NO_CALIB;
    else if (controller->controlOverrunFlag)
        controller->state = STATE_CONTROL_OVERRUN;
    else if (controller->isStalled)
        controller->state = STATE_STALL;
    else if (controller->overloadFlag)
        controller->state = STATE_OVERLOAD;
    else if (controller->modeRunning == MODE_STOP)
        controller->state = STATE_STOP;
    else
    {
        if (controller->modeRunning == MODE_COMMAND_POSITION)
        {
            if ((controller->softPosition == controller->goalPosition)
                && (controller->softVelocity == 0)
                && abs(controller->goalPosition - controller->realPosition) <= SOFT_DIVIDE_NUM
                && abs(controller->estVelocity) <= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 100)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else if (controller->modeRunning == MODE_COMMAND_VELOCITY)
        {
            if (controller->softVelocity == controller->goalVelocity)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else if (controller->modeRunning == MODE_COMMAND_CURRENT)
        {
            if (controller->softCurrent == controller->goalCurrent)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        } else
        {
            controller->state = STATE_FINISH;
        }
    }
}


int32_t Motor::Controller::ApplyLeaseLocalCompensation(int32_t current)
{
    if (!dragActive || dragLeaseTicks == 0) return current;
    const LocalCompensationConfig_t& compensation = config->localComp;
    return ApplyLocalCompensationQ16(current, estVelocity,
                                     compensation.assistCoeffQ16, compensation.coulomb_mA,
                                     compensation.deadzoneSteps, compensation.rampInvQ16,
                                     compensation.limit_mA,
                                     context->config.motionParams.ratedCurrent);
}


void Motor::Controller::CalcCurrentToOutput(int32_t current)
{
    focCurrent = current;

    if (focCurrent > 0)
        focPosition = estPosition + context->SOFT_DIVIDE_NUM; // ahead phase of 90°
    else if (focCurrent < 0)
        focPosition = estPosition - context->SOFT_DIVIDE_NUM; // behind phase of 90°
    else focPosition = estPosition;

    context->driver->SetFocCurrentVector(focPosition, focCurrent);
}


void Motor::Controller::CalcPidToOutput(int32_t _speed)
{
    config->pid.vErrorLast = config->pid.vError;
    config->pid.vError = _speed - estVelocity;
    if (config->pid.vError > (1024 * 1024)) config->pid.vError = (1024 * 1024);
    if (config->pid.vError < (-1024 * 1024)) config->pid.vError = (-1024 * 1024);
    config->pid.outputKp = ((config->pid.kp) * (config->pid.vError));

    config->pid.outputKd = config->pid.kd * (config->pid.vError - config->pid.vErrorLast);
    IntegrateLimited<1024>(config->pid.outputKi, config->pid.integralRound,
                     static_cast<int64_t>(config->pid.ki) * config->pid.vError,
                     static_cast<int64_t>(config->pid.outputKp) + config->pid.outputKd,
                     context->config.motionParams.ratedCurrent);

    config->pid.output = (config->pid.outputKp + config->pid.outputKi + config->pid.outputKd) >> 10;
    if (config->pid.output > context->config.motionParams.ratedCurrent)
        config->pid.output = context->config.motionParams.ratedCurrent;
    else if (config->pid.output < -context->config.motionParams.ratedCurrent)
        config->pid.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->pid.output);
}


void Motor::Controller::CalcDceToOutput(int32_t _location, int32_t _speed)
{
    config->dce.pError = _location - estPosition;
    if (config->dce.pError > (3200)) config->dce.pError = (3200);   // limited pError to 1/16r (51200/16)
    if (config->dce.pError < (-3200)) config->dce.pError = (-3200);
    config->dce.vError = (_speed - estVelocity) >> 7;
    if (config->dce.vError > (4000)) config->dce.vError = (4000);   // limited vError
    if (config->dce.vError < (-4000)) config->dce.vError = (-4000);

    config->dce.outputKp = config->dce.kp * config->dce.pError;

    config->dce.outputKd = config->dce.kd * config->dce.vError;
    const int64_t ff = static_cast<int64_t>(feedForwardActive ? feedForwardCurrent_mA : 0) * 1024;
    const int64_t proportionalDerivative =
        static_cast<int64_t>(config->dce.outputKp) + config->dce.outputKd + ff;
    IntegrateLimited<128>(config->dce.outputKi, config->dce.integralRound,
                     static_cast<int64_t>(config->dce.ki) * config->dce.pError +
                     static_cast<int64_t>(config->dce.kv) * config->dce.vError,
                     proportionalDerivative,
                     context->config.motionParams.ratedCurrent);

    const int64_t rawFeedback = static_cast<int64_t>(config->dce.outputKp) +
                                config->dce.outputKi + config->dce.outputKd;
    const int64_t rawTotal = rawFeedback + ff;
    const int32_t limit = context->config.motionParams.ratedCurrent;
    feedbackCurrent_mA = static_cast<int32_t>(rawFeedback >> 10);
    feedForwardLimited = rawTotal > static_cast<int64_t>(limit) * 1024 ||
                         rawTotal < -static_cast<int64_t>(limit) * 1024;
    int64_t total = rawTotal >> 10;
    if (total > limit) total = limit;
    else if (total < -limit) total = -limit;
    config->dce.output = static_cast<int32_t>(total);
    totalCurrent_mA = config->dce.output;

    CalcCurrentToOutput(config->dce.output);
}


void Motor::Controller::SetCtrlMode(Motor::Mode_t _mode)
{
    if (_mode != MODE_STOP && (GetFaultFlags() || motionBlocked)) return;
    if (_mode == MODE_STOP)
    {
        holding = false;
        holdRequested = false;
        motionBlocked = true;
    }
    if (_mode != MODE_COMMAND_POSITION && _mode != MODE_COMMAND_Trajectory)
        ClearFeedForward();
    if (_mode != MODE_COMMAND_CURRENT)
        RestoreCurrentAcceleration();
    requestMode = _mode;
}


void Motor::Controller::AddTrajectorySetPoint(int32_t _pos, int32_t _vel)
{
    SetPositionSetPoint(_pos);
    SetVelocitySetPoint(_vel);
}


void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    if (GetFaultFlags() || motionBlocked) return;
    holding = false;
    goalPosition = _pos + context->config.motionParams.encoderHomeOffset;
}


bool Motor::Controller::SetPositionSetPointWithTime(int32_t _pos, float _time)
{
    int32_t deltaPos = abs(_pos - realPosition + context->config.motionParams.encoderHomeOffset);

    float pMax = (float) context->config.motionParams.ratedVelocityAcc * _time * _time / 4;
    if ((float) deltaPos > pMax)
    {
        context->config.motionParams.ratedVelocity = boardConfig.velocityLimit;
        SetPositionSetPoint(_pos);

        return false;
    } else
    {
        float vMax = _time * (float) context->config.motionParams.ratedVelocityAcc;
        vMax -= (float) context->config.motionParams.ratedVelocityAcc *
                (sqrtf(_time * _time - 4 * (float) deltaPos /
                                       (float) context->config.motionParams.ratedVelocityAcc));
        vMax /= 2;

        context->config.motionParams.ratedVelocity = (int32_t) vMax;
        SetPositionSetPoint(_pos);

        return true;
    }
}


void Motor::Controller::SetVelocitySetPoint(int32_t _vel)
{
    if (GetFaultFlags() || motionBlocked) return;
    if ((_vel >= -context->config.motionParams.ratedVelocity) &&
        (_vel <= context->config.motionParams.ratedVelocity))
    {
        goalVelocity = _vel;
    }
}


float Motor::Controller::GetPosition(bool _isLap)
{
    return _isLap ?
           (float) (realLapPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                  :
           (float) (realPosition - context->config.motionParams.encoderHomeOffset) /
           (float) (context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}


float Motor::Controller::GetVelocity()
{
    return (float) estVelocity / (float) context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}


float Motor::Controller::GetFocCurrent()
{
    return (float) focCurrent / 1000.f;
}


void Motor::Controller::SetCurrentSetPoint(int32_t _cur)
{
    if (GetFaultFlags() || motionBlocked) return;
    if (_cur > context->config.motionParams.ratedCurrent)
        goalCurrent = context->config.motionParams.ratedCurrent;
    else if (_cur < -context->config.motionParams.ratedCurrent)
        goalCurrent = -context->config.motionParams.ratedCurrent;
    else
        goalCurrent = _cur;
}


bool Motor::Controller::RequestCurrentFeedForward(float _amps, uint16_t _seq, uint16_t _leaseMs)
{
    if (!config || !std::isfinite(_amps) ||
        fabsf(_amps) * 1000.0f > config->feedForward.maxCurrent ||
        _leaseMs < config->feedForward.minLeaseMs ||
        _leaseMs > config->feedForward.maxLeaseMs ||
        _seq == ffRequestSeq ||
        !IsNewerSequence(_seq, lastFeedForwardSequence))
        return false;

    ffRequestWriteSeq++; // Odd: request is being written.
    ffRequestCurrent_mA = static_cast<int32_t>(_amps * 1000.0f);
    ffRequestSeq = _seq;
    ffRequestLeaseMs = _leaseMs;
    ffRequestWriteSeq++; // Even: request is complete.
    return true;
}


bool Motor::Controller::SetLocalCompensationParam(uint8_t _index, int32_t _value, bool _storeToEeprom)
{
    if (!config || _value < 0 || _index > 4)
        return false;

    const int32_t maximum = (_index == 1 || _index == 4) ? 3300 : 1000000;
    if (_value > maximum)
        return false;
    if (_index == 3 && _value < LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000)
        _value = LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000;

    LocalCompensationConfig_t* local = &config->localComp;
    switch (_index)
    {
        case 0: local->damping_mA_per_motorRevS = _value; break;
        case 1: local->coulomb_mA = _value; break;
        case 2: local->viscous_mA_per_motorRevS = _value; break;
        case 3: local->deadzoneSteps =
                    static_cast<int32_t>((static_cast<int64_t>(_value) *
                                          context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS + 500) / 1000);
            break;
        case 4: local->limit_mA = _value; break;
        default: return false;
    }

    const int64_t assist_mA_per_rev =
        static_cast<int64_t>(local->viscous_mA_per_motorRevS) -
        local->damping_mA_per_motorRevS;
    local->assistCoeffQ16 = static_cast<int32_t>(
        (assist_mA_per_rev * 65536 + context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2) /
        context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
    local->rampInvQ16 = local->deadzoneSteps > 0 ?
        static_cast<int32_t>((65536 + local->deadzoneSteps / 2) / local->deadzoneSteps) : 0;

    if (_storeToEeprom)
    {
        switch (_index)
        {
            case 0: boardConfig.localDamping_mA_per_motorRevS = _value; break;
            case 1: boardConfig.localCoulomb_mA = _value; break;
            case 2: boardConfig.localViscous_mA_per_motorRevS = _value; break;
            case 3: boardConfig.localDeadzone_motorRevS_x1000 = _value; break;
            case 4: boardConfig.localCompLimit_mA = _value; break;
            default: break;
        }
        // The five frames are one group; only the last frame commits EEPROM.
        if (_index == 4)
        {
            boardConfig.version = BOARD_CONFIG_VERSION;
            boardConfig.configStatus = CONFIG_COMMIT;
        }
    }
    return true;
}


bool Motor::Controller::RequestDragCurrent(float _amps, uint16_t _seq, uint16_t _leaseMs)
{
    if (!config || !std::isfinite(_amps) ||
        fabsf(_amps) * 1000.0f > config->feedForward.maxCurrent ||
        _leaseMs < config->feedForward.minLeaseMs ||
        _leaseMs > config->feedForward.maxLeaseMs ||
        _seq == dragRequestSeq ||
        !IsNewerSequence(_seq, lastDragSequence))
        return false;

    dragRequestWriteSeq++;
    dragRequestCurrent_mA = static_cast<int32_t>(_amps * 1000.0f);
    dragRequestSeq = _seq;
    dragRequestLeaseMs = _leaseMs;
    dragRequestWriteSeq++;
    return true;
}


void Motor::Controller::ClearFeedForward()
{
    feedForwardCurrent_mA = 0;
    feedForwardActive = false;
    feedForwardLeaseTicks = 0;
    feedForwardLimited = false;
    dragActive = false;
    dragLeaseTicks = 0;
    goalCurrent = softCurrent = 0;
}


int32_t Motor::Controller::GetFeedbackCurrent_mA() const
{
    return feedbackCurrent_mA;
}


int32_t Motor::Controller::GetFeedForwardCurrent_mA() const
{
    return feedForwardActive ? feedForwardCurrent_mA : 0;
}


int32_t Motor::Controller::GetTotalCurrent_mA() const
{
    return totalCurrent_mA;
}


uint16_t Motor::Controller::GetFeedForwardSequence() const
{
    return lastFeedForwardSequence;
}


uint16_t Motor::Controller::GetDragSequence() const
{
    return lastDragSequence;
}


bool Motor::Controller::IsFeedForwardLimited() const
{
    return feedForwardLimited;
}


uint16_t Motor::Controller::GetRemainingLeaseMs() const
{
    const uint32_t ticks = feedForwardLeaseTicks > dragLeaseTicks ?
                           feedForwardLeaseTicks : dragLeaseTicks;
    return static_cast<uint16_t>(ticks / context->motionPlanner.CONTROL_TICKS_PER_MS);
}


void Motor::Controller::ConsumeFeedForwardRequest()
{
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const uint32_t before = ffRequestWriteSeq;
        if (before & 1) continue;
        const int32_t mA = ffRequestCurrent_mA;
        const uint16_t seq = ffRequestSeq;
        const uint16_t lease = ffRequestLeaseMs;
        if (before != ffRequestWriteSeq) continue;
        if (seq != 0 && IsNewerSequence(seq, lastFeedForwardSequence))
        {
            lastFeedForwardSequence = seq;
            feedForwardCurrent_mA = mA;
            feedForwardActive = true;
            feedForwardLeaseTicks = static_cast<uint32_t>(lease) *
                                    context->motionPlanner.CONTROL_TICKS_PER_MS;
        }
        break;
    }

    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const uint32_t before = dragRequestWriteSeq;
        if (before & 1) continue;
        const int32_t mA = dragRequestCurrent_mA;
        const uint16_t seq = dragRequestSeq;
        const uint16_t lease = dragRequestLeaseMs;
        if (before != dragRequestWriteSeq) continue;
        if (seq != 0 && IsNewerSequence(seq, lastDragSequence))
        {
            lastDragSequence = seq;
            if (modeRunning != MODE_COMMAND_CURRENT)
                SetCtrlMode(MODE_COMMAND_CURRENT);
            context->motionPlanner.currentTracker.SetCurrentAcc(config->feedForward.dragCurrentAcc);
            SetCurrentSetPoint(mA);
            dragActive = true;
            dragLeaseTicks = static_cast<uint32_t>(lease) *
                             context->motionPlanner.CONTROL_TICKS_PER_MS;
        }
        break;
    }
}


void Motor::Controller::ReleaseDragLease()
{
    // A missed drag frame must fall back to the position hold, but must not
    // latch motionBlocked; otherwise later fresh drag frames stay rejected.
    dragActive = false;
    ClearFeedForward();
    RestoreCurrentAcceleration();
    if (!holding) holdRequested = true;
}


void Motor::Controller::RestoreCurrentAcceleration()
{
    context->motionPlanner.currentTracker.SetCurrentAcc(context->config.motionParams.ratedCurrentAcc);
}


void Motor::Controller::UpdateFeedForwardLease()
{
    if (feedForwardActive && feedForwardLeaseTicks)
    {
        --feedForwardLeaseTicks;
        if (!feedForwardLeaseTicks)
        {
            feedForwardActive = false;
            feedForwardCurrent_mA = 0;
        }
    }
    if (dragActive && dragLeaseTicks)
    {
        --dragLeaseTicks;
        if (!dragLeaseTicks) ReleaseDragLease();
    }
}


void Motor::Controller::SetDisable(bool _disable)
{
    goalDisable = _disable;
}


void Motor::Controller::SetBrake(bool _brake)
{
    goalBrake = _brake;

}


void Motor::Controller::ClearStallFlag()
{
    // Clearing always leaves the motor stopped and discards the old target.
    motionBlocked = true;
    clearFaultsRequested = true;
    ClearFeedForward();
}

void Motor::Controller::RequestHold()
{
    motionBlocked = true;
    ClearFeedForward();
    if (!holding) holdRequested = true;
}

void Motor::Controller::ReportControlOverrun()
{
    if (controlOverrunFlag) return;
    controlOverrunFlag = true;
    state = STATE_CONTROL_OVERRUN;
    RequestHold(); // Locally discard the target even if CAN cannot run yet.
}

void Motor::Controller::SetEnable(bool enable)
{
    if (!enable)
    {
        SetCtrlMode(MODE_STOP);
        ClearFeedForward();
        return;
    }
    if (GetFaultFlags() || clearFaultsRequested) return;
    motionBlocked = false;
    holding = false;
    holdRequested = true; // Capture the current pose on enable, not the old goal.
}

uint8_t Motor::Controller::GetFaultFlags() const
{
    return (overloadFlag ? 1 : 0) | (isStalled ? 2 : 0) |
           (!context->encoder || !context->encoder->IsCalibrated() ? 4 : 0) |
           (controlOverrunFlag ? 128 : 0);
}


int32_t Motor::Controller::CompensateAdvancedAngle(int32_t _vel)
{
    /*
     * The code is for DPS series sensors, need to measured and renew for TLE5012/MT6816.
     */

    int32_t compensate;

    if (_vel < 0)
    {
        if (_vel > -100000) compensate = 0;
        else if (_vel > -1300000) compensate = (((_vel + 100000) * 262) >> 20) - 0;
        else if (_vel > -2200000) compensate = (((_vel + 1300000) * 105) >> 20) - 300;
        else compensate = (((_vel + 2200000) * 52) >> 20) - 390;

        if (compensate < -430) compensate = -430;
    } else
    {
        if (_vel < 100000) compensate = 0;
        else if (_vel < 1300000) compensate = (((_vel - 100000) * 262) >> 20) + 0;
        else if (_vel < 2200000) compensate = (((_vel - 1300000) * 105) >> 20) + 300;
        else compensate = (((_vel - 2200000) * 52) >> 20) + 390;

        if (compensate > 430) compensate = 430;
    }

    return compensate;
}


void Motor::Controller::Init()
{
    requestMode = boardConfig.enableMotorOnBoot ? static_cast<Mode_t>(boardConfig.defaultMode) : MODE_STOP;

    modeRunning = MODE_STOP;
    state = STATE_STOP;
    motionBlocked = !boardConfig.enableMotorOnBoot;

    realLapPosition = 0;
    realLapPositionLast = 0;
    realPosition = 0;
    realPositionLast = 0;

    estVelocityIntegral = 0;
    estVelocity = 0;
    estLeadPosition = 0;
    estPosition = 0;
    estError = 0;

    goalPosition = context->config.motionParams.encoderHomeOffset;
    goalVelocity = 0;
    goalCurrent = 0;
    goalDisable = false;
    goalBrake = false;
    ffRequestWriteSeq = 0;
    ffRequestCurrent_mA = 0;
    ffRequestSeq = 0;
    ffRequestLeaseMs = 0;
    dragRequestWriteSeq = 0;
    dragRequestCurrent_mA = 0;
    dragRequestSeq = 0;
    dragRequestLeaseMs = 0;
    feedForwardCurrent_mA = 0;
    feedbackCurrent_mA = 0;
    totalCurrent_mA = 0;
    lastFeedForwardSequence = 0;
    lastDragSequence = 0;
    feedForwardLeaseTicks = 0;
    dragLeaseTicks = 0;
    feedForwardActive = false;
    dragActive = false;
    feedForwardLimited = false;

    softPosition = 0;
    softVelocity = 0;
    softCurrent = 0;
    softDisable = false;
    softBrake = false;
    softNewCurve = false;

    focPosition = 0;
    focCurrent = 0;

    stalledTime = 0;
    isStalled = false;

    overloadTime = 0;
    overloadFlag = false;
    controlOverrunFlag = false;

    config->pid.vError = 0;
    config->pid.vErrorLast = 0;
    config->pid.outputKp = 0;
    config->pid.outputKi = 0;
    config->pid.outputKd = 0;
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.output = 0;

    config->dce.pError = 0;
    config->dce.vError = 0;
    config->dce.outputKp = 0;
    config->dce.outputKi = 0;
    config->dce.outputKd = 0;
    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.output = 0;
}


void Motor::Controller::ApplyPosAsHomeOffset()
{
    context->config.motionParams.encoderHomeOffset = realPosition %
                                                     context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}


void Motor::Controller::AttachConfig(Motor::Controller::Config_t* _config)
{
    config = _config;
}


void Motor::Controller::ClearIntegral() const
{
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.outputKi = 0;

    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.outputKi = 0;
}


