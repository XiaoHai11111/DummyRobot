#include "Motor/motor.h"
#include "Motor/control_math.h"
#include "Motor/control_timing.h"
#include <climits>
#include <limits>
#include "configurations.h"
#include <cassert>
#include <iostream>

BoardConfig_t boardConfig{};
struct Encoder : EncoderBase {
    bool calibrated = true;
    bool Init() override { return true; }
    uint16_t UpdateAngle() override { return angleData.rectifiedAngle; }
    bool IsCalibrated() override { return calibrated; }
};
struct Driver : DriverBase {
    int current = 0;
    int peak = 0;
    void Init() override {}
    void SetFocCurrentVector(uint32_t, int32_t value) override { current = value; if (value > peak) peak = value; }
    void Sleep() override { current = 0; }
    void Brake() override { current = 0; }
    void SetTwoCoilsCurrent(uint16_t, uint16_t) override {}
};
void ticks(Motor &motor, int count) { while (count--) motor.Tick20kHz(); }
template<int32_t divisor> void compareIntegral() {
    const int64_t increments[] = {0,1,-1,127,-127,128,-128,129,-129,1023,-1023,1024,-1024,
                                  int64_t(INT32_MAX)*3200, -int64_t(INT32_MAX)*3200};
    for (int32_t start : {-3072000, -1000, 0, 1000, 3072000})
        for (int32_t rem : {-divisor+1, 0, divisor-1})
            for (int64_t inc : increments)
                for (int64_t pd : {-4000000, 0, 4000000}) {
                    // Golden behaviour of the previous runtime-divisor implementation.
                    int32_t expected = start, expectedRem = rem;
                    const int64_t acc = rem + inc;
                    int64_t next = start + acc / divisor;
                    if (next > 3072000) next = 3072000;
                    if (next < -3072000) next = -3072000;
                    if (!((pd+next > 3072000 && inc > 0) || (pd+next < -3072000 && inc < 0))) {
                        expected = next; expectedRem = acc % divisor;
                    }
                    int32_t value = start, remainder = rem;
                    IntegrateLimited<divisor>(value, remainder, inc, pd, 3000);
                    assert(value == expected && remainder == expectedRem);
                }
}
int main() {
    compareIntegral<128>(); compareIntegral<1024>();
    assert(CalculateLocalCompensationQ16(
               51200, 128, 100, 1024, 64, 200, 32768, 500) == 200);
    assert(CalculateLocalCompensationQ16(
               51200, 128, 100, 1024, 64, 200, 32768, 100) == 100);
    assert(CalculateLocalCompensationQ16(
               51200, 128, 100, 1024, 64, 200, 0, 500) == 0);
    ControlTimingStats timing;
    assert(!timing.Observe(3500, 3600));
    assert(!timing.Observe(3601, 3600));
    assert(!timing.Observe(3700, 3600));
    assert(!timing.Observe(3500, 3600)); // A good cycle resets consecutive overrun count.
    assert(!timing.Observe(3601, 3600));
    assert(!timing.Observe(3700, 3600));
    assert(timing.Observe(4000, 3600));
    assert(timing.maxCycles == 4000 && timing.lastCycles == 4000 && timing.overrunCount == 5);
    int32_t integral = 0, remainder = 0;
    for (int n = 0; n < 40000; ++n)
        IntegrateLimited<128>(integral, remainder, 300 * 3200, 3200000, 3000);
    assert(integral == 0); // P alone is saturated: no integral windup.
    integral = 1024000;
    IntegrateLimited<128>(integral, remainder, -128000, 4000000, 3000);
    assert(integral < 1024000); // Allow unwinding while still saturated.
    integral = 0;
    IntegrateLimited<128>(integral, remainder, 128000, 0, 3000);
    assert(integral == 1000); // Preserve integration for static loads.

    Motor motor; Encoder encoder; Driver driver;
    motor.AttachEncoder(&encoder); motor.AttachDriver(&driver); motor.controller->Init();
    motor.config.motionParams.ratedCurrent = 3000;
    motor.config.ctrlParams.dce.kp = 1000;
    motor.Tick20kHz();
    motor.controller->SetEnable(true); ticks(motor, 3);
    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
    motor.controller->SetPositionSetPoint(20000);
    ticks(motor, 50000);
    assert(motor.controller->GetFaultFlags() == 1);
    assert(motor.controller->state == Motor::STATE_OVERLOAD);
    assert(driver.peak == 3000);
    assert(driver.current == 0); // F1 cancels the stalled goal without waiting for F4.
    motor.controller->SetCtrlMode(Motor::MODE_STOP);
    ticks(motor, 3);
    assert(driver.current == 0);
    assert(motor.controller->GetFaultFlags() == 1); // STOP does not erase fault.
    motor.controller->RequestHold(); ticks(motor, 3);
    assert(driver.current == 0); // Old goal was discarded.
    motor.controller->SetPositionSetPoint(10000); ticks(motor, 100);
    assert(driver.current == 0); // A late position frame cannot cancel HOLD.
    motor.controller->ClearStallFlag(); ticks(motor, 3);
    assert(motor.controller->GetFaultFlags() == 0);
    assert(motor.controller->modeRunning == Motor::MODE_STOP);
    motor.controller->SetEnable(true); ticks(motor, 3);
    assert(motor.controller->modeRunning == Motor::MODE_COMMAND_POSITION);

    motor.controller->SetPositionSetPoint(20000); ticks(motor, 20);
    motor.controller->ReportControlOverrun();
    assert(motor.controller->GetFaultFlags() == 128);
    ticks(motor, 3);
    assert(motor.controller->state == Motor::STATE_CONTROL_OVERRUN && driver.current == 0);
    motor.controller->SetPositionSetPoint(10000); ticks(motor, 100);
    assert(driver.current == 0); // Late target cannot bypass a timing fault.
    motor.controller->SetEnable(true); ticks(motor, 3);
    assert(motor.controller->GetFaultFlags() == 128);
    motor.controller->ClearStallFlag(); ticks(motor, 3);
    assert(motor.controller->GetFaultFlags() == 0 && motor.controller->modeRunning == Motor::MODE_STOP);

    // Stall time must be continuous, not the sum of separated low-speed periods.
    motor.controller->ClearStallFlag(); ticks(motor, 3);
    motor.controller->SetEnable(true); ticks(motor, 3);
    motor.config.ctrlParams.stallProtectSwitch = true;
    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
    motor.controller->SetCurrentSetPoint(1000);
    ticks(motor, 12000);
    // A real motion period above the stall-speed threshold breaks continuity.
    for (int n = 0; n < 1000; ++n) { encoder.angleData.rectifiedAngle += 32; motor.Tick20kHz(); }
    ticks(motor, 12000);
    assert(!motor.controller->isStalled);
    ticks(motor, 10000);
    assert(motor.controller->isStalled);
    motor.controller->SetCtrlMode(Motor::MODE_STOP); ticks(motor, 3);
    assert(motor.controller->isStalled);
    motor.controller->ClearStallFlag(); ticks(motor, 3);
    assert(!motor.controller->isStalled);
    assert(motor.controller->modeRunning == Motor::MODE_STOP);
    encoder.calibrated = false;
    motor.controller->ClearStallFlag(); ticks(motor, 3);
    assert(motor.controller->GetFaultFlags() == 4); // Clearing cannot create calibration.
    // RAM-only feed-forward rejects bad packets and expires without a refresh.
    Motor ffMotor;
    Driver ffDriver;
    encoder.angleData.rectifiedAngle = 0;
    ffMotor.AttachEncoder(&encoder);
    ffMotor.AttachDriver(&ffDriver);
    ffMotor.controller->Init();
    encoder.calibrated = true;
    ffMotor.config.motionParams.ratedCurrent = 3000;
    ffMotor.config.ctrlParams.dce.kp = 1000;
    ffMotor.Tick20kHz();
    ffMotor.controller->SetEnable(true); ticks(ffMotor, 3);
    ffMotor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
    ffMotor.controller->SetPositionSetPoint(0);
    assert(ffMotor.controller->RequestCurrentFeedForward(0.5f, 1, 100));
    assert(!ffMotor.controller->RequestCurrentFeedForward(0.5f, 1, 100));
    assert(!ffMotor.controller->RequestCurrentFeedForward(0.5f, 2, 0));
    ffMotor.Tick20kHz();
    assert(ffMotor.controller->GetFeedForwardSequence() == 1);
    assert(ffMotor.controller->GetFeedForwardCurrent_mA() == 500);
    assert(ffDriver.current == 500);
    ticks(ffMotor, 2000);
    assert(ffMotor.controller->GetFeedForwardCurrent_mA() == 0);
    assert(ffDriver.current == 0);

    // Stage 2 parameters use F1 BoardConfig units and reject invalid indices/values.
    assert(ffMotor.controller->SetLocalCompensationParam(0, 100));
    assert(ffMotor.controller->SetLocalCompensationParam(1, 200));
    assert(ffMotor.controller->SetLocalCompensationParam(2, 300));
    assert(ffMotor.controller->SetLocalCompensationParam(3, 10));
    assert(ffMotor.controller->SetLocalCompensationParam(4, 800));
    assert(!ffMotor.controller->SetLocalCompensationParam(5, 100));
    assert(!ffMotor.controller->SetLocalCompensationParam(0, -100));
    assert(!ffMotor.controller->SetLocalCompensationParam(4, 4000));
    assert(ffMotor.controller->SetLocalCompensationParam(4, 800, true));
    assert(boardConfig.localCompLimit_mA == 800);
    assert(boardConfig.version == BOARD_CONFIG_VERSION);
    assert(boardConfig.configStatus == CONFIG_COMMIT);
    boardConfig.configStatus = CONFIG_OK;

    int32_t reportedLocal[5]{};
    ffMotor.controller->GetLocalCompensationParams(reportedLocal);
    assert(reportedLocal[0] == 100 && reportedLocal[1] == 200 &&
           reportedLocal[2] == 300 &&
           reportedLocal[3] ==
               Motor::Controller::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000 &&
           reportedLocal[4] == 800);

    // Legacy #LOCALCOMP remains active without the new opt-in. Drag current is
    // leased; expiry returns to position hold.
    assert(ffMotor.controller->RequestDragCurrent(0.3f, 1, 500));
    ticks(ffMotor, 4000); // Current mode switch and 2 mA/ms current ramp.
    assert(ffMotor.controller->modeRunning == Motor::MODE_COMMAND_CURRENT);
    assert(ffDriver.current == 300);
    ticks(ffMotor, 10000);
    ticks(ffMotor, 3);
    assert(ffMotor.controller->modeRunning == Motor::MODE_COMMAND_POSITION);
    assert(ffDriver.current == 0);

    // A later fresh drag request must work without re-enabling the motor.
    assert(ffMotor.controller->RequestDragCurrent(0.3f, 2, 500));
    ticks(ffMotor, 4000);
    assert(ffMotor.controller->modeRunning == Motor::MODE_COMMAND_CURRENT);
    assert(ffDriver.current == 300);

    // Stage 2 local compensation uses F1 velocity and only acts during a lease.
    // One encoder step per tick is below the clamped deadzone and must be ignored.
    for (int i = 0; i < 200; ++i)
    {
        ++encoder.angleData.rectifiedAngle;
        ffMotor.Tick20kHz();
    }
    assert(ffDriver.current == 300);

    // Above the deadzone the fitted correction is applied.
    for (int i = 0; i < 200; ++i)
    {
        encoder.angleData.rectifiedAngle += 4;
        ffMotor.Tick20kHz();
    }
    assert(ffDriver.current > 300);

    // A deadzone above the measured speed disables the correction.
    assert(ffMotor.controller->SetLocalCompensationParam(3, 100000));
    for (int i = 0; i < 200; ++i)
    {
        encoder.angleData.rectifiedAngle += 4;
        ffMotor.Tick20kHz();
    }
    assert(ffDriver.current == 300);

    // The correction itself is clamped and never exceeds the motor current limit.
    // A requested deadzone of 10 is clamped to the safe minimum.
    assert(ffMotor.controller->SetLocalCompensationParam(3, 10));
    assert(ffMotor.controller->config->localComp.deadzoneSteps ==
           Motor::Controller::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000 * 51200 / 1000);
    assert(ffMotor.controller->SetLocalCompensationParam(0, 1000000));
    assert(ffMotor.controller->SetLocalCompensationParam(4, 500));
    for (int i = 0; i < 200; ++i)
    {
        encoder.angleData.rectifiedAngle += 4;
        ffMotor.Tick20kHz();
    }
    assert(ffDriver.current == -200);

    // Opt-in speed-only mode keeps old 0x0B frames torque-neutral even when the
    // legacy #LOCALCOMP parameters are nonzero.
    Motor plainCurrentMotor;
    Driver plainCurrentDriver;
    encoder.angleData.rectifiedAngle = 0;
    plainCurrentMotor.AttachEncoder(&encoder);
    plainCurrentMotor.AttachDriver(&plainCurrentDriver);
    plainCurrentMotor.controller->Init();
    encoder.calibrated = true;
    plainCurrentMotor.config.motionParams.ratedCurrent = 3000;
    plainCurrentMotor.Tick20kHz();
    assert(plainCurrentMotor.controller->SetLocalCompOptIn(true, 0, 2, 0));
    plainCurrentMotor.controller->SetEnable(true);
    ticks(plainCurrentMotor, 3);
    plainCurrentMotor.controller->SetLocalCompensationParam(0, 0);
    plainCurrentMotor.controller->SetLocalCompensationParam(1, 500);
    plainCurrentMotor.controller->SetLocalCompensationParam(2, 0);
    plainCurrentMotor.controller->SetLocalCompensationParam(3, 20);
    plainCurrentMotor.controller->SetLocalCompensationParam(4, 500);
    assert(plainCurrentMotor.controller->RequestDragCurrent(0.3f, 1, 1000));
    ticks(plainCurrentMotor, 4000);
    for (int i = 0; i < 1024; ++i)
    {
        encoder.angleData.rectifiedAngle += 4;
        plainCurrentMotor.Tick20kHz();
    }
    assert(plainCurrentDriver.current == 300);
    assert(plainCurrentMotor.controller->GetLocalCompensationCurrent_mA() == 0);

    // Opt-in low-speed feedback uses a 256-tick (12.8 ms), non-overlapping
    // window. Direction confidence advances once per completed window.
    Motor lowSpeedMotor;
    Driver lowSpeedDriver;
    encoder.angleData.rectifiedAngle = 51100;
    encoder.calibrated = true;
    lowSpeedMotor.AttachEncoder(&encoder);
    lowSpeedMotor.AttachDriver(&lowSpeedDriver);
    lowSpeedMotor.controller->Init();
    lowSpeedMotor.config.motionParams.ratedCurrent = 3000;
    lowSpeedMotor.Tick20kHz();
    assert(!lowSpeedMotor.controller->IsLocalCompOptInEnabled());
    lowSpeedMotor.controller->SetEnable(true);
    ticks(lowSpeedMotor, 3);
    assert(lowSpeedMotor.controller->SetLocalCompOptIn(true, 0, 0x1234, 0));
    for (int tick = 0; tick < Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS; ++tick)
    {
        if ((tick & 7) == 0)
        {
            ++encoder.angleData.rectifiedAngle;
            if (encoder.angleData.rectifiedAngle == lowSpeedMotor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                encoder.angleData.rectifiedAngle = 0;
        }
        lowSpeedMotor.Tick20kHz();
    }
    assert(lowSpeedMotor.controller->IsAccurateVelocityReady());
    assert(lowSpeedMotor.controller->GetMotionDirection() == 0);
    for (int tick = 1;
         tick < Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS *
                    Motor::Controller::LOCAL_COMP_DIRECTION_CONFIRM_WINDOWS;
         ++tick)
    {
        if ((tick & 7) == 0)
        {
            ++encoder.angleData.rectifiedAngle;
            if (encoder.angleData.rectifiedAngle ==
                lowSpeedMotor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                encoder.angleData.rectifiedAngle = 0;
        }
        lowSpeedMotor.Tick20kHz();
    }
    assert(lowSpeedMotor.controller->GetVelocity() *
               (float) lowSpeedMotor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS ==
           2500.0f);
    assert(lowSpeedMotor.controller->GetMotionDirection() == 1);

    // A same-direction speed between the exit and enter thresholds keeps the
    // direction. Crossing the exit threshold on a fresh window releases it.
    for (int tick = 0; tick < Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS; ++tick)
    {
        if ((tick & 15) == 0) ++encoder.angleData.rectifiedAngle;
        lowSpeedMotor.Tick20kHz();
    }
    lowSpeedMotor.Tick20kHz(); // Complete the 256-tick sample boundary.
    assert(lowSpeedMotor.controller->GetMotionDirection() == 1);
    for (int tick = 0; tick < Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS; ++tick)
    {
        if ((tick & 31) == 0) ++encoder.angleData.rectifiedAngle;
        lowSpeedMotor.Tick20kHz();
    }
    assert(lowSpeedMotor.controller->GetMotionDirection() == 0);

    // Very high speed and a deliberately oversized viscous term must saturate
    // cleanly at the new assist cap without overflowing the 20 kHz integer path.
    Motor highSpeedMotor;
    Driver highSpeedDriver;
    encoder.angleData.rectifiedAngle = 0;
    highSpeedMotor.AttachEncoder(&encoder);
    highSpeedMotor.AttachDriver(&highSpeedDriver);
    highSpeedMotor.controller->Init();
    highSpeedMotor.config.motionParams.ratedCurrent = 3000;
    highSpeedMotor.Tick20kHz();
    assert(highSpeedMotor.controller->SetLocalCompOptIn(true, 100, 0x5678, 500));
    assert(highSpeedMotor.controller->SetLocalCompensationParam(0, 0));
    assert(highSpeedMotor.controller->SetLocalCompensationParam(1, 0));
    assert(highSpeedMotor.controller->SetLocalCompensationParam(2, 1000000));
    assert(highSpeedMotor.controller->SetLocalCompensationParam(3, 20));
    assert(highSpeedMotor.controller->SetLocalCompensationParam(4, 500));
    highSpeedMotor.controller->SetEnable(true);
    ticks(highSpeedMotor, 3);
    assert(highSpeedMotor.controller->RequestExtendedDragCurrent(
        0, 600, 1, 1000));
    for (int tick = 0; tick < 1024; ++tick)
    {
        encoder.angleData.rectifiedAngle =
            (encoder.angleData.rectifiedAngle + 255) % 51200;
        highSpeedMotor.Tick20kHz();
    }
    assert(highSpeedMotor.controller->GetMotionDirection() == 1);
    assert(highSpeedMotor.controller->GetLocalCompensationCurrent_mA() == 500);
    assert(highSpeedDriver.current == 500);

    // E/O are applied in motor coordinates relative to configured home. The
    // correction ramps in only after direction confidence and fades to zero
    // at rest, while the underlying current stream remains qualified by drag.
    Motor harmonicMotor;
    Driver harmonicDriver;
    encoder.angleData.rectifiedAngle = 10000;
    encoder.calibrated = true;
    harmonicMotor.AttachEncoder(&encoder);
    harmonicMotor.AttachDriver(&harmonicDriver);
    harmonicMotor.controller->Init();
    harmonicMotor.config.motionParams.ratedCurrent = 3000;
    harmonicMotor.config.motionParams.encoderHomeOffset = 10000;
    harmonicMotor.Tick20kHz();
    assert(harmonicMotor.controller->SetLocalCompOptIn(true, 100, 0x55AA, 500));
    assert(harmonicMotor.controller->SetMotorDomainHarmonics(0, 400, 0, 200));
    assert(harmonicMotor.controller->SetMotorDomainHarmonics(11, -22, 33, -44, true));
    assert(boardConfig.harmonicESin_mA == 11 && boardConfig.harmonicECos_mA == -22 &&
           boardConfig.harmonicOSin_mA == 33 && boardConfig.harmonicOCos_mA == -44);
    assert(boardConfig.configStatus == CONFIG_COMMIT);
    boardConfig.configStatus = CONFIG_OK;
    assert(harmonicMotor.controller->SetMotorDomainHarmonics(0, 400, 0, 200));
    int16_t checkESin, checkECos, checkOSin, checkOCos;
    assert(!harmonicMotor.controller->SetMotorDomainHarmonics(
        INT16_MIN, 400, 0, 200));
    assert(!harmonicMotor.controller->SetMotorDomainHarmonics(
        0, INT16_MAX, 0, 200));
    assert(!harmonicMotor.controller->SetMotorDomainHarmonics(
        0, 400, -501, 200));
    assert(!harmonicMotor.controller->SetMotorDomainHarmonics(
        0, 400, 0, 501));
    harmonicMotor.controller->GetMotorDomainHarmonics(
        checkESin, checkECos, checkOSin, checkOCos);
    assert(checkESin == 0 && checkECos == 400 &&
           checkOSin == 0 && checkOCos == 200);
    assert(harmonicMotor.controller->SetMotorDomainHarmonics(
        -500, 500, -500, 500));
    harmonicMotor.controller->GetMotorDomainHarmonics(
        checkESin, checkECos, checkOSin, checkOCos);
    assert(checkESin == -500 && checkECos == 500 &&
           checkOSin == -500 && checkOCos == 500);
    assert(harmonicMotor.controller->SetMotorDomainHarmonics(0, 400, 0, 200));
    assert(harmonicMotor.controller->SetLocalCompensationParam(0, 0));
    assert(harmonicMotor.controller->SetLocalCompensationParam(1, 0));
    assert(harmonicMotor.controller->SetLocalCompensationParam(2, 0));
    assert(harmonicMotor.controller->SetLocalCompensationParam(3, 20));
    assert(harmonicMotor.controller->SetLocalCompensationParam(4, 500));
    harmonicMotor.controller->SetEnable(true);
    ticks(harmonicMotor, 3);
    assert(harmonicMotor.controller->RequestExtendedDragCurrent(
        0, 600, 1, 1000));
    for (int tick = 0; tick < 1024; ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angleData.rectifiedAngle;
        harmonicMotor.Tick20kHz();
    }
    assert(harmonicMotor.controller->modeRunning == Motor::MODE_COMMAND_CURRENT);
    assert(harmonicMotor.controller->GetMotionDirection() == 1);
    assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() > 400);
    assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() <= 500);
    assert(harmonicDriver.current ==
           harmonicMotor.controller->GetLocalCompensationCurrent_mA());

    ticks(harmonicMotor, Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS +
                             20);
    assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() == 0);
    assert(harmonicDriver.current == 0);

    assert(harmonicMotor.controller->SetMotorDomainHarmonics(0, 0, 0, 500));
    assert(harmonicMotor.controller->RequestExtendedDragCurrent(
        0, 600, 2, 1000));
    for (int tick = 0; tick < Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS; ++tick)
    {
        if ((tick & 3) == 0)
        {
            if (encoder.angleData.rectifiedAngle == 0)
                encoder.angleData.rectifiedAngle =
                    harmonicMotor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS - 1;
            else
                --encoder.angleData.rectifiedAngle;
        }
        harmonicMotor.Tick20kHz();
        assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() >= 0);
    }
    assert(harmonicMotor.controller->GetMotionDirection() == 0);
    ticks(harmonicMotor, 20);
    assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() == 0);
    for (int tick = 20; tick <
         Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS *
             Motor::Controller::LOCAL_COMP_DIRECTION_CONFIRM_WINDOWS;
         ++tick)
    {
        if ((tick & 3) == 0)
        {
            if (encoder.angleData.rectifiedAngle == 0)
                encoder.angleData.rectifiedAngle =
                    harmonicMotor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS - 1;
            else
                --encoder.angleData.rectifiedAngle;
        }
        harmonicMotor.Tick20kHz();
    }
    assert(harmonicMotor.controller->GetMotionDirection() == -1);
    ticks(harmonicMotor, 200);
    assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() < -400);
    harmonicMotor.controller->SetCtrlMode(Motor::MODE_STOP);
    ticks(harmonicMotor, 1);
    assert(harmonicMotor.controller->GetLocalCompensationCurrent_mA() == 0);
    assert(!harmonicMotor.controller->IsAccurateVelocityReady());
    assert(harmonicMotor.controller->GetMotionDirection() == 0);
    assert(harmonicDriver.current == 0);

    std::cout << "F1 controller low-speed, direction, harmonic and drag tests passed\n";
}
