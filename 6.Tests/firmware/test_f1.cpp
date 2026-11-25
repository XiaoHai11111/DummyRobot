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

    // Drag current is leased; expiry returns to position hold.
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

    // Without a drag lease, the same nonzero parameters must not change current mode.
    Motor plainCurrentMotor;
    Driver plainCurrentDriver;
    encoder.angleData.rectifiedAngle = 0;
    plainCurrentMotor.AttachEncoder(&encoder);
    plainCurrentMotor.AttachDriver(&plainCurrentDriver);
    plainCurrentMotor.controller->Init();
    encoder.calibrated = true;
    plainCurrentMotor.config.motionParams.ratedCurrent = 3000;
    plainCurrentMotor.Tick20kHz();
    plainCurrentMotor.controller->SetEnable(true);
    ticks(plainCurrentMotor, 3);
    plainCurrentMotor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
    plainCurrentMotor.controller->SetCurrentSetPoint(300);
    ticks(plainCurrentMotor, 4000);
    plainCurrentMotor.controller->SetLocalCompensationParam(1, 500);
    plainCurrentMotor.controller->SetLocalCompensationParam(4, 500);
    for (int i = 0; i < 200; ++i)
    {
        ++encoder.angleData.rectifiedAngle;
        plainCurrentMotor.Tick20kHz();
    }
    assert(plainCurrentDriver.current == 300);

    std::cout << "F1 controller fault, hold, feed-forward and drag tests passed\n";
}
