#include "common_inc.h"
#include "configurations.h"
#include <array>
#include <cstring>
#include <cassert>
#include <limits>
#include <iostream>
#include "Motor/control_timing.h"
#include "can_command_filter.h"
ControlTimingStats controlTimingStats;
Motor motor;
EncoderCalibrator encoderCalibrator;
BoardConfig_t boardConfig{};
std::array<uint8_t, 8> reply{};
unsigned replies = 0, lastId = 0;
void HAL_NVIC_SystemReset() {}
void CAN_Send(CAN_TxHeaderTypeDef* header, uint8_t* data) {
    ++replies; lastId = header->StdId; memcpy(reply.data(), data, 8);
}
struct Encoder : EncoderBase {
    bool calibrated = true;
    uint16_t angle = 0;
    bool Init() override { return true; }
    uint16_t UpdateAngle() override {
        angleData.rectifiedAngle = angle;
        return angle;
    }
    bool IsCalibrated() override { return calibrated; }
} encoder;
struct Driver : DriverBase {
    int current = 0;
    void Init() override {}
    void SetFocCurrentVector(uint32_t, int32_t value) override { current = value; }
    void Sleep() override { current = 0; }
    void Brake() override { current = 0; }
    void SetTwoCoilsCurrent(uint16_t, uint16_t) override {}
} driver;
void ticks(Motor& motor, int count) { while (count--) motor.Tick20kHz(); }
int main() {
    // Boot inquiry before encoder attachment is valid, and reports not calibrated.
    uint8_t data[8]{};
    boardConfig.canNodeId = 2;
    for (uint8_t command : {uint8_t{0x0D}, uint8_t{0x0E}, uint8_t{0x0F},
                            uint8_t{0x10}, uint8_t{0x27}, uint8_t{0x28},
                            uint8_t{0x29}, uint8_t{0x2A}, uint8_t{0x2B},
                            uint8_t{0x2C}})
    {
        assert(!CanRxCommandAccepted(2, true, true, 0, command));
        assert(CanRxCommandAccepted(2, true, true, 2, command));
    }
    for (uint8_t command : {uint8_t{0x01}, uint8_t{0x08}, uint8_t{0x09}})
        assert(CanRxCommandAccepted(2, true, true, 0, command));
    assert(!CanRxCommandAccepted(2, false, true, 2, 0x0F));
    assert(!CanRxCommandAccepted(2, true, false, 2, 0x0F));
    assert(!CanRxCommandAccepted(2, true, true, 1, 0x0F));
    OnCanCmd(0x23, data, 0);
    assert(lastId == (2U << 7 | 0x23) && reply[6] == 4 && reply[7] == 0xA1);
    motor.AttachEncoder(&encoder);
    motor.AttachDriver(&driver);
    boardConfig.velocityLimit = 2 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    for (uint8_t cmd : {1,3,4,5,6,7,0x11,0x12,0x13,0x14,0x1B}) {
        const auto count = replies;
        OnCanCmd(cmd, data, 0);
        assert(replies == count);
    }
    float position = 1, speed = 5;
    memcpy(data, &position, 4); memcpy(data + 4, &speed, 4);
    OnCanCmd(0x07, data, 8);
    assert(motor.config.motionParams.ratedVelocity == boardConfig.velocityLimit);
    assert(reply[7] == 0xA1 && reply[6] == 0);
    const auto previous = replies;
    speed = std::numeric_limits<float>::quiet_NaN(); memcpy(data + 4, &speed, 4);
    OnCanCmd(0x07, data, 8); assert(replies == previous);
    speed = -1; memcpy(data + 4, &speed, 4);
    OnCanCmd(0x06, data, 8); assert(replies == previous);
    controlTimingStats.Observe(1500, 3240);
    controlTimingStats.Observe(70000, 3240);
    OnCanCmd(0x25, data, 0);
    uint16_t last, peak; uint32_t overruns;
    memcpy(&last, reply.data(), 2); memcpy(&peak, reply.data()+2, 2); memcpy(&overruns, reply.data()+4, 4);
    assert(lastId == ((2U << 7) | 0x25) && last == 65535 && peak == 65535 && overruns == 1);
    motor.controller->ReportControlOverrun();
    OnCanCmd(0x23, data, 8); assert(reply[5] == 6 && reply[6] == 128);
    motor.controller->Init();
    motor.controller->isStalled = true;
    OnCanCmd(0x23, data, 8); assert(reply[6] == 2 && reply[7] == 0xA1);
    encoder.calibrated = false;
    OnCanCmd(0x23, data, 8); assert(reply[6] == 6);
    encoder.calibrated = true;
    motor.controller->Init();
    motor.controller->SetCtrlMode(Motor::MODE_STOP);
    motor.Tick20kHz(); // Consume the one-time first-call encoder initialization.

    // Stage 2 local compensation uses F1 BoardConfig fixed-point integers.
    int32_t localValue = 250;
    memcpy(data, &localValue, 4);
    data[4] = 2;
    data[5] = 1;
    const auto localCount = replies;
    OnCanCmd(0x0C, data, 4);
    assert(replies == localCount);
    OnCanCmd(0x0C, data, 8);
    assert(motor.controller->config->localComp.viscous_mA_per_motorRevS == 250);
    assert(boardConfig.localViscous_mA_per_motorRevS == 250);
    assert(boardConfig.configStatus == CONFIG_RESTORE); // First frames do not commit flash.

    data[4] = 5;
    OnCanCmd(0x0C, data, 8);
    assert(motor.controller->config->localComp.viscous_mA_per_motorRevS == 250);

    data[4] = 4;
    localValue = 800;
    memcpy(data, &localValue, 4);
    OnCanCmd(0x0C, data, 8);
    assert(motor.controller->config->localComp.limit_mA == 800);
    assert(boardConfig.localCompLimit_mA == 800);
    assert(boardConfig.configStatus == CONFIG_COMMIT);
    boardConfig.configStatus = CONFIG_OK;

    localValue = 4000;
    memcpy(data, &localValue, 4);
    OnCanCmd(0x0C, data, 8);
    assert(motor.controller->config->localComp.limit_mA == 800);

    float ff = 0.4f;
    uint16_t ffSeq = 1, lease = 100;
    memcpy(data, &ff, 4);
    memcpy(data + 4, &ffSeq, 2);
    memcpy(data + 6, &lease, 2);
    const auto before = replies;
    OnCanCmd(0x0A, data, 4); assert(replies == before);
    OnCanCmd(0x0A, data, 8); assert(replies == before);
    motor.Tick20kHz();
    assert(motor.controller->GetFeedForwardSequence() == 1);
    assert(motor.controller->GetFeedForwardCurrent_mA() == 400);
    assert(motor.controller->modeRunning == Motor::MODE_STOP);
    ffSeq = 1; memcpy(data + 4, &ffSeq, 2);
    motor.Tick20kHz();
    assert(motor.controller->GetFeedForwardSequence() == 1);
    ff = std::numeric_limits<float>::quiet_NaN();
    memcpy(data, &ff, 4);
    OnCanCmd(0x0B, data, 8);
    motor.Tick20kHz();
    assert(motor.controller->GetDragSequence() == 0);
    OnCanCmd(0x26, data, 0);
    assert(lastId == ((2U << 7) | 0x26) && reply[7] == 0xA2);
    assert(!motor.controller->RequestExtendedDragCurrent(0, 600, 1, 1000));

    // Motor-domain harmonics are signed little-endian int16 values, limited
    // per coefficient and read back without changing per-node RAM.
    int16_t eSin = -500, eCos = 400, oSin = -300, oCos = 200;
    memcpy(data, &eSin, 2); memcpy(data + 2, &eCos, 2);
    memcpy(data + 4, &oSin, 2); memcpy(data + 6, &oCos, 2);
    const auto harmonicCount = replies;
    OnCanCmd(0x0D, data, 4);
    assert(replies == harmonicCount);
    OnCanCmd(0x0D, data, 8);
    assert(replies == harmonicCount);
    OnCanCmd(0x28, data, 0);
    int16_t readESin, readECos, readOSin, readOCos;
    memcpy(&readESin, reply.data(), 2); memcpy(&readECos, reply.data() + 2, 2);
    memcpy(&readOSin, reply.data() + 4, 2); memcpy(&readOCos, reply.data() + 6, 2);
    assert(lastId == ((2U << 7) | 0x28));
    assert(readESin == eSin && readECos == eCos &&
           readOSin == oSin && readOCos == oCos);
    int16_t limits[4] = {-500, 500, -500, 500};
    memcpy(data, limits, sizeof(limits));
    OnCanCmd(0x0D, data, 8);
    OnCanCmd(0x28, data, 0);
    memcpy(&readESin, reply.data(), 2); memcpy(&readECos, reply.data() + 2, 2);
    memcpy(&readOSin, reply.data() + 4, 2); memcpy(&readOCos, reply.data() + 6, 2);
    assert(readESin == limits[0] && readECos == limits[1] &&
           readOSin == limits[2] && readOCos == limits[3]);
    int16_t extremes[4] = {INT16_MIN, INT16_MAX, -501, 501};
    memcpy(data, extremes, sizeof(extremes));
    OnCanCmd(0x0D, data, 8);
    OnCanCmd(0x28, data, 0);
    memcpy(&readESin, reply.data(), 2); memcpy(&readECos, reply.data() + 2, 2);
    memcpy(&readOSin, reply.data() + 4, 2); memcpy(&readOCos, reply.data() + 6, 2);
    assert(readESin == limits[0] && readECos == limits[1] &&
           readOSin == limits[2] && readOCos == limits[3]);
    memcpy(data, &eSin, 2); memcpy(data + 2, &eCos, 2);
    memcpy(data + 4, &oSin, 2); memcpy(data + 6, &oCos, 2);
    OnCanCmd(0x0D, data, 8);

    // 0x10 stores the same four coefficients in BoardConfig; 0x29 reports the
    // effective values, including the five local-compensation parameters.
    const auto storedCount = replies;
    OnCanCmd(0x10, data, 4);
    assert(replies == storedCount);
    OnCanCmd(0x10, data, 8);
    assert(replies == storedCount);
    assert(boardConfig.harmonicESin_mA == eSin && boardConfig.harmonicECos_mA == eCos &&
           boardConfig.harmonicOSin_mA == oSin && boardConfig.harmonicOCos_mA == oCos);
    assert(boardConfig.configStatus == CONFIG_COMMIT);
    boardConfig.configStatus = CONFIG_OK;

    data[0] = 2;
    OnCanCmd(0x2B, data, 1);
    assert(lastId == ((2U << 7) | 0x2B));
    int32_t reportedLimit;
    memcpy(&reportedLimit, reply.data(), 4);
    assert(reportedLimit == 800);
    data[0] = 3;
    OnCanCmd(0x2C, data, 1);
    int16_t reportedESin, reportedECos, reportedOSin, reportedOCos;
    memcpy(&reportedESin, reply.data(), 2);
    memcpy(&reportedECos, reply.data() + 2, 2);
    memcpy(&reportedOSin, reply.data() + 4, 2);
    assert(reportedESin == eSin && reportedECos == eCos && reportedOSin == oSin);
    data[0] = 0;

    // Version 2 opt-in carries gain/cap and reports the exact latest write
    // sequence. Version 1, invalid ranges, and unsafe disable frames are ignored.
    OnCanCmd(0x27, data, 0);
    assert(lastId == ((2U << 7) | 0x27));
    assert(reply[0] == 0xB4 && reply[1] == 2 && reply[2] == 0 && reply[3] == 0);
    data[0] = 0xB4; data[1] = 1; data[2] = 1; data[3] = 100;
    uint16_t optInSeq = 0x1234;
    uint16_t assistCap = 500;
    memcpy(data + 4, &optInSeq, 2); memcpy(data + 6, &assistCap, 2);
    OnCanCmd(0x0E, data, 8);
    assert(!motor.controller->IsLocalCompOptInEnabled());
    data[0] = 0xB4;
    data[1] = 2;
    data[3] = 101;
    OnCanCmd(0x0E, data, 8);
    assert(!motor.controller->IsLocalCompOptInEnabled());
    data[3] = 100;
    assistCap = 501;
    memcpy(data + 6, &assistCap, 2);
    OnCanCmd(0x0E, data, 8);
    assert(!motor.controller->IsLocalCompOptInEnabled());
    data[2] = 0;
    data[3] = 0;
    assistCap = 0;
    memcpy(data + 6, &assistCap, 2);
    OnCanCmd(0x0E, data, 8);
    assert(!motor.controller->IsLocalCompOptInEnabled());

    data[2] = 1;
    data[3] = 100;
    assistCap = 500;
    memcpy(data + 6, &assistCap, 2);
    OnCanCmd(0x0E, data, 8);
    assert(motor.controller->IsLocalCompOptInEnabled());
    assert(motor.controller->GetLocalCompGainPercent() == 100);
    assert(motor.controller->GetLocalCompAssistCap_mA() == 500);
    uint16_t zeroSequence = 0;
    data[2] = 1;
    data[3] = 100;
    memcpy(data + 4, &zeroSequence, 2);
    OnCanCmd(0x0E, data, 8);
    assert(motor.controller->GetLocalCompSequence() == 0x1234);
    memcpy(data + 4, &optInSeq, 2);
    OnCanCmd(0x27, data, 0);
    uint16_t statusSeq;
    memcpy(&statusSeq, reply.data() + 6, 2);
    assert(reply[1] == 2 && reply[2] == 1 && statusSeq == optInSeq);

    motor.config.motionParams.ratedCurrent = 3000;
    assert(motor.controller->SetLocalCompensationParam(0, 0));
    assert(motor.controller->SetLocalCompensationParam(1, 500));
    assert(motor.controller->SetLocalCompensationParam(2, 0));
    assert(motor.controller->SetLocalCompensationParam(3, 20));
    assert(motor.controller->SetLocalCompensationParam(4, 500));
    motor.controller->SetEnable(true);
    ticks(motor, 3);
    for (int tick = 0; tick <
         Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS *
             Motor::Controller::LOCAL_COMP_DIRECTION_CONFIRM_WINDOWS;
         ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angle;
        motor.Tick20kHz();
    }
    OnCanCmd(0x27, data, 0);
    assert((reply[2] & 3) == 3 && static_cast<int8_t>(reply[3]) == 1);

    // In opt-in mode an old 0x0B stream remains legacy torque-neutral even
    // though the old #LOCALCOMP parameters and harmonics are nonzero.
    assert(motor.controller->RequestDragCurrent(0.0f, 1, 1000));
    for (int tick = 0; tick <
         Motor::Controller::LOCAL_COMP_SPEED_WINDOW_TICKS *
             Motor::Controller::LOCAL_COMP_DIRECTION_CONFIRM_WINDOWS;
         ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angle;
        motor.Tick20kHz();
    }
    assert(driver.current == 0);
    assert(motor.controller->GetLocalCompensationCurrent_mA() == 0);

    const auto sendExtended = [&](int16_t baseCurrent, uint16_t totalLimit,
                                  uint16_t sequence, uint16_t leaseFlags)
    {
        memcpy(data, &baseCurrent, 2);
        memcpy(data + 2, &totalLimit, 2);
        memcpy(data + 4, &sequence, 2);
        memcpy(data + 6, &leaseFlags, 2);
        OnCanCmd(0x0F, data, 8);
    };

    // Only an addressed 0x0F frame activates the new assist.
    sendExtended(0, 600, 2, 1000);
    for (int tick = 0; tick < 1024; ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angle;
        motor.Tick20kHz();
    }
    OnCanCmd(0x27, data, 0);
    int16_t actualAssist;
    memcpy(&actualAssist, reply.data() + 4, 2);
    assert(reply[2] == 3 && static_cast<int8_t>(reply[3]) == 1);
    assert(actualAssist > 0 && actualAssist <= 500);

    // CAN STOP clears the final torque and session-local velocity/direction
    // state on the next 20 kHz control tick.
    uint32_t enableCommand = 0;
    memcpy(data, &enableCommand, sizeof(enableCommand));
    OnCanCmd(0x01, data, 4);
    ticks(motor, 1);
    assert(driver.current == 0);
    assert(motor.controller->GetLocalCompensationCurrent_mA() == 0);
    assert(!motor.controller->IsAccurateVelocityReady());
    assert(motor.controller->GetMotionDirection() == 0);

    enableCommand = 1;
    memcpy(data, &enableCommand, sizeof(enableCommand));
    OnCanCmd(0x01, data, 4);
    ticks(motor, 3);

    // Positive-current inhibit is applied to the final output, after local
    // assist is added.
    sendExtended(300, 600, 3, 1000 | 0x4000U);
    ticks(motor, 1);
    for (int tick = 0; tick < 2048; ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angle;
        motor.Tick20kHz();
    }
    assert(driver.current == 0);

    // The extended total cap includes both the base motor current and all
    // local correction, so this frame reaches exactly 600 mA.
    sendExtended(500, 600, 4, 1000);
    for (int tick = 0; tick < 2048; ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angle;
        motor.Tick20kHz();
    }
    assert(driver.current == 600);
    assert(motor.controller->GetLocalCompensationCurrent_mA() == 100);
    OnCanCmd(0x27, data, 0);
    memcpy(&actualAssist, reply.data() + 4, 2);
    assert(actualAssist == 100);

    // A control-run fault immediately drops the active base current and local
    // assist, then leaves diagnostics unavailable until an explicit clear.
    motor.controller->ReportControlOverrun();
    ticks(motor, 1);
    assert(motor.controller->GetFaultFlags() == 128);
    assert(motor.controller->state == Motor::STATE_CONTROL_OVERRUN);
    assert(driver.current == 0);
    assert(motor.controller->GetLocalCompensationCurrent_mA() == 0);
    assert(!motor.controller->IsAccurateVelocityReady());
    assert(motor.controller->GetMotionDirection() == 0);
    OnCanCmd(0x08, data, 0);
    ticks(motor, 3);
    assert(motor.controller->GetFaultFlags() == 0);
    assert(motor.controller->modeRunning == Motor::MODE_STOP);
    enableCommand = 1;
    memcpy(data, &enableCommand, sizeof(enableCommand));
    OnCanCmd(0x01, data, 4);
    ticks(motor, 3);

    // Invalid total limits and lease ranges are rejected without advancing the
    // shared monotonic drag sequence.
    const uint16_t dragSequence = motor.controller->GetDragSequence();
    sendExtended(0, 0, 5, 1000);
    assert(motor.controller->GetDragSequence() == dragSequence);
    sendExtended(0, 3001, 5, 1000);
    assert(motor.controller->GetDragSequence() == dragSequence);
    sendExtended(0, 600, 5, 9);
    assert(motor.controller->GetDragSequence() == dragSequence);
    sendExtended(0, 600, 5, 1001);
    assert(motor.controller->GetDragSequence() == dragSequence);

    // A newer lease can inhibit negative final current.
    assert(motor.controller->SetLocalCompensationParam(1, 0));
    int16_t zeroHarmonic = 0;
    for (unsigned i = 0; i < 4; ++i)
        memcpy(data + i * 2, &zeroHarmonic, 2);
    OnCanCmd(0x0D, data, 8);
    sendExtended(-300, 600, 5, 1000 | 0x8000U);
    ticks(motor, 1);
    for (int tick = 0; tick < 2048; ++tick)
    {
        if ((tick & 3) == 0) ++encoder.angle;
        motor.Tick20kHz();
    }
    assert(driver.current == 0);

    // Lease expiry clears final torque and all local velocity/direction state.
    sendExtended(300, 600, 6, 10);
    ticks(motor, 1);
    while (motor.controller->GetRemainingLeaseMs() != 0)
        motor.Tick20kHz();
    assert(driver.current == 0);
    assert(motor.controller->GetLocalCompensationCurrent_mA() == 0);
    assert(!motor.controller->IsAccurateVelocityReady());
    assert(motor.controller->GetMotionDirection() == 0);
    enableCommand = 0;
    memcpy(data, &enableCommand, sizeof(enableCommand));
    OnCanCmd(0x01, data, 4);
    ticks(motor, 1);
    assert(driver.current == 0);

    // With speed-only opt-in, base +200 mA and positive-current inhibit make
    // both the safely limited base and final current zero. The 0x27 reported
    // assist must therefore be zero, not the raw -200 mA difference.
    data[0] = 0xB4; data[1] = 2; data[2] = 1; data[3] = 0;
    optInSeq = 0x1235;
    assistCap = 0;
    memcpy(data + 4, &optInSeq, 2);
    memcpy(data + 6, &assistCap, 2);
    OnCanCmd(0x0E, data, 8);
    ticks(motor, 1);
    assert(motor.controller->IsLocalCompOptInEnabled());
    enableCommand = 1;
    memcpy(data, &enableCommand, sizeof(enableCommand));
    OnCanCmd(0x01, data, 4);
    ticks(motor, 3);
    sendExtended(200, 600, 7, 1000 | 0x4000U);
    ticks(motor, 2);
    assert(driver.current == 0);
    OnCanCmd(0x27, data, 0);
    memcpy(&actualAssist, reply.data() + 4, 2);
    assert(actualAssist == 0);
    assert(motor.controller->GetLocalCompensationCurrent_mA() == 0);

    data[0] = 0xB4; data[1] = 2; data[2] = 0; data[3] = 0;
    optInSeq = 0x1236;
    assistCap = 0;
    memcpy(data + 4, &optInSeq, 2);
    memcpy(data + 6, &assistCap, 2);
    OnCanCmd(0x0E, data, 8);
    ticks(motor, 1);
    assert(!motor.controller->IsLocalCompOptInEnabled());
    OnCanCmd(0x27, data, 0);
    assert(reply[1] == 2 && reply[2] == 0);
    memcpy(&actualAssist, reply.data() + 4, 2);
    assert(actualAssist == 0);
    memcpy(&statusSeq, reply.data() + 6, 2);
    assert(statusSeq == optInSeq);

    std::cout << "F1 CAN address filtering, v2 opt-in, zeroing and limit tests passed\n";
}
