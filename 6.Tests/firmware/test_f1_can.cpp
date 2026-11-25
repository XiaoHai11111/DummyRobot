#include "common_inc.h"
#include "configurations.h"
#include <array>
#include <cstring>
#include <cassert>
#include <limits>
#include <iostream>
#include "Motor/control_timing.h"
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
    bool Init() override { return true; }
    uint16_t UpdateAngle() override { return 0; }
    bool IsCalibrated() override { return calibrated; }
} encoder;
struct Driver : DriverBase {
    void Init() override {}
    void SetFocCurrentVector(uint32_t, int32_t) override {}
    void Sleep() override {}
    void Brake() override {}
    void SetTwoCoilsCurrent(uint16_t, uint16_t) override {}
} driver;
int main() {
    // Boot inquiry before encoder attachment is valid, and reports not calibrated.
    uint8_t data[8]{};
    boardConfig.canNodeId = 2;
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
    std::cout << "F1 CAN status, feed-forward and malformed request tests passed\n";
}
