#include "communication.hpp"
#include "instances/dummy_robot.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

static uint32_t now = 0;
static bool acknowledge = true, transmit = true;
static bool feedbackOnClockRead = false;
struct Frame { CAN_TxHeaderTypeDef header; std::array<uint8_t, 8> data; };
static std::vector<Frame> frames;
CAN_HandleTypeDef handle, hcan1;
static bool preemptSend = false, replyToTiming = true;
static bool replyToTelemetry = true, malformedTelemetry = false;
static bool replyToWristReadback = true, replyToWristStatus = true;
static bool replyToCompensationParams = true;
static bool corruptWristReadback = false, wrongWristVersion = false;
static bool ignoreWristEnableSequence = false, wristSpeedReady = true;
static bool failNextExtendedDrag = false;
static bool failNextWristStatusRequest = false;
static int8_t failWristControlNode = 0;
static int16_t wristReadback[4]{};
static uint8_t wristStatusFlags = 0;
static int8_t wristDirection = 0;
static int16_t wristActualAssistMa = 0;
static uint16_t wristStatusSequence = 0;
DummyRobot dummy(&handle);
CAN_context context{&handle};
CAN_context* get_can_ctx(CAN_HandleTypeDef*) { return &context; }
void fresh();
uint32_t HAL_GetTick() {
    const uint32_t sampled = now;
    if (feedbackOnClockRead) {
        feedbackOnClockRead = false;
        ++now;
        fresh(); // Simulate a CAN ISR after the clock was sampled.
    }
    return sampled;
}
void HAL_NVIC_SystemReset() {}
void fresh() {
    for (unsigned j = 1; j <= 6; ++j) dummy.motorJ[j]->UpdateStatus(0, 0, true);
}
void osDelay(unsigned ticks) {
    now += ticks;
    if (acknowledge) fresh();
    // Do not run the safety watchdog inside synchronous CAN inquiries: the
    // real F4 CAN receive callback updates the motor object directly in ISR.
}
bool CanSendMessage(CAN_context*, uint8_t* data, CAN_TxHeaderTypeDef* header) {
    if (preemptSend) {
        preemptSend = false;
        dummy.motorJ[1]->SetCurrentLimit(0.5f); // Another producer while the CAN mutex is awaited.
    }
    if ((header->StdId & 0x7f) == 0x25 && replyToTiming)
        dummy.motorJ[header->StdId >> 7]->UpdateControlTiming(1200, 1900, 2);
    const unsigned cmd = header->StdId & 0x7f;
    if (failWristControlNode && cmd == 0x0E &&
        (header->StdId >> 7) == failWristControlNode)
    {
        failWristControlNode = 0;
        return false;
    }
    if (failNextWristStatusRequest && cmd == 0x27)
    {
        failNextWristStatusRequest = false;
        return false;
    }
    if (cmd >= 0x29 && cmd <= 0x2C && replyToCompensationParams) {
        uint8_t reply[8]{};
        switch (cmd - 0x29) {
            case 0: {
                int32_t damping = 0, coulomb = 250;
                memcpy(reply, &damping, 4);
                memcpy(reply + 4, &coulomb, 4);
                break;
            }
            case 1: {
                int32_t viscous = 54, deadzone = 20;
                memcpy(reply, &viscous, 4);
                memcpy(reply + 4, &deadzone, 4);
                break;
            }
            case 2: {
                int32_t limit = 500;
                memcpy(reply, &limit, 4);
                break;
            }
            case 3: {
                const int16_t harmonics[4] = {46, -84, -70, -33};
                memcpy(reply, harmonics, sizeof(harmonics));
                break;
            }
            default:
                break;
        }
        if ((header->StdId >> 7) >= 1 && (header->StdId >> 7) <= 6) {
            CAN_RxHeaderTypeDef rx{header->StdId, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
            OnCanMessage(&context, &rx, reply);
        }
    }
    if ((header->StdId >> 7) >= 4 && (header->StdId >> 7) <= 5) {
        if (cmd == 0x0D) {
            for (unsigned i = 0; i < 4; ++i) {
                uint16_t bits = static_cast<uint16_t>(data[i * 2]) |
                                (static_cast<uint16_t>(data[i * 2 + 1]) << 8);
                wristReadback[i] = static_cast<int16_t>(bits);
            }
        } else if (cmd == 0x0E) {
            uint16_t sequence = static_cast<uint16_t>(data[4]) |
                                (static_cast<uint16_t>(data[5]) << 8);
            if (!ignoreWristEnableSequence) wristStatusSequence = sequence;
            wristStatusFlags = data[2] ?
                               static_cast<uint8_t>(1U | (wristSpeedReady ? 2U : 0U)) :
                               0U;
        } else if (cmd == 0x27 && replyToWristStatus) {
            wristStatusFlags = (wristStatusFlags & 1U) |
                               ((wristStatusFlags & 1U) && wristSpeedReady ? 2U : 0U);
            uint8_t reply[8]{0xB4, static_cast<uint8_t>(wrongWristVersion ? 1 : 2),
                             wristStatusFlags, static_cast<uint8_t>(wristDirection)};
            uint16_t actual = static_cast<uint16_t>(wristActualAssistMa);
            reply[4] = static_cast<uint8_t>(actual);
            reply[5] = static_cast<uint8_t>(actual >> 8);
            reply[6] = static_cast<uint8_t>(wristStatusSequence);
            reply[7] = static_cast<uint8_t>(wristStatusSequence >> 8);
            CAN_RxHeaderTypeDef rx{header->StdId, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
            OnCanMessage(&context, &rx, reply);
        } else if (cmd == 0x28 && replyToWristReadback) {
            uint8_t reply[8]{};
            for (unsigned i = 0; i < 4; ++i) {
                int16_t value = wristReadback[i];
                if (corruptWristReadback && i == 0) ++value;
                const uint16_t bits = static_cast<uint16_t>(value);
                reply[i * 2] = static_cast<uint8_t>(bits);
                reply[i * 2 + 1] = static_cast<uint8_t>(bits >> 8);
            }
            CAN_RxHeaderTypeDef rx{header->StdId, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
            OnCanMessage(&context, &rx, reply);
        }
    }
    if (replyToTelemetry && cmd >= 0x21 && cmd <= 0x23) {
        uint8_t reply[8]{}; reply[5] = 2; reply[7] = 0xA1;
        const float v = cmd == 0x21 ? -0.5f : cmd == 0x22 ? 0.25f : 1.0f;
        memcpy(reply, &v, 4);
        CAN_RxHeaderTypeDef rx{header->StdId, 0, CAN_ID_STD, CAN_RTR_DATA,
                              malformedTelemetry && cmd != 0x23 ? 4U : 8U, 0};
        OnCanMessage(&context, &rx, reply);
    }
    Frame f{*header, {}}; memcpy(f.data.data(), data, 8); frames.push_back(f);
    if (failNextExtendedDrag && cmd == 0x0F)
    {
        failNextExtendedDrag = false;
        return false;
    }
    return transmit;
}
float value(const Frame &f, unsigned offset) { float v; memcpy(&v, f.data.data() + offset, 4); return v; }
uint16_t uint16Value(const Frame &f, unsigned offset) {
    return static_cast<uint16_t>(f.data[offset]) |
           (static_cast<uint16_t>(f.data[offset + 1]) << 8);
}
void ascii(const char* text, StreamSink &sink, bool uart = false) {
    sink.output.clear();
    if (uart) OnUart4AsciiCmd(text, strlen(text), sink);
    else OnUsbAsciiCmd(text, strlen(text), sink);
}

struct GravityRegressionCase
{
    float q[6];
    float legacyCurrentA[6];
    float wristTorqueNm[2];
};

static const GravityRegressionCase kGravityRegressionCases[] = {
    {
        {0.0f, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, -0.091155037f, -0.143425360f, 0.007438710f, 0.069814500f, 0.0f},
        {-0.0013025755f, -0.1341844936f}
    },
    {
        {-108.9540177f, -52.0080313f, 103.0739967f,
         132.3815767f, -28.1603279f, -47.6849955f},
        {0.0f, 0.309114516f, -0.038319468f, 0.023243600f, -0.058295400f, 0.0f},
        {-0.0352619699f, 0.1016733858f}
    },
    {
        {-125.2984692f, -60.8280549f, 101.9351846f,
         -39.7402901f, -70.3436551f, 328.6639728f},
        {0.0f, 0.380979925f, 0.019537641f, -0.019041100f, -0.042205400f, 0.0f},
        {0.0522394081f, 0.0719232353f}
    }
};

static void VerifyGravityRegression()
{
    float maxLegacyError = 0;
    float maxWristTorqueError = 0;
    bool positiveWristTorque = false;
    bool negativeWristTorque = false;
    for (const auto& sample : kGravityRegressionCases)
    {
        float actualA[7]{};
        float actualNm[7]{};
        dummy.gravityModel.CalculateGravityCurrentA(sample.q, actualA);
        dummy.gravityModel.CalculateGravityTorqueNm(sample.q, actualNm);
        for (unsigned joint = 1; joint <= 6; ++joint)
        {
            maxLegacyError = std::max(
                maxLegacyError, fabsf(actualA[joint] - sample.legacyCurrentA[joint - 1]));
        }
        for (unsigned wrist = 0; wrist < 2; ++wrist)
        {
            const float torque = actualNm[wrist + 4];
            maxWristTorqueError = std::max(
                maxWristTorqueError, fabsf(torque - sample.wristTorqueNm[wrist]));
            positiveWristTorque |= torque > 1e-6f;
            negativeWristTorque |= torque < -1e-6f;
        }
    }
    assert(positiveWristTorque && negativeWristTorque);
    std::cout << "Representative per-joint gravity current max error: "
              << maxLegacyError << " A; MJCF wrist torque max error: "
              << maxWristTorqueError << " Nm\n";
    assert(maxLegacyError < 1e-6f);
    assert(maxWristTorqueError < 1e-5f);
}

int main() {
    dummy.Init(); fresh(); dummy.UpdateJointAnglesCallback(); dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    VerifyGravityRegression();
    feedbackOnClockRead = true;
    dummy.CheckSafety();
    assert(!dummy.HasFault() && dummy.IsEnabled());
    feedbackOnClockRead = true;
    assert(dummy.motorJ[1]->FeedbackAge() == 0);
    dummy.motorJ[1]->lastFeedbackTick = UINT32_MAX - 9;
    now = 10;
    assert(dummy.motorJ[1]->FeedbackAge() == 20); // 32-bit tick wrap.
    now = 0;
    fresh();
    StreamSink usb, uart; uart.channelType = StreamSink::CHANNEL_TYPE_UART4;

    ascii(">0,-75,180,0,0,0,30,40,0.6", usb);
    assert(usb.output == "15\r\n");
    dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(0));
    assert(usb.output.find("ok\r\n") != std::string::npos);
    frames.clear(); dummy.MoveJoints(dummy.targetJoints);
    assert(frames.size() == 6);
    for (auto &f : frames) assert(std::isfinite(value(f, 4)) && value(f, 4) > 0);
    assert(dummy.MoveJ(0,-45,180,0,0,0));
    frames.clear(); dummy.MoveJoints(dummy.targetJoints);
    assert(fabsf(value(frames[1], 4) - 2.5f) < 0.001f); // J2: 30 deg/s * 30 / 360.

    frames.clear(); preemptSend = true;
    dummy.motorJ[1]->SetPositionWithVelocityLimit(1.25f, 2.5f);
    assert(frames.size() == 2);
    assert(frames[0].header.StdId == ((1U << 7) | 0x12));
    assert(frames[1].header.StdId == ((1U << 7) | 0x07));
    assert(value(frames[1], 0) == 1.25f && value(frames[1], 4) == 2.5f);
    ascii("#GETCAN", usb); assert(usb.output == "okc 0 0 0 0 0 0\r\n");
    ascii("#GETDRV 0", usb); assert(usb.output.find("error") == 0);
    ascii("#GETDRV 7", usb); assert(usb.output.find("error") == 0);
    ascii("#GETDRV 1 extra", usb); assert(usb.output.find("error") == 0);
    ascii("#GETDRV 1", usb); assert(usb.output.find("okd 1 0 0 0 0 0 ") == 0);

    const uint32_t feedbackBeforeTiming = dummy.motorJ[1]->lastFeedbackTick;
    ascii("#GETTIMING 1", usb); assert(usb.output == "okt 1 1200 1900 2\r\n");
    assert(dummy.motorJ[1]->lastFeedbackTick == feedbackBeforeTiming);
    replyToTiming = false;
    const auto timingStart = now;
    ascii("#GETTIMING 1", usb); assert(usb.output.find("error F1 timing response timeout") == 0);
    assert(now - timingStart == 50);
    replyToTiming = true;
    frames.clear(); dummy.SetEnable(false);
    for (const auto& frame : frames)
    {
        const unsigned node = frame.header.StdId >> 7;
        const unsigned command = frame.header.StdId & 0x7f;
        if (node == 4 || node == 5)
            assert(command != 0x0E && command != 0x27 && command != 0x28);
    }
    ascii("#GETTELEM 0", usb); assert(usb.output.find("error") == 0);
    ascii("#GETTELEM 7", usb); assert(usb.output.find("error") == 0);
    ascii("#GETTELEM 2 extra", usb); assert(usb.output.find("error") == 0);
    now = 10;
    ascii("#GETTELEM 2", usb);
    assert(usb.output == "okm 2 10 10 10 -87.00000 -3.00000 -0.5000 2 0 1\r\n");
    replyToTelemetry = false;
    const auto telemetryStart = now;
    ascii("#GETTELEM 2", usb); assert(usb.output.find("error F1 telemetry timeout") == 0);
    assert(now - telemetryStart == 50);
    replyToTelemetry = true; malformedTelemetry = true;
    ascii("#GETTELEM 2", usb); assert(usb.output.find("error F1 telemetry timeout") == 0);
    malformedTelemetry = false;
    // Diagnostics must never mask lost position/status feedback.
    now = 150;
    const auto feedbackTick = dummy.motorJ[2]->lastFeedbackTick;
    const auto feedbackCounter = dummy.motorJ[2]->feedbackCount;
    uint8_t diag[8]{}; diag[7] = 0xA1; diag[5] = 2;
    CAN_RxHeaderTypeDef diagHeader{2U << 7 | 0x21, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
    const float diagnosticCurrent = 0.25f; memcpy(diag, &diagnosticCurrent, 4);
    OnCanMessage(&context, &diagHeader, diag);
    diagHeader.StdId = 2U << 7 | 0x22;
    OnCanMessage(&context, &diagHeader, diag);
    assert(dummy.motorJ[2]->lastFeedbackTick == feedbackTick);
    assert(dummy.motorJ[2]->feedbackCount == feedbackCounter);
    now = 0; fresh();
    for (unsigned j = 1; j <= 6; ++j) dummy.motorJ[j]->UpdateAngleCallback(0, true);
    dummy.UpdateJointAnglesCallback();
    dummy.SetEnable(true);
    // INT replaces pending work. SEQ preserves all points, then rejects overflow.
    assert(dummy.commandHandler.Push(">1,-75,180,0,0,0") == 15);
    assert(dummy.commandHandler.Push(">2,-75,180,0,0,0") == 15);
    assert(std::string(dummy.commandHandler.Pop(0).text).find(">2,") == 0);
    dummy.SetCommandMode(1);
    for (int n = 0; n < 16; ++n) assert(dummy.commandHandler.Push(">1,-75,180,0,0,0") != 255);
    assert(dummy.commandHandler.Push(">2,-75,180,0,0,0") == 255);
    dummy.SetCommandMode(2);
    assert(dummy.commandHandler.Push(std::string(128, '>')) == 255);
    // Partial USB input cannot contaminate UART input (and vice versa).
    usb.output.clear(); uart.output.clear();
    const char* prefix = ">0,-";
    ASCII_protocol_parse_stream(reinterpret_cast<const uint8_t*>(prefix), strlen(prefix), usb);
    const char* status = "#GETSTATUS\r\n";
    ASCII_protocol_parse_stream(reinterpret_cast<const uint8_t*>(status), strlen(status), uart);
    assert(uart.output.find("oks ") == 0 && usb.output.empty());
    const char* suffix = "75,180,0,0,0,30\n";
    ASCII_protocol_parse_stream(reinterpret_cast<const uint8_t*>(suffix), strlen(suffix), usb);
    assert(usb.output == "15\r\n");
    usb.output.clear(); Respond(usb, "%s", std::string(500, 'x').c_str());
    assert(usb.output.size() == 159 && usb.output.substr(157) == "\r\n");
    ascii("&0,-75,180,0,0,0", usb); assert(usb.output.find("error") == 0);
    ascii(">nan,-75,180,0,0,0", uart, true); assert(uart.output.find("error") == 0);
    ascii("#CMDMODE 4", usb); assert(dummy.commandMode == DummyRobot::COMMAND_MOTOR_TUNING);
    ascii(">0,-75,180,0,0,0", usb); assert(usb.output.find("error") == 0);

    // Drag mode emits leased current commands, never a stale position target.
    ascii("#CMDMODE 5", usb); assert(dummy.commandMode == DummyRobot::COMMAND_DRAG);
    ascii("#GETDRAG", usb); assert(usb.output.find("okg 5 0 0.000 0.000 1.000") == 0);
    ascii("#DRAG_GAIN 0.01 0.25 1.0", usb);
    assert(usb.output.find("Drag gains") == 0);
    ascii("#DRAG_GAIN nan 0.25 1.0", usb); assert(usb.output.find("error") == 0);

    // Stage 2 local compensation accepts F1 BoardConfig units and sends five RAM frames.
    frames.clear(); ascii("#LOCALCOMP 2 0 125 69 0 500", usb);
    assert(usb.output.find("Local compensation J2") == 0 && frames.size() == 5);
    const int32_t expectedLocal[5] = {0, 125, 69, 25, 500};
    for (unsigned i = 0; i < frames.size(); ++i)
    {
        int32_t sent = 0;
        memcpy(&sent, frames[i].data.data(), 4);
        assert(frames[i].header.StdId == ((2U << 7) | 0x0C));
        assert(sent == expectedLocal[i]);
        assert(frames[i].data[4] == i);
        assert(frames[i].data[5] == 0);
    }
    frames.clear(); ascii("#LOCALCOMP 2 0 125 69 0 500 1", usb);
    assert(usb.output.find("Local compensation J2") == 0 &&
           usb.output.find("stored") != std::string::npos && frames.size() == 5);
    for (const auto& frame : frames) assert(frame.data[5] == 1);
    frames.clear(); ascii("#LOCALCOMP 7 0 0 0 0 0", usb);
    assert(usb.output.find("error") == 0 && frames.empty());
    frames.clear(); ascii("#LOCALCOMP 2 0 4000 0 0 0", usb);
    assert(usb.output.find("error") == 0 && frames.empty());
    frames.clear(); ascii("#LOCALCOMP 1 0 77 24 0 500", usb);
    assert(frames.size() == 5);
    int32_t j1Deadzone = 0;
    memcpy(&j1Deadzone, frames[3].data.data(), 4);
    assert(j1Deadzone == DummyRobot::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000[1]);

    // One-shot deployment writes DCE, five local compensation columns and four
    // harmonic columns to every F1; wrist RAM mirrors the stored harmonics.
    dummy.SetEnable(false);
    frames.clear(); ascii("#SETALLPARAMS", usb);
    assert(usb.output == "okall joint params stored\r\n");
    assert(frames.size() == 6 * (4 + 5) + 2);
    unsigned allDce = 0, allLocal = 0, allStoredHarmonic = 0;
    for (const auto& frame : frames)
    {
        const unsigned node = frame.header.StdId >> 7;
        const unsigned command = frame.header.StdId & 0x7f;
        assert(node >= 1 && node <= 6);
        if (command >= 0x17 && command <= 0x1A)
        {
            ++allDce;
            int32_t value = 0;
            memcpy(&value, frame.data.data(), 4);
            const int32_t expected[4] = {
                DummyRobot::DCE_KP[node], DummyRobot::DCE_KV[node],
                DummyRobot::DCE_KI[node], DummyRobot::DCE_KD[node]};
            assert(value == expected[command - 0x17]);
            assert(frame.data[4] == 1);
        } else if (command == 0x0C)
        {
            ++allLocal;
            int32_t value = 0;
            memcpy(&value, frame.data.data(), 4);
            assert(value == DummyRobot::LOCAL_COMP[node][frame.data[4]]);
            assert(frame.data[4] <= 4 && frame.data[5] == 1);
        } else if (command == 0x10)
        {
            ++allStoredHarmonic;
            int16_t value = 0;
            for (unsigned i = 0; i < 4; ++i)
            {
                memcpy(&value, frame.data.data() + i * 2, 2);
                assert(value == DummyRobot::ALL_HARMONICS_MA[node][i]);
            }
        } else
        {
            assert(false);
        }
    }
    assert(allDce == 24 && allLocal == 30 && allStoredHarmonic == 2);
    assert(dummy.GetWristConfig(4).harmonicBaseMa[0] == 46);
    assert(dummy.GetWristConfig(5).harmonicBaseMa[0] == -25);
    assert(dummy.GetWristConfig(5).frictionLimitMa == 500);

    frames.clear(); ascii("#GETCOMP 4", usb);
    assert(usb.output.find("okcomp 4 local 0 250 54 20 500 harm 46 -84 -70 -33") == 0);
    assert(frames.size() == 4);
    for (unsigned i = 0; i < frames.size(); ++i)
        assert((frames[i].header.StdId & 0x7f) == 0x29 + i);
    replyToCompensationParams = false;
    ascii("#GETCOMP 4", usb);
    assert(usb.output.find("error compensation inquiry timeout") == 0);
    replyToCompensationParams = true;

    dummy.SetEnable(false);
    frames.clear(); ascii("#SETCOMP 4 0 250 54 25 500 1 46 -84 -70 -33", usb);
    assert(usb.output == "okcomp 4 configured stored\r\n");
    assert(frames.size() == 6);
    unsigned storedHarmonicFrames = 0;
    for (const auto& frame : frames)
    {
        const unsigned command = frame.header.StdId & 0x7f;
        if (command == 0x0C) assert(frame.data[5] == 1);
        if (command == 0x10) ++storedHarmonicFrames;
    }
    assert(storedHarmonicFrames == 1);
    assert(dummy.GetWristConfig(4).dragLocalLimitMa == 500);

    dummy.SetCommandMode(5);
    assert(dummy.GetWristConfig(4).dragAutoEnabled);
    assert(dummy.GetWristConfig(4).frictionRequested);
    assert(dummy.GetWristConfig(4).speedRequested);
    assert(dummy.GetWristConfig(4).harmonicVerified);
    dummy.StopMotion();
    dummy.SetEnable(false);
    dummy.SetCommandMode(5);

    now += 1000; fresh(); dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    dummy.StopMotion();
    dummy.SetEnable(false);
    dummy.SetCommandMode(2);
    now += 1000; fresh(); dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    frames.clear(); dummy.DragTick(1);
    assert(frames.size() == 6);
    for (const auto& frame : frames)
    {
        assert(frame.header.StdId == ((frame.header.StdId >> 7) << 7 | 0x0B) &&
               std::isfinite(value(frame, 0)) && std::isfinite(value(frame, 4)));
        assert((frame.header.StdId & 0x7f) != 0x27 &&
               (frame.header.StdId & 0x7f) != 0x28);
    }
    dummy.SetCommandMode(5);
    const int32_t defaultWristLimit = dummy.GetWristConfig(4).frictionLimitMa;
    frames.clear(); ascii("#WRIST_FRICTION 4 0.50 200", usb);
    assert(usb.output.find("error") == 0 && frames.empty());
    assert(dummy.GetWristConfig(4).frictionLimitMa == defaultWristLimit);
    // REST pose preserves the HEAD J5 drag baseline. New MJCF wrist gravity is
    // an independent blend and does not silently replace the legacy curve.
    assert(value(frames[1], 0) < -0.1f && value(frames[2], 0) > 0.1f);
    // J5 now uses the fitted six-axis model; the REST-pose value may differ
    // from the legacy HEAD curve.
    frames.clear(); dummy.DragTick(20);
    unsigned dragFrames = 0, feedbackRequests = 0;
    for (const auto& frame : frames)
    {
        const unsigned cmd = frame.header.StdId & 0x7f;
        if (cmd == 0x0B) ++dragFrames;
        if (cmd == 0x22 || cmd == 0x23) ++feedbackRequests;
        assert(cmd != 0x07);
    }
    assert(dragFrames == 6 && feedbackRequests == 12);
    ascii("#GETDRAG", usb); assert(usb.output.find("okg 5 0 ") == 0);

    // Position-mode legacy feed-forward follows the HEAD model and total switch.
    for (unsigned j = 1; j <= 6; ++j) dummy.currentJoints.a[j - 1] = 0;
    dummy.currentJoints.a[1] = 0;
    dummy.currentJoints.a[2] = 90;
    ascii("#GRAVITY_FF_ON", usb);
    frames.clear(); dummy.UpdateGravityFeedForward(4);
    assert(frames.size() == 6);
    for (const auto& frame : frames) assert((frame.header.StdId & 0x7f) == 0x0A);
    assert(value(frames[1], 0) > 0 && value(frames[2], 0) > 0);
    const float defaultJ4Current = value(frames[3], 0);
    const float defaultJ5Current = value(frames[4], 0);
    ascii("#GRAVITY_FF_OFF", usb);
    frames.clear(); dummy.UpdateGravityFeedForward(8);
    assert(frames.size() == 6);
    for (const auto& frame : frames) assert(value(frame, 0) == 0);

    // Wrist MJCF gravity is cross-faded from the HEAD baseline to the new
    // per-axis candidate. The original total switch still owns position mode.
    dummy.StopMotion();
    frames.clear();
    ascii("#WRIST_GRAVITY 4 0.25", usb);
    assert(usb.output == "okw 4 gravity 0.250\r\n");
    ascii("#WRIST_GRAVITY 5 0.10", usb);
    assert(usb.output == "okw 5 gravity 0.100\r\n");
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    frames.clear(); dummy.UpdateGravityFeedForward(12);
    assert(frames.size() == 6);
    for (const auto& frame : frames) assert(value(frame, 0) == 0);
    ascii("#GRAVITY_FF_ON", usb);
    frames.clear(); dummy.UpdateGravityFeedForward(16);
    const float newJ4 = value(frames[3], 0);
    const float newJ5 = value(frames[4], 0);
    assert(newJ4 != 0 && newJ5 != 0);
    assert(value(frames[0], 0) == 0 && value(frames[5], 0) == 0);
    ascii("#GRAVITY_FF_OFF", usb);
    dummy.StopMotion();
    frames.clear();
    ascii("#GRAVITY 4 1", usb);
    ascii("#GRAVITY 5 1", usb);
    assert(dummy.gravityModel.GetJointGravityModel(4).scale == 1.0f);
    assert(dummy.gravityModel.GetJointGravityModel(5).scale == 1.0f);
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    ascii("#GRAVITY_FF_ON", usb);
    frames.clear(); dummy.UpdateGravityFeedForward(20);
    assert(value(frames[3], 0) == defaultJ4Current);
    assert(value(frames[4], 0) == defaultJ5Current);
    ascii("#GRAVITY_FF_OFF", usb);
    dummy.SetCommandMode(2);

    // Wrong node, short, remote, extended and NaN feedback must not index/refresh a joint.
    uint8_t data[8]{}; data[7] = 0xA1;
    CAN_RxHeaderTypeDef header{15U << 7 | 0x23, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
    OnCanMessage(&context, &header, data);
    header.StdId = 1U << 7 | 0x23; header.DLC = 4;
    for (unsigned joint = 1; joint <= 6; ++joint) dummy.motorJ[joint]->lastFeedbackTick = 0;
    now = 100; OnCanMessage(&context, &header, data); assert(dummy.motorJ[1]->lastFeedbackTick == 0);
    header.DLC = 8; float nan = std::numeric_limits<float>::quiet_NaN(); memcpy(data, &nan, 4);
    OnCanMessage(&context, &header, data); assert(dummy.motorJ[1]->lastFeedbackTick == 0);
    memset(data, 0, 4); header.RTR = 1;
    OnCanMessage(&context, &header, data); assert(dummy.motorJ[1]->lastFeedbackTick == 0);
    header.RTR = 0; header.StdId = 2U << 7 | 0x23; data[5] = 3; data[6] = 1;
    OnCanMessage(&context, &header, data); dummy.CheckSafety();
    assert(dummy.GetJointFault(2) == 1 && !dummy.IsEnabled());
    ascii("#GETSTATUS", usb); assert(usb.output == "oks 0 2 2 0 1 0 0 0 0\r\n");
    ascii(">0,-75,180,0,0,0", uart, true); assert(uart.output.find("error") == 0);
    fresh(); dummy.SetEnable(true); assert(!dummy.IsEnabled()); // Fault stays latched.
    assert(dummy.ClearFaults()); assert(!dummy.IsEnabled());
    dummy.SetEnable(true); assert(dummy.IsEnabled());
    assert(!dummy.ClearFaults()); // Refuse clear while enabled.

    dummy.commandHandler.Push(">20,-75,180,0,0,0");
    const auto pending = dummy.commandHandler.Pop(0);
    dummy.StopMotion(); dummy.SetEnable(true);
    dummy.commandHandler.ParseCommand(pending);
    assert(dummy.targetJoints.a[0] == dummy.currentJoints.a[0]); // No stale replay.
    for (unsigned j = 1; j <= 6; ++j) dummy.motorJ[j]->UpdateStatus(2, 0, true);
    now += 251; acknowledge = false; dummy.CheckSafety();
    assert(!dummy.IsEnabled() && (dummy.GetJointFault(1) & 8));
    assert(dummy.GetFaultFeedbackAge(1) == 251);
    ascii("#GETDRV 1", usb); assert(usb.output.find("okd 1 251 2 0 8 251 ") == 0);
    const auto start = now;
    assert(!dummy.ClearFaults()); assert(now - start <= 500);
    acknowledge = true; assert(dummy.ClearFaults());
    assert(dummy.GetFaultFeedbackAge(1) == 0);
    dummy.motorJ[3]->UpdateStatus(0, 0, false); dummy.CheckSafety();
    assert(dummy.GetJointFault(3) & 16); fresh(); assert(dummy.ClearFaults());
    header.StdId = 3U << 7 | 0x23; header.DLC = 8; data[5] = 6; data[6] = 128;
    OnCanMessage(&context, &header, data); dummy.CheckSafety();
    assert(dummy.GetJointFault(3) == 128); // Accepted as a timing fault, not old firmware.
    fresh(); assert(dummy.ClearFaults());
    header.StdId = 3U << 7 | 0x25;
    const uint16_t last = 1200, peak = 2200; const uint32_t overruns = 3;
    memcpy(data, &last, 2); memcpy(data+2, &peak, 2); memcpy(data+4, &overruns, 4);
    header.DLC = 7;
    const auto replyCount = dummy.motorJ[3]->timingReplyCount;
    OnCanMessage(&context, &header, data); assert(dummy.motorJ[3]->timingReplyCount == replyCount);
    header.DLC = 8; OnCanMessage(&context, &header, data);
    assert(dummy.motorJ[3]->controlLastCycles == 1200 && dummy.motorJ[3]->controlMaxCycles == 2200);
    assert(dummy.motorJ[3]->controlOverruns == 3);
    dummy.SetEnable(true); now += 101; fresh(); dummy.CheckSafety();
    assert(!dummy.IsEnabled() && (dummy.GetJointFault(3) & 64));
    assert(dummy.ClearFaults());
    dummy.SetEnable(true); dummy.MoveJ(50,-75,180,0,0,0);
    for (unsigned j = 1; j <= 6; ++j) dummy.motorJ[j]->state = CtrlStepMotor::FINISH;
    assert(!dummy.WaitForMotion(20));
    assert(!dummy.IsEnabled() && (dummy.GetJointFault(1) & 32));
    transmit = false; assert(!dummy.ClearFaults()); transmit = true; assert(dummy.ClearFaults());
    assert(dummy.commandHandler.Pop(0).text[0] == 0);

    // New wrist opt-in is node-addressed and closed by readback/version/seq.
    // RAM configuration is performed while F4 is stopped.
    assert(!dummy.IsEnabled());
    frames.clear(); replyToWristReadback = false;
    const auto oldFirmwareStart = now;
    ascii("#WRIST_SPEED 4 1", usb);
    assert(usb.output.find("error wrist protocol/readback timeout") == 0);
    assert(now - oldFirmwareStart == 50);
    assert(dummy.GetWristConfig(4).oldProtocol);
    assert(!dummy.GetWristConfig(4).speedEnabled);
    assert(frames.size() == 2);
    assert((frames[0].header.StdId & 0x7f) == 0x28);
    assert((frames[1].header.StdId & 0x7f) == 0x09);
    assert((frames[0].header.StdId >> 7) == 4 && (frames[1].header.StdId >> 7) == 4);
    replyToWristReadback = true;

    // v2 writes unscaled motor-domain harmonics, a gain/cap in 0x0E, and
    // never mutates the legacy 0x0C/local-limit path.
    wristSpeedReady = false;
    frames.clear(); ascii("#WRIST_FRICTION 4 0.25 200", usb);
    assert(usb.output.find("okw 4 friction 0.250 limit 200") == 0);
    const uint8_t expectedCommands[] = {0x28, 0x27, 0x0D, 0x28, 0x0E, 0x27};
    assert(frames.size() == sizeof(expectedCommands));
    for (unsigned i = 0; i < sizeof(expectedCommands); ++i)
    {
        assert((frames[i].header.StdId >> 7) == 4);
        assert((frames[i].header.StdId & 0x7f) == expectedCommands[i]);
    }
    const int16_t expectedJ4Harmonic[4] = {46, -84, -70, -33};
    for (unsigned i = 0; i < 4; ++i)
        assert(static_cast<int16_t>(uint16Value(frames[2], i * 2)) ==
               expectedJ4Harmonic[i]);
    const uint16_t enabledSequence = uint16Value(frames[4], 4);
    const uint8_t expectedEnable[] = {
        0xB4, 2, 1, 25, static_cast<uint8_t>(enabledSequence),
        static_cast<uint8_t>(enabledSequence >> 8),
        static_cast<uint8_t>(200), 0
    };
    assert(memcmp(frames[4].data.data(), expectedEnable, 8) == 0);
    for (const auto& frame : frames) assert((frame.header.StdId & 0x7f) != 0x0C);
    assert(dummy.GetWristConfig(4).protocolVerified);
    assert(dummy.GetWristConfig(4).harmonicVerified);
    assert(dummy.GetWristConfig(4).speedEnabled);
    assert(!dummy.GetWristConfig(4).frictionEnabled);
    assert(dummy.GetWristConfig(4).statusSequence == enabledSequence);
    wristSpeedReady = true;
    dummy.UpdateGravityFeedForward(20);
    assert(dummy.GetWristConfig(4).frictionEnabled);
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());

    // A speed-only acknowledgement is not friction assistance. End-stop
    // bits are emitted even when the base current is exactly zero.
    dummy.SetCommandMode(5);
    dummy.currentJoints = dummy.REST_POSE;
    dummy.currentJoints.a[3] = dummy.motorJ[4]->angleLimitMax - 0.5f;
    frames.clear(); dummy.DragTick(21);
    bool sawHighStop = false;
    bool sawLegacyJ5 = false;
    for (const auto& frame : frames)
    {
        const unsigned node = frame.header.StdId >> 7;
        const unsigned command = frame.header.StdId & 0x7f;
        if (command == 0x0F) assert(node == 4 || node == 5);
        if (node == 5 && command == 0x0B) sawLegacyJ5 = true;
        if (node == 4 && command == 0x0F)
        {
            sawHighStop = true;
            const uint16_t leaseFlags = uint16Value(frame, 6);
            assert((leaseFlags & 0x4000U) != 0 && (leaseFlags & 0x8000U) == 0);
            assert((leaseFlags & 0x3FFFU) == DummyRobot::Config_t().dragLeaseMs);
            assert(uint16Value(frame, 2) == 1000);
        }
    }
    assert(sawHighStop);
    assert(sawLegacyJ5);

    dummy.currentJoints.a[3] = dummy.motorJ[4]->angleLimitMin + 0.5f;
    frames.clear(); dummy.DragTick(22);
    bool sawLowStop = false;
    for (const auto& frame : frames)
        if ((frame.header.StdId >> 7) == 4 && (frame.header.StdId & 0x7f) == 0x0F)
        {
            sawLowStop = true;
            const uint16_t leaseFlags = uint16Value(frame, 6);
            assert((leaseFlags & 0x8000U) != 0 && (leaseFlags & 0x4000U) == 0);
        }
    assert(sawLowStop);

    // A missing periodic status closes the node-local opt-in and sends a
    // speed-only lease before F4 stops issuing extended drag commands.
    replyToWristStatus = false;
    now += 251;
    frames.clear(); dummy.UpdateGravityFeedForward(40);
    assert(!dummy.GetWristConfig(4).speedEnabled);
    assert(!dummy.GetWristConfig(4).frictionEnabled);
    bool leaseDisabledJ4 = false;
    for (const auto& frame : frames)
        if ((frame.header.StdId >> 7) == 4 &&
            (frame.header.StdId & 0x7f) == 0x0E && frame.data[2] == 0)
            leaseDisabledJ4 = true;
    assert(leaseDisabledJ4);
    replyToWristStatus = true;
    dummy.currentJoints = dummy.REST_POSE;
    dummy.SetEnable(false);
    fresh();
    assert(dummy.ClearFaults());

    // Parameter readback mismatch and protocol-version mismatch keep opt-in off.
    corruptWristReadback = true; frames.clear();
    ascii("#WRIST_FRICTION 5 0.50 300", usb);
    assert(usb.output.find("error wrist protocol/readback timeout") == 0);
    assert(!dummy.GetWristConfig(5).speedEnabled && !dummy.GetWristConfig(5).frictionEnabled);
    for (const auto& frame : frames)
        assert((frame.header.StdId & 0x7f) != 0x0E || frame.data[2] == 0);
    corruptWristReadback = false;

    wrongWristVersion = true; frames.clear();
    ascii("#WRIST_SPEED 5 1", usb);
    assert(usb.output.find("error wrist protocol/readback timeout") == 0);
    assert(!dummy.GetWristConfig(5).speedEnabled);
    for (const auto& frame : frames)
        assert((frame.header.StdId & 0x7f) != 0x0E || frame.data[2] == 0);
    wrongWristVersion = false;

    // Speed measurement is an opt-in with gain=cap=0 and no 0x0D write.
    frames.clear(); ascii("#WRIST_FRICTION 5 0 100", usb);
    assert(usb.output.find("okw 5 friction 0.000 limit 100") == 0);
    assert(!dummy.GetWristConfig(5).frictionRequested);
    ascii("#WRIST_SPEED 5 0", usb);
    assert(usb.output == "okw 5 speed 0\r\n");
    frames.clear(); ascii("#WRIST_SPEED 5 1", usb);
    assert(usb.output == "okw 5 speed 1\r\n");
    assert(dummy.GetWristConfig(5).speedEnabled);
    assert(!dummy.GetWristConfig(5).frictionEnabled);
    const uint8_t speedOnlyCommands[] = {0x28, 0x27, 0x0E, 0x27};
    assert(frames.size() == sizeof(speedOnlyCommands));
    for (unsigned i = 0; i < sizeof(speedOnlyCommands); ++i)
        assert((frames[i].header.StdId & 0x7f) == speedOnlyCommands[i]);
    assert(frames[2].data[2] == 1 && frames[2].data[3] == 0);
    assert(uint16Value(frames[2], 6) == 0);
    for (const auto& frame : frames)
    {
        const unsigned command = frame.header.StdId & 0x7f;
        assert(command != 0x0C && command != 0x0D);
    }

    // A failed status request must immediately invalidate both the control and
    // reported-speed bits, and hold the wrist until a fresh handshake succeeds.
    failNextWristStatusRequest = true;
    frames.clear(); dummy.UpdateGravityFeedForward(60);
    assert(!dummy.GetWristConfig(5).protocolVerified);
    assert(!dummy.GetWristConfig(5).controlVerified);
    assert(!dummy.GetWristConfig(5).speedEnabled);
    bool heldJ5 = false;
    for (const auto& frame : frames)
        if ((frame.header.StdId >> 7) == 5 && (frame.header.StdId & 0x7f) == 0x09)
            heldJ5 = true;
    assert(heldJ5);
    frames.clear(); ascii("#WRIST_SPEED 5 1", usb);
    assert(usb.output == "okw 5 speed 1\r\n");

    // J5 has inverse motor polarity. A joint-positive end stop must disable
    // motor-positive current, and a failed 0x0F write stops the whole stream.
    assert(!dummy.IsEnabled());
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    dummy.currentJoints = dummy.REST_POSE;
    dummy.currentJoints.a[4] = dummy.motorJ[5]->angleLimitMax - 0.5f;
    frames.clear(); dummy.DragTick(23);
    bool sawInverseStop = false;
    for (const auto& frame : frames)
    {
        const unsigned node = frame.header.StdId >> 7;
        const unsigned command = frame.header.StdId & 0x7f;
        if (command == 0x0F) assert(node == 4 || node == 5);
        if (node == 5 && command == 0x0F)
        {
            sawInverseStop = true;
            const uint16_t leaseFlags = uint16Value(frame, 6);
            assert((leaseFlags & 0x8000U) != 0 && (leaseFlags & 0x4000U) == 0);
        }
    }
    assert(sawInverseStop);

    frames.clear(); failNextExtendedDrag = true; dummy.DragTick(24);
    assert(!dummy.IsEnabled() && dummy.IsHolding());
    size_t failedExtendedFrame = frames.size();
    for (size_t i = 0; i < frames.size(); ++i)
        if ((frames[i].header.StdId >> 7) == 5 &&
            (frames[i].header.StdId & 0x7f) == 0x0F)
        {
            failedExtendedFrame = i;
            break;
        }
    assert(failedExtendedFrame < frames.size());
    for (size_t i = failedExtendedFrame + 1; i < frames.size(); ++i)
        assert((frames[i].header.StdId & 0x7f) != 0x0F);
    dummy.SetEnable(false);

    assert(!dummy.IsEnabled());
    frames.clear(); ascii("#WRIST_FRICTION 5 0.01 100", usb);
    assert(usb.output.find("okw 5 friction 0.010 limit 100") == 0);
    dummy.SetEnable(true);
    wristDirection = -1;
    wristActualAssistMa = 47;
    ascii("#GETWRIST 5", usb);
    assert(usb.output.find(
        "okw 5 g0.100 gm1.004686,-0.003045,0.180 f0.010 fs0.010 "
        "s1 flags3 dir-1 current47") == 0);
    assert(wristDirection == -1 && wristActualAssistMa == 47);

    // Emergency STOP does not query, clears assistance to gain=cap=0, and
    // keeps only the explicitly configured accurate-velocity window.
    frames.clear(); dummy.StopMotion();
    assert(!dummy.IsEnabled() && dummy.IsHolding());
    bool stoppedAssistOff = false;
    for (const auto& frame : frames)
    {
        const unsigned command = frame.header.StdId & 0x7f;
        assert(command != 0x0F);
        if ((frame.header.StdId >> 7) == 5 && command == 0x0E)
        {
            stoppedAssistOff = frame.data[0] == 0xB4 && frame.data[1] == 2 &&
                               frame.data[2] == 1 && frame.data[3] == 0 &&
                               uint16Value(frame, 6) == 0;
        }
    }
    assert(stoppedAssistOff);
    wristDirection = 0;
    wristActualAssistMa = 0;
    ascii("#GETWRIST 5", usb);
    assert(usb.output.find("f0.000 fs0.010 s1") != std::string::npos);
    assert(usb.output.find("current0") != std::string::npos);

    // START after HOLD must resynchronize the speed-only sequence before the
    // extended node emits 0x0F. It must not silently fall back to old 0x0B.
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    frames.clear(); dummy.DragTick(101);
    assert(dummy.IsEnabled());
    bool restartExtendedJ5 = false;
    for (const auto& frame : frames)
    {
        const unsigned node = frame.header.StdId >> 7;
        const unsigned command = frame.header.StdId & 0x7f;
        assert(node != 5 || command != 0x0B);
        if (node == 5 && command == 0x0F) restartExtendedJ5 = true;
    }
    assert(restartExtendedJ5);

    // If the STOP zero-assist write is lost, a mismatched sequence blocks every
    // J5 old-current lease until status is explicitly re-established.
    failWristControlNode = 5;
    dummy.StopMotion();
    assert(!dummy.IsEnabled() && dummy.GetWristConfig(5).speedRequested);
    dummy.SetEnable(true);
    assert(dummy.IsEnabled());
    frames.clear(); dummy.DragTick(103);
    assert(!dummy.IsEnabled() && dummy.IsHolding());
    for (const auto& frame : frames)
        if ((frame.header.StdId >> 7) == 5)
            assert((frame.header.StdId & 0x7f) != 0x0B &&
                   (frame.header.StdId & 0x7f) != 0x0F);

    ascii("#WRIST_RESET 5", usb);
    assert(usb.output == "okw reset 5 gravity and local RAM opt-in off\r\n");
    assert(!dummy.GetWristConfig(5).frictionRequested);
    assert(!dummy.GetWristConfig(5).speedEnabled);

    // An explicit disable also latches the v2 path if its zero-assist frame is
    // lost, so a later START cannot fall back to an old J5 0x0B lease.
    frames.clear(); ascii("#WRIST_SPEED 5 1", usb);
    assert(usb.output == "okw 5 speed 1\r\n");
    dummy.SetEnable(true);
    failWristControlNode = 5;
    dummy.SetEnable(false);
    assert(dummy.GetWristConfig(5).speedRequested);
    assert(!dummy.GetWristConfig(5).protocolVerified);
    dummy.SetEnable(true);
    frames.clear(); dummy.DragTick(105);
    assert(!dummy.IsEnabled() && dummy.IsHolding());
    for (const auto& frame : frames)
        if ((frame.header.StdId >> 7) == 5)
            assert((frame.header.StdId & 0x7f) != 0x0B &&
                   (frame.header.StdId & 0x7f) != 0x0F);
    frames.clear(); ascii("#WRIST_RESET 5", usb);
    assert(usb.output == "okw reset 5 gravity and local RAM opt-in off\r\n");

    dummy.SetEnable(false);
    frames.clear();
    ascii("#WRIST_GRAVITY_MODEL 4 0.934275 0.010061 0.500", usb);
    assert(usb.output == "okw 4 gravity-model 0.934275 0.010061 0.500\r\n");
    ascii("#WRIST_HARMONIC 4 46 -84 -70 -33", usb);
    assert(usb.output == "okw 4 harmonic 46 -84 -70 -33\r\n");
    assert(dummy.GetWristConfig(4).gravityAPerNm == 0.934275f);
    assert(dummy.GetWristConfig(4).harmonicBaseMa[0] == 46);

    frames.clear();
    ascii("#WRIST_SPEED 3 1", usb); assert(usb.output.find("error") == 0);
    ascii("#WRIST_GRAVITY 4 1.01", usb); assert(usb.output.find("error") == 0);
    ascii("#WRIST_GRAVITY_MODEL 4 nan 0 0.5", usb);
    assert(usb.output.find("error") == 0);
    ascii("#GRAVITY_MODEL 2 -0.203 -0.011 3.3", usb);
    assert(usb.output.find("okg 2 gravity-model -0.203000 -0.011000 3.300") == 0);
    assert(dummy.gravityModel.GetJointGravityModel(2).currentPerNm == -0.203f);
    ascii("#GRAVITY 3 0", usb);
    assert(usb.output.find("okg 3 gravity 0.000") == 0);
    assert(dummy.gravityModel.GetJointGravityModel(3).scale == 0.0f);
    ascii("#GRAVITY 3 1", usb);
    ascii("#WRIST_HARMONIC 4 501 0 0 0", usb);
    assert(usb.output.find("error") == 0);
    ascii("#WRIST_FRICTION 4 0.5 501", usb); assert(usb.output.find("error") == 0);
    ascii("#WRIST_RESET 5 extra", usb); assert(usb.output.find("error") == 0);
    assert(frames.empty());

    // Malformed status/readback replies must not refresh the close-loop counters.
    const auto harmonicReplies = dummy.motorJ[4]->HarmonicCompensationReplyCount();
    const auto statusReplies = dummy.motorJ[4]->WristCompensationReplyCount();
    uint8_t malformed[8]{0xB4, 2, 4, 0, 47, 0, 1, 0};
    CAN_RxHeaderTypeDef malformedHeader{4U << 7 | 0x27, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
    OnCanMessage(&context, &malformedHeader, malformed);
    assert(dummy.motorJ[4]->WristCompensationReplyCount() == statusReplies);
    malformedHeader.StdId = 4U << 7 | 0x28; malformedHeader.DLC = 7;
    OnCanMessage(&context, &malformedHeader, malformed);
    assert(dummy.motorJ[4]->HarmonicCompensationReplyCount() == harmonicReplies);

    std::cout << "F4 protocol, queue, feedback timeout and recovery tests passed\n";
}
