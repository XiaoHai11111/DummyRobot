#include "communication.hpp"
#include "instances/dummy_robot.h"
#include <cassert>
#include <array>
#include <limits>
#include <iostream>

static uint32_t now = 0;
static bool acknowledge = true, transmit = true;
static bool feedbackOnClockRead = false;
struct Frame { CAN_TxHeaderTypeDef header; std::array<uint8_t, 8> data; };
static std::vector<Frame> frames;
CAN_HandleTypeDef handle, hcan1;
static bool preemptSend = false, replyToTiming = true;
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
    dummy.CheckSafety();
}
bool CanSendMessage(CAN_context*, uint8_t* data, CAN_TxHeaderTypeDef* header) {
    if (preemptSend) {
        preemptSend = false;
        dummy.motorJ[1]->SetCurrentLimit(0.5f); // Another producer while the CAN mutex is awaited.
    }
    if ((header->StdId & 0x7f) == 0x25 && replyToTiming)
        dummy.motorJ[header->StdId >> 7]->UpdateControlTiming(1200, 1900, 2);
    Frame f{*header, {}}; memcpy(f.data.data(), data, 8); frames.push_back(f); return transmit;
}
float value(const Frame &f, unsigned offset) { float v; memcpy(&v, f.data.data() + offset, 4); return v; }
void ascii(const char* text, StreamSink &sink, bool uart = false) {
    sink.output.clear();
    if (uart) OnUart4AsciiCmd(text, strlen(text), sink);
    else OnUsbAsciiCmd(text, strlen(text), sink);
}
int main() {
    dummy.Init(); fresh(); dummy.UpdateJointAnglesCallback(); dummy.SetEnable(true);
    assert(dummy.IsEnabled());
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
    now = 0; fresh();
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
    dummy.SetCommandMode(2);

    // Wrong node, short, remote, extended and NaN feedback must not index/refresh a joint.
    uint8_t data[8]{}; data[7] = 0xA1;
    CAN_RxHeaderTypeDef header{15U << 7 | 0x23, 0, CAN_ID_STD, CAN_RTR_DATA, 8, 0};
    OnCanMessage(&context, &header, data);
    header.StdId = 1U << 7 | 0x23; header.DLC = 4;
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
    std::cout << "F4 protocol, queue, feedback timeout and recovery tests passed\n";
}
