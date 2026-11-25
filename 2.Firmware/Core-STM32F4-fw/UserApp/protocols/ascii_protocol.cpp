#include "common_inc.h"
#include "instances/motion_command.h"

extern DummyRobot dummy;

// USB and UART4 intentionally share the exact same command semantics.
static void OnRobotAsciiCmd(const char* cmd, size_t len, StreamSink &response)
{
    if (!len) return;
    std::string text(cmd, len);
    while (!text.empty() && isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
    cmd = text.c_str();
    if (text == "!STOP")
    {
        dummy.commandHandler.EmergencyStop();
        Respond(response, "Stopped ok");
    } else if (text == "!START")
    {
        dummy.SetEnable(true);
        if (dummy.IsEnabled())
        {
            dummy.hand->SetEnable(true);
            Respond(response, "Started ok");
        } else
            Respond(response, "error start blocked: check #GETSTATUS / !CLEARFAULT");
    } else if (text == "!DISABLE")
    {
        dummy.SetEnable(false);
        dummy.hand->SetEnable(false);
        Respond(response, "Disabled ok");
    } else if (text == "!CLEARFAULT")
    {
        Respond(response, dummy.ClearFaults() ? "Faults cleared ok" :
                "error clear failed: disable first, check drivers/CAN");
    } else if (text == "#GETSTATUS")
    {
        Respond(response, "oks %u %u %u %u %u %u %u %u %u",
                unsigned(dummy.IsEnabled()), unsigned(dummy.commandMode), unsigned(dummy.GetFaultJointMask()),
                unsigned(dummy.GetJointFault(1)), unsigned(dummy.GetJointFault(2)),
                unsigned(dummy.GetJointFault(3)), unsigned(dummy.GetJointFault(4)),
                unsigned(dummy.GetJointFault(5)), unsigned(dummy.GetJointFault(6)));
    } else if (text == "#GETCAN")
    {
        const auto ctx = get_can_ctx(&hcan1);
        Respond(response, "okc %lu %lu %lu %lu %lu %lu",
                static_cast<unsigned long>(ctx->received_msg_cnt),
                static_cast<unsigned long>(ctx->tx_msg_cnt),
                static_cast<unsigned long>(ctx->unexpected_errors),
                static_cast<unsigned long>(ctx->TxMailboxAbortCallbackCnt),
                static_cast<unsigned long>(ctx->last_error),
                static_cast<unsigned long>(ctx->bus_error_status));
    } else if (text.rfind("#GETDRV ", 0) == 0)
    {
        unsigned joint;
        char extra;
        if (sscanf(cmd, "#GETDRV %u %c", &joint, &extra) != 1 || joint < 1 || joint > 6)
            Respond(response, "error invalid joint (1..6)");
        else
        {
            const auto motor = dummy.motorJ[joint];
            Respond(response, "okd %u %lu %u %u %u %lu %lu", joint,
                    static_cast<unsigned long>(motor->feedbackReceived ? motor->FeedbackAge() : UINT32_MAX),
                    unsigned(motor->driverState), unsigned(motor->faultFlags), unsigned(dummy.GetJointFault(joint)),
                    static_cast<unsigned long>(dummy.GetFaultFeedbackAge(joint)),
                    static_cast<unsigned long>(motor->feedbackCount));
        }
    } else if (text.rfind("#GETTIMING ", 0) == 0)
    {
        unsigned joint;
        char extra;
        if (sscanf(cmd, "#GETTIMING %u %c", &joint, &extra) != 1 || joint < 1 || joint > 6)
            Respond(response, "error invalid joint (1..6)");
        else
        {
            const auto motor = dummy.motorJ[joint];
            if (!motor->ReadControlTiming())
                Respond(response, "error F1 timing response timeout (update F1)");
            else
                Respond(response, "okt %u %u %u %lu", joint, unsigned(motor->controlLastCycles),
                        unsigned(motor->controlMaxCycles), static_cast<unsigned long>(motor->controlOverruns));
        }
    } else if (text.rfind("#GETTELEM ", 0) == 0)
    {
        unsigned joint;
        char extra;
        if (sscanf(cmd, "#GETTELEM %u %c", &joint, &extra) != 1 || joint < 1 || joint > 6)
            Respond(response, "error invalid joint (1..6)");
        else
        {
            CtrlStepMotor::Telemetry sample{};
            if (!dummy.motorJ[joint]->ReadTelemetry(sample))
                Respond(response, "error F1 telemetry timeout (requires extended 0x21/0x22/0x23)");
            else
                Respond(response, "okm %u %lu %lu %lu %.5f %.5f %.4f %u %u %u", joint,
                        static_cast<unsigned long>(sample.positionTick),
                        static_cast<unsigned long>(sample.velocityTick),
                        static_cast<unsigned long>(sample.currentTick),
                        sample.positionDeg + dummy.initPose.a[joint - 1],
                        sample.velocityDegS, sample.currentCommandA,
                        unsigned(sample.driverState), unsigned(sample.faultFlags),
                        unsigned(sample.statusSupported));
        }
    } else if (text == "#GETJPOS")
    {
        Respond(response, "okj %.2f %.2f %.2f %.2f %.2f %.2f",
                dummy.currentJoints.a[0], dummy.currentJoints.a[1], dummy.currentJoints.a[2],
                dummy.currentJoints.a[3], dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
    } else if (text == "#GETLPOS")
    {
        Respond(response, "okl %.2f %.2f %.2f %.2f %.2f %.2f",
                dummy.currentPose6D.X, dummy.currentPose6D.Y, dummy.currentPose6D.Z,
                dummy.currentPose6D.A, dummy.currentPose6D.B, dummy.currentPose6D.C);
    } else if (text.rfind("#CMDMODE ", 0) == 0)
    {
        unsigned long mode;
        char extra;
        if (sscanf(cmd, "#CMDMODE %lu %c", &mode, &extra) != 1 || mode < 1 || mode > 5)
            Respond(response, "error invalid mode (1=SEQ 2=INT 3=TRJ 4=TUN 5=DRAG)");
        else
        {
            dummy.SetCommandMode(mode);
            Respond(response, "Set command mode to [%lu]", mode);
        }
    } else if (text == "#GRAVITY_FF_ON")
    {
        dummy.SetGravityFeedForwardEnabled(true);
        Respond(response, "Gravity feed-forward enabled, review required");
    } else if (text == "#GRAVITY_FF_OFF")
    {
        dummy.SetGravityFeedForwardEnabled(false);
        Respond(response, "Gravity feed-forward disabled");
    } else if (text.rfind("#DRAG_GAIN ", 0) == 0)
    {
        float damping, frictionAlpha, currentLimit;
        char extra;
        if (sscanf(cmd, "#DRAG_GAIN %f %f %f %c", &damping, &frictionAlpha,
                   &currentLimit, &extra) != 3 ||
            !MotionFinite(damping) || !MotionFinite(frictionAlpha) ||
            !MotionFinite(currentLimit) || damping < 0 || damping > 0.2f ||
            frictionAlpha < 0 || frictionAlpha > 1 ||
            currentLimit < 0.1f || currentLimit > 1.5f)
            Respond(response, "error invalid drag gains");
        else
        {
            dummy.SetDragGains(damping, frictionAlpha, currentLimit);
            Respond(response, "Drag gains %.3f %.3f %.3f", damping, frictionAlpha, currentLimit);
        }
    } else if (text.rfind("#LOCALCOMP ", 0) == 0)
    {
        unsigned joint, store;
        int32_t damping, coulomb, viscous, deadzone, limit;
        char extra;
        const int fields = sscanf(cmd, "#LOCALCOMP %u %d %d %d %d %d %u %c", &joint, &damping,
                                  &coulomb, &viscous, &deadzone, &limit, &store, &extra);
        const bool validFields = (fields == 6) || (fields == 7 && store <= 1);
        if (!validFields || joint < 1 || joint > 6 ||
            damping < 0 || damping > 1000000 || coulomb < 0 || coulomb > 3300 ||
            viscous < 0 || viscous > 1000000 || deadzone < 0 || deadzone > 1000000 ||
            limit < 0 || limit > 3300)
            Respond(response, "error invalid local compensation");
        else
        {
            const int32_t appliedDeadzone =
                deadzone < DummyRobot::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000[joint] ?
                DummyRobot::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000[joint] : deadzone;
            dummy.motorJ[joint]->SetLocalCompensation(damping, coulomb, viscous,
                                                       appliedDeadzone, limit,
                                                       fields == 7 && store == 1);
            Respond(response, "Local compensation J%u %d %d %d %d %d%s",
                    joint, damping, coulomb, viscous, appliedDeadzone, limit,
                    fields == 7 && store == 1 ? " stored" : "");
        }
    } else if (text == "#GETDRAG")
    {
        float gains[3];
        dummy.GetDragGains(gains);
        Respond(response, "okg %lu %d %.3f %.3f %.3f %u %u",
                static_cast<unsigned long>(dummy.commandMode),
                dummy.IsGravityFeedForwardEnabled() ? 1 : 0,
                gains[0], gains[1], gains[2],
                dummy.GetFeedForwardSequence(), dummy.GetDragSequence());
    } else if (text.rfind("#GRIP", 0) == 0)
    {
        float value;
        char extra;
        const bool current = text.rfind("#GRIP_CURRENT ", 0) == 0;
        const char* format = current ? "#GRIP_CURRENT %f %c" : "#GRIP %f %c";
        if (!dummy.IsEnabled() || dummy.HasFault())
            Respond(response, "error robot disabled or faulted");
        else if (sscanf(cmd, format, &value, &extra) != 1 || !MotionFinite(value) ||
                 value < 0 || value > (current ? 1.0f : 45.0f))
            Respond(response, "error invalid gripper parameter");
        else if (current)
        {
            dummy.hand->SetMaxCurrent(value);
            Respond(response, "Grip current %.2f", value);
        } else
        {
            dummy.hand->SetAngle(value);
            Respond(response, "Grip angle %.2f", value);
        }
    } else if ((cmd[0] == '>' || cmd[0] == '@') && dummy.IsEnabled())
    {
        const uint32_t freeSize = dummy.commandHandler.Push(text, &response);
        if (freeSize == 0xFF)
            Respond(response, "error invalid motion, queue full or faulted");
        else
            Respond(response, "%lu", static_cast<unsigned long>(freeSize));
    } else
        Respond(response, "error unknown command or robot disabled");
}

void OnUsbAsciiCmd(const char* cmd, size_t len, StreamSink &response)
{
    OnRobotAsciiCmd(cmd, len, response);
}

void OnUart4AsciiCmd(const char* cmd, size_t len, StreamSink &response)
{
    OnRobotAsciiCmd(cmd, len, response);
}

void OnUart5AsciiCmd(const char* cmd, size_t len, StreamSink &response)
{
    // UART5 is reserved for application-specific peripherals.
}
