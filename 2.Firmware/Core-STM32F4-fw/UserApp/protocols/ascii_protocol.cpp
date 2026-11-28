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
    } else if (text.rfind("#GRAVITY ", 0) == 0)
    {
        unsigned joint;
        float scale;
        char extra;
        if (sscanf(cmd, "#GRAVITY %u %f %c", &joint, &scale, &extra) != 2 ||
            joint < 1 || joint > 6 || !MotionFinite(scale) ||
            scale < 0 || scale > 1)
            Respond(response, "error invalid gravity (J1..J6, scale 0..1)");
        else if (dummy.SetGravity(joint, scale))
            Respond(response, "okg %u gravity %.3f", joint, scale);
        else
            Respond(response, "error gravity rejected");
    } else if (text.rfind("#GRAVITY_MODEL ", 0) == 0)
    {
        unsigned joint;
        float aPerNm, biasA, limitA;
        char extra;
        if (sscanf(cmd, "#GRAVITY_MODEL %u %f %f %f %c",
                   &joint, &aPerNm, &biasA, &limitA, &extra) != 4 ||
            joint < 1 || joint > 6 || !MotionFinite(aPerNm) ||
            !MotionFinite(biasA) || !MotionFinite(limitA) ||
            fabsf(aPerNm) > 10 || biasA < -1 || biasA > 1 ||
            limitA <= 0 || limitA > 3.3f)
            Respond(response, "error invalid gravity model (J1..J6)");
        else if (dummy.SetGravityModel(joint, aPerNm, biasA, limitA))
            Respond(response, "okg %u gravity-model %.6f %.6f %.3f",
                    joint, aPerNm, biasA, limitA);
        else
            Respond(response, "error gravity model rejected");
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
            const bool sent = dummy.motorJ[joint]->SetLocalCompensation(
                damping, coulomb, viscous, appliedDeadzone, limit,
                fields == 7 && store == 1);
            if (!sent)
            {
                dummy.SetEnable(false);
                Respond(response, "error local compensation CAN send failed");
            }
            else
            {
                if (joint == 4 || joint == 5)
                {
                    dummy.SetWristLocalLimit(joint, limit);
                    dummy.SetDragLocalLimit(joint, limit);
                }
                Respond(response, "Local compensation J%u %d %d %d %d %d%s",
                        joint, damping, coulomb, viscous, appliedDeadzone, limit,
                        fields == 7 && store == 1 ? " stored" : "");
            }
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
    } else if (text == "#SETALLPARAMS")
    {
        Respond(response, dummy.SetAllJointParams() ?
                "okall joint params stored" :
                "error set all joint params rejected: disable robot first");
    } else if (text.rfind("#GETCOMP ", 0) == 0)
    {
        unsigned joint;
        char extra;
        if (sscanf(cmd, "#GETCOMP %u %c", &joint, &extra) != 1 || joint < 1 || joint > 6)
            Respond(response, "error invalid joint (1..6)");
        else if (!dummy.motorJ[joint]->RefreshCompensationParams())
            Respond(response, "error compensation inquiry timeout");
        else
            Respond(response,
                    "okcomp %u local %d %d %d %d %d harm %d %d %d %d",
                    joint,
                    dummy.motorJ[joint]->ReadLocalDamping(),
                    dummy.motorJ[joint]->ReadLocalCoulomb(),
                    dummy.motorJ[joint]->ReadLocalViscous(),
                    dummy.motorJ[joint]->ReadLocalDeadzone(),
                    dummy.motorJ[joint]->ReadLocalLimit(),
                    dummy.motorJ[joint]->ReadHarmonicESin(),
                    dummy.motorJ[joint]->ReadHarmonicECos(),
                    dummy.motorJ[joint]->ReadHarmonicOSin(),
                    dummy.motorJ[joint]->ReadHarmonicOCos());
    } else if (text.rfind("#SETCOMP ", 0) == 0)
    {
        unsigned joint, store;
        int32_t damping, coulomb, viscous, deadzone, limit;
        int32_t eSin, eCos, oSin, oCos;
        char extra;
        const int fields = sscanf(cmd,
                                  "#SETCOMP %u %d %d %d %d %d %u "
                                  "%d %d %d %d %c",
                                  &joint, &damping, &coulomb, &viscous,
                                  &deadzone, &limit, &store,
                                  &eSin, &eCos, &oSin, &oCos, &extra);
        const int32_t local[5] = {damping, coulomb, viscous, deadzone, limit};
        if (fields != 11 || store > 1 || joint < 1 || joint > 6)
            Respond(response, "error invalid compensation command");
        else if (!dummy.SetLocalCompensation(joint, local, store == 1) ||
                 ((joint == 4 || joint == 5) &&
                  !dummy.SetWristHarmonicsStored(joint, eSin, eCos, oSin, oCos)))
            Respond(response, "error compensation rejected");
        else
        {
            if (joint == 4 || joint == 5)
            {
                dummy.SetWristLocalLimit(joint, limit);
                dummy.SetDragLocalLimit(joint, limit);
                dummy.wrist[joint].dragFrictionScale = damping > 0 ? 0.3f : 0.0f;
                const int16_t harmonics[4] = {
                    static_cast<int16_t>(eSin), static_cast<int16_t>(eCos),
                    static_cast<int16_t>(oSin), static_cast<int16_t>(oCos)};
                memcpy(dummy.wrist[joint].dragHarmonicMa, harmonics,
                       sizeof(dummy.wrist[joint].dragHarmonicMa));
            }
            Respond(response, "okcomp %u configured%s", joint, store == 1 ? " stored" : "");
        }
    } else if (text.rfind("#WRIST_GRAVITY ", 0) == 0)
    {
        unsigned joint;
        float scale;
        char extra;
        if (sscanf(cmd, "#WRIST_GRAVITY %u %f %c", &joint, &scale, &extra) != 2 ||
            joint < 4 || joint > 5 || !MotionFinite(scale) || scale < 0 || scale > 1)
            Respond(response, "error invalid wrist gravity (J4/J5, scale 0..1)");
        else if (dummy.SetWristGravity(joint, scale))
            Respond(response, "okw %u gravity %.3f", joint, scale);
        else
            Respond(response, "error wrist gravity rejected");
    } else if (text.rfind("#WRIST_GRAVITY_MODEL ", 0) == 0)
    {
        unsigned joint;
        float aPerNm, biasA, limitA;
        char extra;
        if (sscanf(cmd, "#WRIST_GRAVITY_MODEL %u %f %f %f %c",
                   &joint, &aPerNm, &biasA, &limitA, &extra) != 4 ||
            joint < 4 || joint > 5 || !MotionFinite(aPerNm) ||
            !MotionFinite(biasA) || !MotionFinite(limitA) ||
            aPerNm < 0 || aPerNm > 10 || biasA < -1 || biasA > 1 ||
            limitA <= 0 || limitA > 3.3f)
            Respond(response, "error invalid wrist gravity model");
        else if (dummy.SetWristGravityModel(joint, aPerNm, biasA, limitA))
            Respond(response, "okw %u gravity-model %.6f %.6f %.3f",
                    joint, aPerNm, biasA, limitA);
        else
            Respond(response, "error wrist gravity model rejected");
    } else if (text.rfind("#WRIST_HARMONIC ", 0) == 0)
    {
        unsigned joint;
        int32_t eSin, eCos, oSin, oCos;
        char extra;
        if (sscanf(cmd, "#WRIST_HARMONIC %u %d %d %d %d %c",
                   &joint, &eSin, &eCos, &oSin, &oCos, &extra) != 5 ||
            joint < 4 || joint > 5 ||
            eSin < -500 || eSin > 500 || eCos < -500 || eCos > 500 ||
            oSin < -500 || oSin > 500 || oCos < -500 || oCos > 500)
            Respond(response, "error invalid wrist harmonics (mA, each +/-500)");
        else if (dummy.SetWristHarmonics(joint, eSin, eCos, oSin, oCos))
            Respond(response, "okw %u harmonic %ld %ld %ld %ld", joint,
                    static_cast<long>(eSin), static_cast<long>(eCos),
                    static_cast<long>(oSin), static_cast<long>(oCos));
        else
            Respond(response, "error wrist harmonics rejected while moving");
    } else if (text.rfind("#WRIST_SPEED ", 0) == 0)
    {
        unsigned joint, enable;
        char extra;
        if (sscanf(cmd, "#WRIST_SPEED %u %u %c", &joint, &enable, &extra) != 2 ||
            joint < 4 || joint > 5 || enable > 1)
            Respond(response, "error invalid wrist speed (J4/J5, enable 0/1)");
        else if (dummy.SetWristSpeed(joint, enable == 1))
            Respond(response, "okw %u speed %u", joint, enable);
        else
            Respond(response, "error wrist protocol/readback timeout; compensation remains off");
    } else if (text.rfind("#WRIST_FRICTION ", 0) == 0)
    {
        unsigned joint;
        float scale;
        int32_t limit;
        char extra;
        if (sscanf(cmd, "#WRIST_FRICTION %u %f %d %c", &joint, &scale, &limit, &extra) != 3 ||
            joint < 4 || joint > 5 || !MotionFinite(scale) || scale < 0 || scale > 1 ||
            limit < 1 || limit > 500)
            Respond(response, "error invalid wrist friction (J4/J5, scale 0..1, limit 1..500)");
        else if (!dummy.SetWristLocalLimit(joint, limit))
            Respond(response, "error invalid wrist local limit");
        else if (dummy.SetWristFriction(joint, scale))
            Respond(response, "okw %u friction %.3f limit %ld", joint, scale,
                    static_cast<long>(limit));
        else
            Respond(response, "error wrist protocol/readback timeout; compensation remains off");
    } else if (text.rfind("#GETWRIST ", 0) == 0)
    {
        unsigned joint;
        char extra;
        if (sscanf(cmd, "#GETWRIST %u %c", &joint, &extra) != 1 || joint < 4 || joint > 5)
            Respond(response, "error invalid wrist joint (4/5)");
        else if (!dummy.RefreshWristStatus(joint))
            Respond(response, "error wrist inquiry timeout (requires new F1 protocol)");
        else
        {
            const auto& wrist = dummy.GetWristConfig(joint);
            const auto& gravityModel = dummy.gravityModel.GetJointGravityModel(joint);
            Respond(response, "okw %u g%.3f gm%.6f,%.6f,%.3f f%.3f fs%.3f "
                    "s%u flags%u dir%d current%d seq%u harm%d,%d,%d,%d",
                    joint, wrist.gravityRequested ? wrist.gravityScale : 0.0f,
                    gravityModel.currentPerNm, gravityModel.biasA, gravityModel.limitA,
                    wrist.frictionEnabled ? wrist.frictionScale : 0.0f,
                    wrist.frictionScale,
                    unsigned(wrist.speedEnabled), unsigned(wrist.statusFlags),
                    int(wrist.direction), int(wrist.actualAssistMa), unsigned(wrist.statusSequence),
                    int(wrist.harmonicReadbackMa[0]), int(wrist.harmonicReadbackMa[1]),
                    int(wrist.harmonicReadbackMa[2]), int(wrist.harmonicReadbackMa[3]));
        }
    } else if (text == "#WRIST_RESET" || text.rfind("#WRIST_RESET ", 0) == 0)
    {
        unsigned joint = 0;
        char extra;
        const int fields = sscanf(cmd, "#WRIST_RESET %u %c", &joint, &extra);
        if (fields != EOF && fields != 1)
            Respond(response, "error invalid wrist reset (optional J4/J5)");
        else if (joint == 0)
        {
            if (dummy.ResetWristCompensation(4) &&
                dummy.ResetWristCompensation(5))
                Respond(response, "okw reset 4 5 gravity and local RAM opt-in off");
            else
                Respond(response, "error wrist reset not confirmed; assist remains off");
        } else if (joint >= 4 && joint <= 5)
        {
            if (dummy.ResetWristCompensation(joint))
                Respond(response, "okw reset %u gravity and local RAM opt-in off", joint);
            else
                Respond(response, "error wrist reset not confirmed; assist remains off");
        } else
            Respond(response, "error invalid wrist joint (4/5)");
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
