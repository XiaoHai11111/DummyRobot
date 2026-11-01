#include "common_inc.h"

extern DummyRobot dummy;


void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    uint8_t argNum;//加
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            dummy.hand->SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            dummy.hand->SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "okj %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "okl %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        } else if (s.rfind("#GRIP_CURRENT", 0) == 0)
        {
            float current;
            if (sscanf(_cmd, "#GRIP_CURRENT %f", &current) == 1)
            {
                dummy.hand->SetMaxCurrent(current);
                Respond(_responseChannel, "Grip current %.2f", current);
            }
        } else if (s.rfind("#GRIP", 0) == 0)
        {
            float angle;
            if (sscanf(_cmd, "#GRIP %f", &angle) == 1)
            {
                dummy.hand->SetAngle(angle);
                Respond(_responseChannel, "Grip angle %.2f", angle);
            }
        }
    } else if (_cmd[0] == '>' || _cmd[0] == '@')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }else if (_cmd[0] == '&')//加
    {
        float joints[6];
        float speed;
        float hangle;
        float hcurrent;

        argNum = sscanf(_cmd, "&%f,%f,%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                        joints + 3, joints + 4, joints + 5, &speed, &hangle, &hcurrent);
        if (argNum == 6)
        {
            dummy.MoveJ(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5]);
        } else if (argNum == 7)
        {
            dummy.SetJointSpeed(speed);
            dummy.MoveJ(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5]);
        } else if (argNum == 8)
        {
            dummy.SetJointSpeed(speed);
            dummy.MoveJ(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5]);
            dummy.hand->SetAngle(hangle);
        } else if (argNum == 9)
        {
            dummy.SetJointSpeed(speed);
            dummy.MoveJ(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5]);
            dummy.hand->SetAngle(hangle);
            dummy.hand->SetMaxCurrent(hcurrent);
        }

        Respond(_responseChannel, "ok");

    }else if (_cmd[0] == '@')//加
    {
        float pose[6];
        float speed;

        argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                        pose + 3, pose + 4, pose + 5, &speed);
        if (argNum == 6)
        {
            dummy.MoveL(pose[0], pose[1], pose[2],
                        pose[3], pose[4], pose[5]);
        } else if (argNum == 7)
        {
            dummy.SetJointSpeed(speed);
            dummy.MoveL(pose[0], pose[1], pose[2],
                        pose[3], pose[4], pose[5]);
        }
        Respond(_responseChannel, "ok");
    }

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    uint8_t argNum;//加
    if (_cmd[0] == '!')
    {
        std::string s(_cmd);
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "okj %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "okl %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        } else if (s.rfind("#GRIP_CURRENT", 0) == 0)
        {
            float current;
            if (sscanf(_cmd, "#GRIP_CURRENT %f", &current) == 1)
            {
                dummy.hand->SetMaxCurrent(current);
                Respond(_responseChannel, "Grip current %.2f", current);
            }
        } else if (s.rfind("#GRIP", 0) == 0)
        {
            float angle;
            if (sscanf(_cmd, "#GRIP %f", &angle) == 1)
            {
                dummy.hand->SetAngle(angle);
                Respond(_responseChannel, "Grip angle %.2f", angle);
            }
        }
    } else if (_cmd[0] == '>' || _cmd[0] == '@')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }else if (_cmd[0] == '&' && dummy.IsEnabled())//加
    {
        float joints[6];
        float speed;

        argNum = sscanf(_cmd, "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                        joints + 3, joints + 4, joints + 5, &speed);
        if (argNum == 6)
        {
            dummy.MoveJ(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5]);
        } else if (argNum == 7)
        {
            dummy.SetJointSpeed(speed);
            dummy.MoveJ(joints[0], joints[1], joints[2],
                        joints[3], joints[4], joints[5]);
        }

        Respond(_responseChannel, "ok");

    }else if (_cmd[0] == '@' && dummy.IsEnabled())//加
    {
        float pose[6];
        float speed;

        argNum = sscanf(_cmd, "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                        pose + 3, pose + 4, pose + 5, &speed);
        if (argNum == 6)
        {
            dummy.MoveL(pose[0], pose[1], pose[2],
                        pose[3], pose[4], pose[5]);
        } else if (argNum == 7)
        {
            dummy.SetJointSpeed(speed);
            dummy.MoveL(pose[0], pose[1], pose[2],
                        pose[3], pose[4], pose[5]);
        }

        Respond(_responseChannel, "ok");
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
