#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include <string>

#include "algorithms/kinematic/6dof_kinematic.h"
#include "algorithms/gravity_compensation/gravity_compensation.h"
#include "actuators/ctrl_step/ctrl_step.hpp"

#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 3               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 3               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 3               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 3               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 3               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 3               | 30             | 1000     | 80       | 200      | 250      |
 */

/*
  Stage 2 F1 local compensation candidate parameters. The columns, order and
  units are exactly the same as F1 BoardConfig_t and #LOCALCOMP:

  #LOCALCOMP <joint> <damping> <coulomb> <viscous> <deadzone> <limit> [store]

  damping: mA per motor rev/s; coulomb: mA; viscous: mA per motor rev/s;
  deadzone: motor rev/s x1000; limit: mA. F1 constructor defaults are all zero;
  main.cpp loads these values from BoardConfig_t at boot. Zero limit disables
  stage-2 compensation, which only acts while the leased drag current is valid.

  The values below are the fixed-point BoardConfig_t values converted from the
  fitted joint-domain coefficients using each joint's reduction. Damping has no
  fitted result and stays at zero. The 500 mA limit is a conservative
  engineering cap for the correction term, not a fitted value. Deadzone must be
  above the F1 encoder-speed quantization (~0.012 motor rev/s); the values below
  correspond to ~0.3 joint deg/s at each reduction and are clamped by F4/F1.

  |   PARAMS   | `damping` | `coulomb` | `viscous` | `deadzone` | `limit` |
  | ---------- | --------- | --------- | --------- | ---------- | ------- |
  | **Joint1** | 0         | 77        | 24        | 42         | 500     |
  | **Joint2** | 0         | 125       | 69        | 25         | 500     |
  | **Joint3** | 0         | 102       | 19        | 25         | 500     |
  | **Joint4** | 0         | 250       | 54        | 25         | 500     |
  | **Joint5** | 0         | 185       | 44        | 25         | 500     |
  | **Joint6** | 0         | 167       | 0         | 42         | 500     |
 */


class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 0.7;


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);


    void SetAngle(float _angle);
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    const CAN_TxHeaderTypeDef baseHeader = {
        .StdId = 0, .ExtId = 0, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA,
        .DLC = 8, .TransmitGlobalTime = DISABLE
    };
    float minAngle = 0;
    float maxAngle = 45;
};


class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING,
        COMMAND_DRAG = 5
    };


    // Indexed by joint 1..6. Derived from the F1 1/32 velocity filter and each
    // joint reduction so #LOCALCOMP cannot inject Coulomb assist on encoder dither.
    static constexpr int32_t LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000[7] = {
        0, 42, 25, 25, 25, 25, 42
    };

    // Robot-wide F1 deployment tables. Rows are indexed by joint 1..6 and row 0
    // is unused. The F1 firmwares only need their CAN ID burned; F4 writes these
    // values to every board through SetAllJointParams().
    static constexpr int32_t DCE_KP[7] = {0, 1000, 1000, 1500, 1000, 1000, 1000};
    static constexpr int32_t DCE_KV[7] = {0, 80, 80, 80, 80, 80, 80};
    static constexpr int32_t DCE_KI[7] = {0, 200, 200, 200, 200, 200, 200};
    static constexpr int32_t DCE_KD[7] = {0, 250, 200, 250, 250, 250, 250};

    // Columns: damping, coulomb, viscous, deadzone, limit.
    static constexpr int32_t LOCAL_COMP[7][5] = {
        {0, 0, 0, 0, 0},
        {0, 77, 24, 42, 500},
        {0, 125, 69, 25, 500},
        {0, 102, 19, 25, 500},
        {0, 250, 54, 25, 500},
        {0, 185, 44, 25, 500},
        {0, 167, 0, 42, 500}
    };

    // Columns: E_sin, E_cos, O_sin, O_cos in motor phase. J5 uses the validated
    // C group, where all four harmonic coefficients are zero.
    static constexpr int16_t ALL_HARMONICS_MA[7][4] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {46, -84, -70, -33},
        {-25, 57, 41, 22},
        {0, 0, 0, 0}
    };


    struct Config_t
    {
        uint32_t controlFrequencyHz;
        uint16_t gravityFeedForwardHz;
        uint16_t feedForwardLeaseMs;
        uint16_t dragFeedbackHz;
        uint16_t dragLeaseMs;
        uint16_t dragVelocityValidMs;
        float dragVelocityFilterAlpha;
        float dragVelocitySmoothingDegS;
        float dragVelocityCutoffDegS;
        float dragEndStopMarginDeg;
        float dragDamping;
        float dragFrictionAlpha;
        float dragCurrentLimit;
        float dragDampingMax;
        float dragFrictionAlphaMax;
        float dragCurrentLimitMin;
        float dragCurrentLimitMax;
        bool gravityFeedForwardEnabled;

        Config_t();
    } config;

    struct WristAxisConfig_t
    {
        bool gravityRequested = false;
        float gravityScale = 1.0f;
        float gravityAPerNm = 0;
        float gravityBiasA = 0;
        float gravityLimitA = 0;
        bool frictionRequested = false;
        float frictionScale = 1.0f;
        int16_t harmonicBaseMa[4]{};
        int32_t frictionLimitMa = 0;
        bool speedRequested = false;
        bool speedEnabled = false;
        bool frictionEnabled = false;
        bool dragAutoEnabled = false;
        int32_t dragLocalLimitMa = 0;
        float dragFrictionScale = 0.0f;
        int16_t dragHarmonicMa[4]{};
        bool protocolVerified = false;
        bool controlVerified = false;
        bool safeOffVerified = true;
        bool harmonicVerified = false;
        bool oldProtocol = false;
        uint8_t appliedGainPercent = 0;
        uint16_t appliedCapMa = 0;
        int16_t harmonicMa[4]{};
        int16_t harmonicReadbackMa[4]{};
        uint16_t requestedSequence = 0;
        uint16_t statusSequence = 0;
        uint8_t statusFlags = 0;
        int8_t direction = 0;
        int16_t actualAssistMa = 0;
        uint32_t statusTick = 0;
    };
    WristAxisConfig_t wrist[7];
    static constexpr int16_t WRIST_BASE_HARMONIC_MA[2][4] = {
        {46, -84, -70, -33}, // J4 motor domain; candidate k/fit round to mA.
        {-25, 57, 41, 22}    // J5 motor domain; old unplugged-cable group.
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -75, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 30;  // degree/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};
    GravityCompensation::Model gravityModel;


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);
    void UpdateGravityFeedForward(uint32_t tick);
    void DragTick(uint32_t tick);
    void SetGravityFeedForwardEnabled(bool _enabled);
    bool IsGravityFeedForwardEnabled() const;
    void SetDragGains(float _damping, float _frictionAlpha, float _currentLimit);
    void GetDragGains(float* _gains) const;

    bool SetWristGravity(uint32_t joint, float scale);
    bool SetWristGravityModel(uint32_t joint, float aPerNm, float biasA, float limitA);
    bool SetGravity(uint32_t joint, float scale);
    bool SetGravityModel(uint32_t joint, float aPerNm, float biasA, float limitA);
    bool SetWristHarmonics(uint32_t joint, int32_t eSin, int32_t eCos,
                           int32_t oSin, int32_t oCos);
    bool SetWristHarmonicsStored(uint32_t joint, int32_t eSin, int32_t eCos,
                                 int32_t oSin, int32_t oCos);
    bool SetLocalCompensation(uint32_t joint, const int32_t values[5], bool storeToEeprom);
    bool SetAllJointParams();
    bool SetWristSpeed(uint32_t joint, bool enable);
    bool SetWristFriction(uint32_t joint, float scale);
    bool SetWristLocalLimit(uint32_t joint, int32_t limitMa);
    void SetDragLocalLimit(uint32_t joint, int32_t limitMa);
    bool RefreshWristStatus(uint32_t joint);
    bool ResetWristCompensation(uint32_t joint);
    const WristAxisConfig_t& GetWristConfig(unsigned joint) const { return wrist[joint]; }
    void UpdateWristHarmonicReadback(unsigned joint, const int16_t valuesMa[4]);
    void UpdateWristCompensationStatus(unsigned joint, uint8_t flags, int8_t direction,
                                       int16_t actualAssistMa, uint16_t sequence);
    void CheckSafety();
    bool ClearFaults();
    bool HasFault() const;
    uint8_t GetFaultJointMask() const;
    uint8_t GetJointFault(unsigned joint) const { return jointFaults[joint]; }
    uint32_t GetFaultFeedbackAge(unsigned joint) const { return faultFeedbackAge[joint]; }
    uint16_t GetFeedForwardSequence() const;
    uint16_t GetDragSequence() const;
    void StopMotion();
    bool WaitForMotion(uint32_t timeoutMs = 0);
    bool IsHolding() const { return holdOnStop; }


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("clear_faults", *this, &DummyRobot::ClearFaults),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_function("set_wrist_gravity", *this, &DummyRobot::SetWristGravity,
                                   "joint", "scale"),
            make_protocol_function("set_wrist_gravity_model", *this,
                                   &DummyRobot::SetWristGravityModel,
                                   "joint", "a_per_nm", "bias_a", "limit_a"),
            make_protocol_function("set_gravity", *this, &DummyRobot::SetGravity,
                                   "joint", "scale"),
            make_protocol_function("set_gravity_model", *this,
                                   &DummyRobot::SetGravityModel,
                                   "joint", "a_per_nm", "bias_a", "limit_a"),
            make_protocol_function("set_wrist_harmonics", *this,
                                   &DummyRobot::SetWristHarmonics,
                                   "joint", "e_sin", "e_cos", "o_sin", "o_cos"),
            make_protocol_function("set_wrist_speed", *this, &DummyRobot::SetWristSpeed,
                                   "joint", "enable"),
            make_protocol_function("set_wrist_friction", *this, &DummyRobot::SetWristFriction,
                                   "joint", "scale"),
            make_protocol_function("set_wrist_local_limit", *this,
                                   &DummyRobot::SetWristLocalLimit, "joint", "limit"),
            make_protocol_function("refresh_wrist_status", *this, &DummyRobot::RefreshWristStatus,
                                   "joint"),
            make_protocol_function("reset_wrist_compensation", *this,
                                   &DummyRobot::ResetWristCompensation, "joint"),
            make_protocol_function("set_all_joint_params", *this,
                                   &DummyRobot::SetAllJointParams),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions())
        );
    }


    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, sizeof(Entry), nullptr);
        }

        struct Entry
        {
            char text[128]{};
            StreamSink* response = nullptr;
            uint32_t generation = 0;
        };
        uint32_t Push(const std::string &_cmd, StreamSink* response = nullptr);
        Entry Pop(uint32_t timeout);
        uint32_t ParseCommand(const Entry &entry);
        uint32_t GetSpace();
        uint32_t GetGeneration() const { return generation; }
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        volatile uint32_t generation = 0;
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    bool MoveJAtGeneration(float j1, float j2, float j3, float j4, float j5, float j6, uint32_t generation);
    bool MoveLAtGeneration(float x, float y, float z, float a, float b, float c, uint32_t generation);
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    volatile bool isEnabled = false;
    volatile bool recovering = false;
    volatile bool holdOnStop = false;
    volatile uint8_t jointFaults[7]{};
    volatile uint32_t faultFeedbackAge[7]{};
    uint32_t safetyStarted = 0;
    uint32_t motionTimeoutMs = 30000;
    uint32_t enabledAt = 0;
    uint16_t feedForwardSequence = 0;
    uint16_t dragSequence = 0;
    uint16_t wristSequence = 0;
    float dragVelocityFiltered[7]{};
    float CalculateWristGravityCurrentA(uint8_t joint) const;
    bool ApplyWristCompensation(unsigned joint, bool enable);
    bool SendWristControl(unsigned joint, bool assistEnabled,
                          bool keepSpeedMeasurement, bool verify = true);
    bool ConfirmWristStatus(unsigned joint, uint16_t expectedSequence,
                            bool expectedEnabled);
    void ClearWristRuntimeState(unsigned joint, bool clearConfiguration);
    void SuspendWristCompensationNow();
    void DisableWristCompensationNow();
    void PollWristStatus(uint32_t tick);
    uint16_t NextWristSequence();
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
