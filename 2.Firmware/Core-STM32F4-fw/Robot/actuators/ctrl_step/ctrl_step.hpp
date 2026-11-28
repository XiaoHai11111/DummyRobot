#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP

#include "fibre/protocol.hpp"
#include "can.h"

class CtrlStepMotor
{
public:
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };


    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    CtrlStepMotor(CAN_HandleTypeDef* _hcan, uint8_t _id, bool _inverse = false, uint8_t _reduction = 1,
                  float _angleLimitMin = -180, float _angleLimitMax = 180);

    uint8_t nodeID;
    float angle = 0;
    float angleLimitMax;
    float angleLimitMin;
    bool inverseDirection;
    uint8_t reduction;
    State state = STOP;
    volatile uint32_t lastFeedbackTick = 0;
    volatile uint32_t feedbackCount = 0;
    volatile uint16_t controlLastCycles = 0, controlMaxCycles = 0;
    volatile uint32_t controlOverruns = 0, timingReplyCount = 0;
    volatile bool feedbackReceived = false;
    volatile bool statusSupported = false;
    volatile uint8_t driverState = 0;
    volatile uint8_t faultFlags = 0;

    void SetAngle(float _angle);
    void SetAngleWithVelocityLimit(float _angle, float _vel);
    // CAN Command
    bool SetEnable(bool _enable);
    void DoCalibration();
    void SetCurrentSetPoint(float _val);
    bool SetCurrentFeedForward(float _val, uint16_t _seq, uint16_t _leaseMs);
    bool SetDragCurrent(float _val, uint16_t _seq, uint16_t _leaseMs);
    bool SetExtendedDragCurrent(int16_t baseMa, uint16_t totalLimitMa,
                                uint16_t seq, uint16_t leaseMs,
                                bool disablePositiveMotor,
                                bool disableNegativeMotor);
    bool SetHarmonicCompensation(const int16_t valuesMa[4]);
    bool SetHarmonicCompensation(const int16_t valuesMa[4], bool storeToEeprom);
    bool SetWristCompensation(bool enable, uint8_t gainPercent,
                              uint16_t maxAssistMa, uint16_t seq);
    bool RequestHarmonicCompensationReadback();
    bool RequestWristCompensationStatus();
    bool QueryHarmonicCompensation(int16_t valuesMa[4], uint32_t timeoutMs = 50);
    bool RequestCompensationParams();
    bool QueryCompensationParams(uint32_t timeoutMs = 50);
    void UpdateCompensationParams(uint8_t page, const uint8_t data[8]);
    struct WristCompensationStatus
    {
        uint8_t flags;
        int8_t direction;
        int16_t actualAssistMa;
        uint16_t sequence;
    };
    bool QueryWristCompensationStatus(WristCompensationStatus& status, uint32_t timeoutMs = 50);
    void UpdateHarmonicCompensationReadback(const int16_t valuesMa[4]);
    void UpdateWristCompensationStatus(uint8_t flags, int8_t direction,
                                       int16_t actualAssistMa, uint16_t sequence);
    uint32_t HarmonicCompensationReplyCount() const { return harmonicReplyCount; }
    uint8_t CompensationParamsMask() const { return compensationParamsPageMask; }
    uint32_t WristCompensationReplyCount() const { return wristCompensationReplyCount; }
    void RequestVelocityFeedback();
    void RequestPositionFeedback();
    void SetVelocitySetPoint(float _val);
    void SetPositionSetPoint(float _val);
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    void SetNodeID(uint32_t _id);
    void SetCurrentLimit(float _val);
    void SetVelocityLimit(float _val);
    void SetAcceleration(float _val);
    void SetDceKp(int32_t _val);
    void SetDceKv(int32_t _val);
    void SetDceKi(int32_t _val);
    void SetDceKd(int32_t _val);
    bool SetLocalCompensation(int32_t damping, int32_t coulomb,
                              int32_t viscous, int32_t deadzone, int32_t limit,
                              bool storeToEeprom = false);
    int32_t ReadLocalDamping() const { return compensationParams[0]; }
    int32_t ReadLocalCoulomb() const { return compensationParams[1]; }
    int32_t ReadLocalViscous() const { return compensationParams[2]; }
    int32_t ReadLocalDeadzone() const { return compensationParams[3]; }
    int32_t ReadLocalLimit() const { return compensationParams[4]; }
    int32_t ReadHarmonicESin() const { return harmonicParams[0]; }
    int32_t ReadHarmonicECos() const { return harmonicParams[1]; }
    int32_t ReadHarmonicOSin() const { return harmonicParams[2]; }
    int32_t ReadHarmonicOCos() const { return harmonicParams[3]; }
    bool SetLocalDamping(int32_t value, bool storeToEeprom = false);
    bool SetLocalCoulomb(int32_t value, bool storeToEeprom = false);
    bool SetLocalViscous(int32_t value, bool storeToEeprom = false);
    bool SetLocalDeadzone(int32_t value, bool storeToEeprom = false);
    bool SetLocalLimit(int32_t value, bool storeToEeprom = false);
    bool SetStoredHarmonics(int32_t eSin, int32_t eCos, int32_t oSin, int32_t oCos);
    bool RefreshCompensationParams();
    void ApplyPositionAsHome();
    void SetEnableOnBoot(bool _enable);
    void SetEnableStallProtect(bool _enable);
    void Reboot();
    void EraseConfigs();

    void UpdateAngle();
    void UpdateAngleCallback(float _pos, bool _isFinished);
    void UpdateStatus(uint8_t _state, uint8_t _faults, bool _supported);
    uint32_t FeedbackAge() const;
    uint32_t VelocityAge() const;
    float GetVelocityDegS() const;
    bool ClearFaults();
    bool ReadControlTiming();
    void UpdateControlTiming(uint16_t last, uint16_t peak, uint32_t overruns);
    // Diagnostic samples are asynchronous CAN replies, not an F1 atomic snapshot.
    struct Telemetry
    {
        float positionDeg, velocityDegS, currentCommandA;
        uint32_t positionTick, velocityTick, currentTick;
        uint8_t driverState, faultFlags;
        bool statusSupported;
    };
    bool ReadTelemetry(Telemetry& sample);
    void UpdateCurrentCommand(float amps, uint8_t faults);
    void UpdateVelocity(float turnsPerSecond, uint8_t faults);
    bool Hold();


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_ro_property("angle", &angle),
            make_protocol_ro_property("local_damping_mA_per_motorRevS", &compensationParams[0]),
            make_protocol_ro_property("local_coulomb_mA", &compensationParams[1]),
            make_protocol_ro_property("local_viscous_mA_per_motorRevS", &compensationParams[2]),
            make_protocol_ro_property("local_deadzone_motorRevS_x1000", &compensationParams[3]),
            make_protocol_ro_property("local_comp_limit_mA", &compensationParams[4]),
            make_protocol_ro_property("harmonic_e_sin_mA", &harmonicParams[0]),
            make_protocol_ro_property("harmonic_e_cos_mA", &harmonicParams[1]),
            make_protocol_ro_property("harmonic_o_sin_mA", &harmonicParams[2]),
            make_protocol_ro_property("harmonic_o_cos_mA", &harmonicParams[3]),
            make_protocol_function("reboot", *this, &CtrlStepMotor::Reboot),
            make_protocol_function("erase_configs", *this, &CtrlStepMotor::EraseConfigs),
            make_protocol_function("set_enable", *this, &CtrlStepMotor::SetEnable, "enable"),
            make_protocol_function("set_position_with_time", *this,
                                   &CtrlStepMotor::SetPositionWithVelocityLimit, "pos", "time"),
            make_protocol_function("set_position", *this, &CtrlStepMotor::SetPositionSetPoint, "pos"),
            make_protocol_function("set_velocity", *this, &CtrlStepMotor::SetVelocitySetPoint, "vel"),
            make_protocol_function("set_velocity_limit", *this, &CtrlStepMotor::SetVelocityLimit, "vel"),
            make_protocol_function("set_current", *this, &CtrlStepMotor::SetCurrentSetPoint, "current"),
            make_protocol_function("set_current_limit", *this, &CtrlStepMotor::SetCurrentLimit, "current"),
            make_protocol_function("set_node_id", *this, &CtrlStepMotor::SetNodeID, "id"),
            make_protocol_function("set_acceleration", *this, &CtrlStepMotor::SetAcceleration, "acc"),
            make_protocol_function("apply_home_offset", *this, &CtrlStepMotor::ApplyPositionAsHome),
            make_protocol_function("do_calibration", *this, &CtrlStepMotor::DoCalibration),
            make_protocol_function("set_enable_on_boot", *this, &CtrlStepMotor::SetEnableOnBoot, "enable"),
            make_protocol_function("set_dce_kp", *this, &CtrlStepMotor::SetDceKp, "vel"),
            make_protocol_function("set_dce_kv", *this, &CtrlStepMotor::SetDceKv, "vel"),
            make_protocol_function("set_dce_ki", *this, &CtrlStepMotor::SetDceKi, "vel"),
            make_protocol_function("set_dce_kd", *this, &CtrlStepMotor::SetDceKd, "vel"),
            make_protocol_function("set_enable_stall_protect", *this, &CtrlStepMotor::SetEnableStallProtect,
                                   "enable"),
            make_protocol_function("set_local_damping", *this,
                                   &CtrlStepMotor::SetLocalDamping, "value", "store"),
            make_protocol_function("set_local_coulomb", *this,
                                   &CtrlStepMotor::SetLocalCoulomb, "value", "store"),
            make_protocol_function("set_local_viscous", *this,
                                   &CtrlStepMotor::SetLocalViscous, "value", "store"),
            make_protocol_function("set_local_deadzone", *this,
                                   &CtrlStepMotor::SetLocalDeadzone, "value", "store"),
            make_protocol_function("set_local_limit", *this,
                                   &CtrlStepMotor::SetLocalLimit, "value", "store"),
            make_protocol_function("set_harmonics", *this,
                                   &CtrlStepMotor::SetStoredHarmonics,
                                   "e_sin", "e_cos", "o_sin", "o_cos"),
            make_protocol_function("refresh_compensation_params", *this,
                                   &CtrlStepMotor::RefreshCompensationParams),
            make_protocol_function("update_angle", *this, &CtrlStepMotor::UpdateAngle)
        );
    }


private:
    volatile float currentCommandA = 0, velocityTurnsPerSecond = 0;
    volatile uint32_t currentTick = 0, velocityTick = 0;
    volatile uint32_t currentReplyCount = 0, velocityReplyCount = 0;
    volatile uint8_t currentFaults = 0, velocityFaults = 0;
    volatile int16_t harmonicReadbackMa[4]{};
    volatile uint8_t wristStatusFlags = 0;
    volatile int8_t wristDirection = 0;
    volatile int16_t wristActualAssistMa = 0;
    volatile uint16_t wristStatusSequence = 0;
    volatile uint32_t harmonicReplyCount = 0;
    volatile uint32_t wristCompensationReplyCount = 0;
    volatile uint32_t compensationParamsReplyCount = 0;
    volatile uint8_t compensationParamsPageMask = 0;
    int32_t compensationParams[5]{};
    int32_t harmonicParams[4]{};
    CAN_HandleTypeDef* hcan;
    const CAN_TxHeaderTypeDef baseHeader = {
        .StdId = 0,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
