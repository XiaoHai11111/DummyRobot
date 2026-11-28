#ifndef CTRL_STEP_FW_MOTOR_H
#define CTRL_STEP_FW_MOTOR_H

#include "Motor/motion_planner.h"
#include "Sensor/Encoder/encoder_base.h"
#include "Driver/driver_base.h"

class Motor
{
public:
    Motor() :
        controller(&controllerInstance)
    {
        /****************** Default Configs *******************/
        config.motionParams.encoderHomeOffset = 0;
        config.motionParams.caliCurrent = 2000;             // (mA)
        config.motionParams.ratedCurrent = 1000;            // (mA)
        config.motionParams.ratedCurrentAcc = 2 * 1000;     // (mA/s)
        config.motionParams.ratedVelocity = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        config.motionParams.ratedVelocityAcc = 1000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        config.ctrlParams.stallProtectSwitch = false;
        config.ctrlParams.feedForward =
            Controller::FeedForwardConfig_t{
                .maxCurrent = 3300,             // (mA) RAM-only command ceiling
                .dragCurrentAcc = 20 * 1000,    // (mA/s), faster than position-current ramps
                .minLeaseMs = 10,
                .maxLeaseMs = 1000
            };
        config.ctrlParams.localComp =
            Controller::LocalCompensationConfig_t{
                .damping_mA_per_motorRevS = 0,
                .coulomb_mA = 0,
                .viscous_mA_per_motorRevS = 0,
                .deadzoneSteps = 0,
                .limit_mA = 0,
                .assistCoeffQ16 = 0,
                .rampInvQ16 = 0
            };
        config.ctrlParams.pid =
            Controller::PID_t{
                .kp = 5,
                .ki = 30,
                .kd = 0
            };
        config.ctrlParams.dce =
            Controller::DCE_t{
                .kp = 200,
                .kv = 80,
                .ki = 300,
                .kd = 250
            };

        /*****************************************************/


        motionPlanner.AttachConfig(&config.motionParams);
        controller->AttachConfig(&config.ctrlParams);
    }


    const int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200; // for 1.8° step-motors
    const int32_t SOFT_DIVIDE_NUM = 256;
    const int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM;

    typedef enum
    {
        MODE_STOP,
        MODE_COMMAND_POSITION,
        MODE_COMMAND_VELOCITY,
        MODE_COMMAND_CURRENT,
        MODE_COMMAND_Trajectory,
        MODE_PWM_POSITION,
        MODE_PWM_VELOCITY,
        MODE_PWM_CURRENT,
        MODE_STEP_DIR,
    } Mode_t;

    typedef enum
    {
        STATE_STOP,
        STATE_FINISH,
        STATE_RUNNING,
        STATE_OVERLOAD,
        STATE_STALL,
        STATE_NO_CALIB,
        STATE_CONTROL_OVERRUN
    } State_t;


    class Controller
    {
    public:
        friend Motor;

        // One encoder step per 20 kHz tick becomes ~625 steps/s after the 1/32
        // velocity filter, i.e. ~12 in BoardConfig units. 20 leaves margin.
        static constexpr int32_t LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000 = 20;
        static constexpr uint16_t LOCAL_COMP_SPEED_WINDOW_TICKS = 256;
        static constexpr uint8_t LOCAL_COMP_DIRECTION_CONFIRM_WINDOWS = 2;
        static constexpr int32_t LOCAL_COMP_RAMP_UP_Q16_PER_TICK = 656;
        static constexpr int32_t LOCAL_COMP_RAMP_DOWN_Q16_PER_TICK = 3277;
        static constexpr uint16_t LOCAL_COMP_ASSIST_CAP_LIMIT_mA = 500;
        static constexpr int16_t MOTOR_DOMAIN_HARMONIC_LIMIT_mA = 500;

        typedef struct
        {
            bool kpValid, kiValid, kdValid;
            int32_t kp, ki, kd;
            int32_t vError, vErrorLast;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } PID_t;

        typedef struct
        {
            int32_t kp, kv, ki, kd;
            int32_t pError, vError;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } DCE_t;

        typedef struct
        {
            int32_t maxCurrent;    // (mA)
            int32_t dragCurrentAcc; // (mA/s), only for leased drag commands
            uint16_t minLeaseMs;
            uint16_t maxLeaseMs;
        } FeedForwardConfig_t;

        typedef struct
        {
            int32_t damping_mA_per_motorRevS;   // opposes local motor velocity
            int32_t coulomb_mA;                 // assists in velocity direction
            int32_t viscous_mA_per_motorRevS;   // assists in velocity direction
            int32_t deadzoneSteps;              // estVelocity threshold, steps/s
            int32_t limit_mA;                   // 0 disables local compensation
            int32_t assistCoeffQ16;              // (viscous - damping) / steps per rev, Q16
            int32_t rampInvQ16;                  // 65536 / deadzoneSteps, Q16
        } LocalCompensationConfig_t;

        typedef struct
        {
            PID_t pid;
            DCE_t dce;
            FeedForwardConfig_t feedForward;
            LocalCompensationConfig_t localComp;

            bool stallProtectSwitch;
        } Config_t;


        explicit Controller(Motor* _context)
        {
            context = _context;

            requestMode = MODE_STOP;
            modeRunning = MODE_STOP;
        }


        Config_t* config = nullptr;
        Mode_t requestMode;
        Mode_t modeRunning;
        State_t state = STATE_STOP;
        bool isStalled = false;


        void Init();
        void SetCtrlMode(Mode_t _mode);
        void SetCurrentSetPoint(int32_t _cur);
        bool RequestCurrentFeedForward(float _amps, uint16_t _seq, uint16_t _leaseMs);
        bool RequestDragCurrent(float _amps, uint16_t _seq, uint16_t _leaseMs);
        bool RequestExtendedDragCurrent(int16_t _baseCurrent_mA,
                                       uint16_t _totalLimit_mA,
                                       uint16_t _sequence,
                                       uint16_t _leaseFlags);
        bool SetLocalCompensationParam(uint8_t _index, int32_t _value, bool _storeToEeprom = false);
        void GetLocalCompensationParams(int32_t values[5]) const;
        bool SetLocalCompOptIn(bool _enable, uint8_t _gainPercent,
                               uint16_t _sequence, uint16_t _assistCap_mA);
        bool SetMotorDomainHarmonics(int16_t _eSin, int16_t _eCos,
                                     int16_t _oSin, int16_t _oCos,
                                     bool _storeToEeprom = false);
        void GetMotorDomainHarmonics(int16_t& _eSin, int16_t& _eCos,
                                     int16_t& _oSin, int16_t& _oCos) const;
        bool IsLocalCompOptInEnabled() const;
        uint8_t GetLocalCompGainPercent() const;
        uint16_t GetLocalCompAssistCap_mA() const;
        bool IsAccurateVelocityReady() const;
        int8_t GetMotionDirection() const;
        int32_t GetLocalCompensationCurrent_mA() const;
        uint16_t GetLocalCompSequence() const;
        void ClearFeedForward();
        int32_t GetFeedbackCurrent_mA() const;
        int32_t GetFeedForwardCurrent_mA() const;
        int32_t GetTotalCurrent_mA() const;
        uint16_t GetFeedForwardSequence() const;
        uint16_t GetDragSequence() const;
        uint16_t GetRemainingLeaseMs() const;
        bool IsFeedForwardLimited() const;
        void SetVelocitySetPoint(int32_t _vel);
        void SetPositionSetPoint(int32_t _pos);
        bool SetPositionSetPointWithTime(int32_t _pos, float _time);
        float GetPosition(bool _isLap = false);
        float GetVelocity();
        float GetFocCurrent();
        void AddTrajectorySetPoint(int32_t _pos, int32_t _vel);
        void SetDisable(bool _disable);
        void SetBrake(bool _brake);
        void ApplyPosAsHomeOffset();
        void ClearStallFlag();
        // Requests are consumed by the 20 kHz loop, never reset a live planner in a CAN ISR.
        void RequestHold();
        void ReportControlOverrun();
        void SetEnable(bool enable);
        uint8_t GetFaultFlags() const;


    private:
        Motor* context;
        int32_t realLapPosition{};
        int32_t realLapPositionLast{};
        int32_t realPosition{};
        int32_t realPositionLast{};
        int32_t estVelocity{};
        int32_t estVelocityIntegral{};
        int32_t estLeadPosition{};
        int32_t estPosition{};
        int32_t estError{};
        int32_t focCurrent{};
        int32_t goalPosition{};
        int32_t goalVelocity{};
        int32_t goalCurrent{};
        bool goalDisable{};
        bool goalBrake{};
        int32_t softPosition{};
        int32_t softVelocity{};
        int32_t softCurrent{};
        bool softDisable{};
        bool softBrake{};
        bool softNewCurve{};
        // Current feed-forward is staged in a seqlock-protected request, then
        // consumed once in the 20 kHz tick. It never changes control mode.
        volatile uint32_t ffRequestWriteSeq{0};
        volatile int32_t ffRequestCurrent_mA{0};
        volatile uint16_t ffRequestSeq{0};
        volatile uint16_t ffRequestLeaseMs{0};
        volatile uint32_t dragRequestWriteSeq{0};
        volatile int32_t dragRequestCurrent_mA{0};
        volatile uint16_t dragRequestSeq{0};
        volatile uint16_t dragRequestLeaseMs{0};
        volatile bool dragRequestEnhanced{false};
        volatile uint16_t dragRequestTotalLimit_mA{0};
        volatile uint16_t dragRequestLeaseFlags{0};
        int32_t feedForwardCurrent_mA{0};
        int32_t feedbackCurrent_mA{0};
        int32_t totalCurrent_mA{0};
        uint16_t lastFeedForwardSequence{0};
        uint16_t lastDragSequence{0};
        uint32_t feedForwardLeaseTicks{0};
        uint32_t dragLeaseTicks{0};
        bool feedForwardActive{false};
        bool dragActive{false};
        bool dragEnhanced{false};
        int32_t dragTotalLimit_mA{0};
        uint16_t dragLeaseFlags{0};
        bool encoderInitialized{false};
        bool localCompOptIn{false};
        int32_t localCompGainQ16{0};
        int32_t localCompAssistCap_mA{0};
        bool accurateVelocityReady{false};
        int32_t accurateVelocityWindowDelta{0};
        uint16_t accurateVelocityWindowTicks{0};
        int32_t accurateVelocity{0};
        int8_t motionDirection{0};
        int8_t motionDirectionCandidate{0};
        uint8_t motionDirectionConfirmWindows{0};
        int32_t localCompRampQ16{0};
        int32_t localCompTargetCurrent_mA{0};
        int32_t localCompensationCurrent_mA{0};
        int16_t harmonicESin_mA{0};
        int16_t harmonicECos_mA{0};
        int16_t harmonicOSin_mA{0};
        int16_t harmonicOCos_mA{0};
        struct LocalCompensationSnapshot_t
        {
            bool enabled;
            uint8_t gainPercent;
            uint16_t sequence;
            uint16_t assistCap_mA;
            int16_t eSin_mA;
            int16_t eCos_mA;
            int16_t oSin_mA;
            int16_t oCos_mA;
        };
        LocalCompensationSnapshot_t localCompSnapshot{};
        volatile uint32_t localCompSnapshotWriteSeq{0};
        volatile uint32_t localCompSnapshotEpoch{0};
        uint32_t appliedLocalCompSnapshotEpoch{0};
        bool feedForwardLimited{false};
        int32_t focPosition{};
        uint32_t stalledTime{};
        uint32_t overloadTime{};
        bool overloadFlag{};
        bool controlOverrunFlag{};
        volatile bool clearFaultsRequested{};
        volatile bool holdRequested{};
        bool holding{};
        bool motionBlocked{};


        void AttachConfig(Config_t* _config);
        bool ReadLocalCompSnapshot(LocalCompensationSnapshot_t& snapshot) const;
        void PublishLocalCompSnapshot(const LocalCompensationSnapshot_t& snapshot);
        void ConsumeLocalCompSnapshot();
        void UpdateAccurateVelocity(int32_t deltaPosition);
        void UpdateMotionDirection();
        int32_t CalculateHarmonicCurrent_mA() const;
        void ResetLocalCompensationDynamics();
        void ClearLocalCompensationOutput();
        int32_t ApplyLeaseLocalCompensation(int32_t current);
        void CalcCurrentToOutput(int32_t current);
        void CalcPidToOutput(int32_t _speed);
        void CalcDceToOutput(int32_t _location, int32_t _speed);
        void ClearIntegral() const;
        void ConsumeFeedForwardRequest();
        void UpdateFeedForwardLease();
        void ReleaseDragLease();
        void RestoreCurrentAcceleration();

        static bool IsNewerSequence(uint16_t sequence, uint16_t last);
        static int32_t CompensateAdvancedAngle(int32_t _vel);
    };


    struct Config_t
    {
        MotionPlanner::Config_t motionParams{};
        Controller::Config_t ctrlParams{};
    };
    Config_t config;

    MotionPlanner motionPlanner;
    Controller* controller = nullptr;
    EncoderBase* encoder = nullptr;
    DriverBase* driver = nullptr;


    void Tick20kHz();
    void AttachEncoder(EncoderBase* _encoder);
    void AttachDriver(DriverBase* _driver);


private:
    Controller controllerInstance = Controller(this);


    void CloseLoopControlTick();
};

#endif
