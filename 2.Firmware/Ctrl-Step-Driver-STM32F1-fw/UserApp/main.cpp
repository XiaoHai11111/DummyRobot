#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>
#include "Motor/control_timing.h"


/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 1000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;
ControlTimingStats controlTimingStats;
static uint32_t controlBudgetCycles;


/* Main Entry ----------------------------------------------------------------*/
void Main()
{
    uint64_t serialNum = GetSerialNumber();
    uint16_t defaultNodeID = 6;

    /*---------- Apply EEPROM Settings ----------*/
    // Setting priority is EEPROM > Motor.h
    EEPROM eeprom;
    eeprom.get(0, boardConfig);
    if (boardConfig.configStatus != CONFIG_OK) // use default settings
    {
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
            .canNodeId = defaultNodeID,
            .encoderHomeOffset = 0,
            .defaultMode = Motor::MODE_COMMAND_POSITION,
            .currentLimit = 3 * 1000,    // mA
            .velocityLimit = 60 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // steps/s
            .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,  // steps/s^2
            .calibrationCurrent = 2000,  // mA
            .dce_kp = 1000,
            .dce_kv = 80,
            .dce_ki = 200,
            .dce_kd = 250,
            .enableMotorOnBoot = false,
            .enableStallProtect = false,
            // Stage 2 local compensation defaults. Fitted candidates are
            // documented in dummy_robot.h and configured per board by F4.
            .localDamping_mA_per_motorRevS = 0,
            .localCoulomb_mA = 0,
            .localViscous_mA_per_motorRevS = 0,
            .localDeadzone_motorRevS_x1000 =
                Motor::Controller::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000,
            .localCompLimit_mA = 0,
            .harmonicESin_mA = 0,
            .harmonicECos_mA = 0,
            .harmonicOSin_mA = 0,
            .harmonicOCos_mA = 0,
            .version = BOARD_CONFIG_VERSION
        };
        eeprom.put(0, boardConfig);
    } else if (boardConfig.version != BOARD_CONFIG_VERSION)
    {
        // Preserve values that already exist in the stored version.
        if (boardConfig.version < 2U)
        {
            boardConfig.localDamping_mA_per_motorRevS = 0;
            boardConfig.localCoulomb_mA = 0;
            boardConfig.localViscous_mA_per_motorRevS = 0;
            boardConfig.localDeadzone_motorRevS_x1000 =
                Motor::Controller::LOCAL_COMP_MIN_DEADZONE_motorRevS_x1000;
            boardConfig.localCompLimit_mA = 0;
        }
        boardConfig.harmonicESin_mA = 0;
        boardConfig.harmonicECos_mA = 0;
        boardConfig.harmonicOSin_mA = 0;
        boardConfig.harmonicOCos_mA = 0;
        boardConfig.version = BOARD_CONFIG_VERSION;
        eeprom.put(0, boardConfig);
    }
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    motor.config.ctrlParams.feedForward.maxCurrent = boardConfig.currentLimit;
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;
    motor.controller->SetLocalCompensationParam(0, boardConfig.localDamping_mA_per_motorRevS);
    motor.controller->SetLocalCompensationParam(1, boardConfig.localCoulomb_mA);
    motor.controller->SetLocalCompensationParam(2, boardConfig.localViscous_mA_per_motorRevS);
    motor.controller->SetLocalCompensationParam(3, boardConfig.localDeadzone_motorRevS_x1000);
    motor.controller->SetLocalCompensationParam(4, boardConfig.localCompLimit_mA);
    motor.controller->SetMotorDomainHarmonics(
        boardConfig.harmonicESin_mA, boardConfig.harmonicECos_mA,
        boardConfig.harmonicOSin_mA, boardConfig.harmonicOCos_mA);


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);
    motor.AttachEncoder(&mt6816);
    motor.controller->Init();
    motor.driver->Init();
    motor.encoder->Init();


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);


    /*------- Start Close-Loop Control Tick ------*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // Reserve 10% of one control period for wrapper/IRQ overhead and CAN service.
    controlBudgetCycles = (SystemCoreClock / static_cast<uint32_t>(motor.motionPlanner.CONTROL_FREQUENCY)) * 9U / 10U;
    HAL_Delay(100);
    HAL_TIM_Base_Start_IT(&htim1);  // 100Hz
    HAL_TIM_Base_Start_IT(&htim4);  // 20kHz

    if (button1.IsPressed() && button2.IsPressed())
        encoderCalibrator.isTriggered = true;


    for (;;)
    {
        encoderCalibrator.TickMainLoop();


        if (boardConfig.configStatus == CONFIG_COMMIT)
        {
            boardConfig.configStatus = CONFIG_OK;
            eeprom.put(0, boardConfig);
        } else if (boardConfig.configStatus == CONFIG_RESTORE)
        {
            eeprom.put(0, boardConfig);
            HAL_NVIC_SystemReset();
        }
    }
}


/* Event Callbacks -----------------------------------------------------------*/
extern "C" void Tim1Callback100Hz()
{
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    button1.Tick(10);
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);
}


extern "C" void Tim4Callback20kHz()
{
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();
    else
    {
        const uint32_t started = DWT->CYCCNT;
        motor.Tick20kHz();
        if (controlTimingStats.Observe(DWT->CYCCNT - started, controlBudgetCycles))
            motor.controller->ReportControlOverrun();
    }
}


void OnButton1Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            HAL_NVIC_SystemReset();
            break;
        case ButtonBase::CLICK:
            if (motor.controller->modeRunning != Motor::MODE_STOP)
            {
                boardConfig.defaultMode = motor.controller->modeRunning;
                motor.controller->SetEnable(false);
            } else
            {
                motor.controller->SetEnable(true);
            }
            break;
    }
}


void OnButton2Event(Button::Event _event)
{
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            switch (motor.controller->modeRunning)
            {
                case Motor::MODE_COMMAND_CURRENT:
                case Motor::MODE_PWM_CURRENT:
                    motor.controller->SetCurrentSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_VELOCITY:
                case Motor::MODE_PWM_VELOCITY:
                    motor.controller->SetVelocitySetPoint(0);
                    break;
                case Motor::MODE_COMMAND_POSITION:
                case Motor::MODE_PWM_POSITION:
                    motor.controller->SetPositionSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_Trajectory:
                case Motor::MODE_STEP_DIR:
                case Motor::MODE_STOP:
                    break;
            }
            break;
        case ButtonBase::CLICK:
            motor.controller->ClearStallFlag();
            break;
    }
}
