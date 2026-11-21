#include "common_inc.h"


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 200);
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
PWM pwm(21000, 21000);
// Robot instance
DummyRobot dummy(&hcan1);


/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        dummy.CheckSafety();
        if (dummy.IsEnabled())
        {
            // Send control command to Motors & update Joint states
            switch (dummy.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);
                    //dummy.UpdateJointAngles();
                    dummy.UpdateJointPose6D();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    break;
            }
        } else
        {
            // Just update Joint states
            static uint32_t lastPoll = 0;
            if (HAL_GetTick() - lastPoll >= 20)
            {
                lastPoll = HAL_GetTick();
                if (dummy.IsHolding()) dummy.motorJ[ALL]->Hold();
                dummy.UpdateJointAngles();
            }
            dummy.UpdateJointPose6D();
        }
    }
}


osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}


osThreadId_t oledTaskHandle;
void ThreadOledUpdate(void* argument)
{
    uint32_t t = micros();
    char buf[16];
    char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};

    for (;;)
    {
        mpu6050.Update(true);

        oled.clearBuffer();
        oled.setFont(u8g2_font_5x8_tr);
        // Keep diagnosis visible even when the serial link cannot return text.
        oled.setCursor(0, 7);
        oled.printf("U%u S%lu R%03lu P%03lu T%03lu", unsigned(UsbDeviceState()),
                    usb_stats_.task_running, usb_stats_.cdc_rx_packets % 1000,
                    usb_stats_.cdc_rx_processed % 1000, usb_stats_.cdc_tx_complete % 1000);
        oled.setCursor(0, 14);
        const uint32_t elapsed = micros() - t;
        oled.printf("Q%03lu E%03lu FPS:%lu", usb_stats_.tx_cnt % 1000,
                    usb_stats_.tx_failures % 1000, elapsed ? 1000000 / elapsed : 0);
        t = micros();

        oled.drawBox(0, 15, 128, 3);
        oled.setCursor(0, 30);
        oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
                    (int) roundf(dummy.currentJoints.a[0]), (int) roundf(dummy.currentJoints.a[1]),
                    (int) roundf(dummy.currentJoints.a[2]), (int) roundf(dummy.currentJoints.a[3]),
                    (int) roundf(dummy.currentJoints.a[4]), (int) roundf(dummy.currentJoints.a[5]));

        oled.drawBox(40, 35, 128, 24);
        oled.setFont(u8g2_font_6x12_tr);
        oled.setDrawColor(0);
        oled.setCursor(42, 45);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.X),
                    (int) roundf(dummy.currentPose6D.Y), (int) roundf(dummy.currentPose6D.Z));
        oled.setCursor(42, 56);
        oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.A),
                    (int) roundf(dummy.currentPose6D.B), (int) roundf(dummy.currentPose6D.C));
        oled.setDrawColor(1);
        oled.setCursor(0, 45);
        oled.printf("[XYZ]:");
        oled.setCursor(0, 56);
        oled.printf("[ABC]:");

        oled.setFont(u8g2_font_10x20_tr);
        oled.setCursor(0, 78);
        const uint8_t faultMask = dummy.GetFaultJointMask();
        if (faultMask)
        {
            // Rotate through affected joints; raw flags preserve combined faults.
            unsigned affected[6], count = 0;
            for (unsigned joint = 1; joint <= 6; ++joint)
                if (faultMask & (1U << (joint - 1))) affected[count++] = joint;
            const unsigned joint = affected[(HAL_GetTick() / 1500U) % count];
            const unsigned flags = dummy.GetJointFault(joint);
            const char* reason = flags & 128U ? "CONTROL OVERRUN" :
                                 flags & 8U ? "CAN TIMEOUT" :
                                 flags & 16U ? "OLD F1 PROTOCOL" :
                                 flags & 4U ? "NO CALIBRATION" :
                                 flags & 2U ? "STALL" :
                                 flags & 1U ? "OVERLOAD" :
                                 flags & 64U ? "DRIVER STOP" :
                                 flags & 32U ? "MOVE TIMEOUT" : "CLEARED";
            oled.setFont(u8g2_font_6x12_tr);
            oled.setCursor(0, 68);
            oled.printf("FAULT 0x%02X J%u:%02X", static_cast<unsigned>(faultMask), joint, flags);
            oled.setCursor(0, 80);
            oled.printf("%s", reason);
        } else if (dummy.IsEnabled())
        {
            for (int i = 1; i <= 6; i++)
                buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
            buf[6] = 0;
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], buf);
        } else
        {
            oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");
        }

        oled.sendBuffer();
        osDelay(1);  // Yield so display/IMU refresh cannot monopolise a priority level.
    }
}


/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    dummy.Init();

    // Init IMU.
    //do
    //{
      //  mpu6050.Init();
      //  osDelay(100);
    //} while (!mpu6050.testConnection());
    //mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    oled.Init();
    pwm.Start();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);

    const osThreadAttr_t oledTask_attributes = {
        .name = "OledTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    };
    oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    // System started, light switch-led up.
    Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    pwm.SetDuty(PWM::CH_A1, 0.5);
}
