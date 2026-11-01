#include "HAL/HAL.h"
#include "HAL/HAL_Def.h"
#include "Configs/Version.h"

extern HAL::Joints_Info_t joints_info;
extern HAL::Pose6D_Info_t pose6d_info;
TaskHandle_t getdummystatusHandle;
TaskHandle_t serial1updateHandle;
void getDummyStatus_Task(void *pvParameters) 
{
  for (;;)
  {
    HAL::Serial1_GetDummyStatus();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void serial1Update_Task(void *pvParameters) 
{
  for (;;)
  {
    HAL::Serial1_Update();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void HAL::Serial_Init()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial.println(VERSION_FIRMWARE_NAME);
    Serial.println("Version: " VERSION_SOFTWARE);
    Serial.println("Author: " VERSION_AUTHOR_NAME);
    xTaskCreatePinnedToCore(
        getDummyStatus_Task, 
        "getDummyStatus", 
        4096, 
        NULL, 
        1, 
        &getdummystatusHandle,
        1
    );
    xTaskCreatePinnedToCore(
        serial1Update_Task, 
        "serial1update", 
        4096, 
        NULL, 
        1, 
        &serial1updateHandle,
        1
    );
}

void HAL::Serial_Update()
{
    if (Serial.available())
    {
        //HAL::Joints_Info_t joints_info;
        uint8_t argNum;
        String rev;
        String rev1;
        rev = Serial.readStringUntil('\n');
        
        // 确保rev不为空，并且以 '&' 开头
        if (rev.length() > 0 && rev[0] == '&')
        {
            // 使用c_str()获取const char*类型的指针
            argNum = sscanf(rev.c_str(), "&%f,%f,%f,%f,%f,%f", &joints_info.joint1, &joints_info.joint2, &joints_info.joint3,
                            &joints_info.joint4, &joints_info.joint5, &joints_info.joint6);
            if (argNum == 6)
            {
                Serial.printf("Joint_angle:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",joints_info.joint1, joints_info.joint2, joints_info.joint3,
                                joints_info.joint4, joints_info.joint5, joints_info.joint6); 
                Serial1.println(rev.c_str());   
            }
        }else if(rev.length() > 0 && rev[0] == '#')
        {
            Serial1.println(rev.c_str());
            Serial.println(rev.c_str());
        }else if (rev.length() > 0 && rev[0] == '!')
        {
            Serial1.println(rev.c_str());
            Serial.println(rev.c_str());
        }else if (rev.length() > 0 && rev[0] == '@')
        {
            Serial.println(rev.c_str());
            Serial.println(HAL::BT_IsConnect());
        }
    }
}

void HAL::Serial1_Update()
{
    if (Serial1.available())
    {
        String rev1;
        rev1 = Serial1.readStringUntil('\n');
        if (rev1.length() > 0)
        {
            Serial.println(rev1.c_str());
            if (rev1[2] == 'j')
            {
                sscanf(rev1.c_str(), "okj %f %f %f %f %f %f", &joints_info.joint1, &joints_info.joint2, &joints_info.joint3,
                    &joints_info.joint4, &joints_info.joint5, &joints_info.joint6);
            }
            else if (rev1[2] == 'l')
            {
                sscanf(rev1.c_str(), "okl %f %f %f %f %f %f", &pose6d_info.x, &pose6d_info.y, &pose6d_info.z,
                    &pose6d_info.roll, &pose6d_info.pitch, &pose6d_info.yaw);
            }
                        
        } 
    } 
}

void HAL::Serial1_GetDummyStatus()
{
    if (HAL::BT_IsConnect())
    {
        Serial1.println("#GETJPOS");
        Serial1.println("#GETLPOS");
    }  
}