#ifndef __HAL_DEF_H
#define __HAL_DEF_H

#include <stdint.h>

namespace HAL
{
    /* Joints Angle */
    typedef struct
    {
        float joint1;
        float joint2;
        float joint3;
        float joint4;
        float joint5;
        float joint6;
    } Joints_Info_t;

    /* Pose6D */
    typedef struct
    {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
    } Pose6D_Info_t;
}

#endif
