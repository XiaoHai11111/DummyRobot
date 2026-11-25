#pragma once

#include <cstdint>

namespace GravityCompensation
{
    class Model
    {
    public:
        struct Vec3
        {
            float x;
            float y;
            float z;
        };

        struct Mat3
        {
            float a[3][3];
        };

        struct JointGeometry
        {
            Vec3 origin;
            Vec3 rpy;
            Vec3 com;
            float massKg;
        };

        struct FrictionModel
        {
            float coulombA;
            float viscousAPerDegS;
        };

        struct Config_t
        {
            JointGeometry joints[6];
            float modelOffsetDeg[6];
            float currentPerNm[6];
            float currentBiasA[6];
            float joint5SinCoeffA;
            float joint5CosCoeffA;
            float joint5BiasA;
            float joint5MaxCurrentA;
            FrictionModel friction[6];
            float gravityMS2;

            Config_t();
        } config;

        static Vec3 Add(Vec3 a, Vec3 b);
        static Vec3 Sub(Vec3 a, Vec3 b);
        static Vec3 Scale(Vec3 a, float s);
        static float Dot(Vec3 a, Vec3 b);
        static Vec3 Cross(Vec3 a, Vec3 b);
        static Mat3 Multiply(Mat3 a, Mat3 b);
        static Vec3 Transform(Mat3 r, Vec3 v);
        static Mat3 RotationRpy(Vec3 rpy);
        static Mat3 RotationZ(float radians);
        static float Clamp(float value, float limit);

        void CalculateGravityCurrentA(const float q[6], float currentA[7]) const;
        float GravityJointA(uint8_t joint, const float q[6]) const;
        float FrictionLossA(uint8_t joint, float velocityDegS) const;
    };
}
