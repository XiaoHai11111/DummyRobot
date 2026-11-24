#pragma once

#include <cmath>
#include <cstdint>

namespace GravityCompensation
{
    class Model
    {
    public:
        struct Vec3
        {
            float x{}, y{}, z{};
        };

        struct Mat3
        {
            float a[3][3]{};
        };

        struct JointGeometry
        {
            Vec3 origin;
            Vec3 rpy;
            Vec3 com;
            float massKg{};
        };

        struct FrictionModel
        {
            float coulombA{};
            float viscousAPerDegS{};
        };

        struct Config_t
        {
            JointGeometry joints[6];
            float modelOffsetDeg[6]{};
            float currentPerNm[6]{};
            float currentBiasA[6]{};
            FrictionModel friction[6];
            float gravityMS2{9.81f};

            Config_t()
            {
                const JointGeometry geometry[6] = {
                    {{0, 0, 0.0865f}, {0, 0, 0},
                     {0.00565647f, 0.00561557f, 0.01103681f}, 0.1407715f},
                    {{0.035f, 0, 0.0275f}, {-1.5708f, 0, 0},
                     {0.00158183f, -0.07501807f, -0.00456731f}, 1.43844277f},
                    {{0, -0.146f, 0}, {0, 0, -1.5708f},
                     {0.05703139f, -0.01762985f, -0.00275017f}, 0.30020083f},
                    {{0.052f, -0.0115f, 0}, {1.5708f, 0, -3.1416f},
                     {-0.00006733f, 0.00046890f, 0.07837243f}, 0.28251937f},
                    {{0, 0, 0.134f}, {1.5708f, -1.5708f, 3.1416f},
                     {0.043953f, 0, -0.002344f}, 0.11168657f},
                    {{0.13725f, 0, 0}, {1.5708f, 0, 1.5708f},
                     {0.00112758f, -0.00037575f, -0.04507409f}, 0.02694263f}
                };
                for (int i = 0; i < 6; ++i) joints[i] = geometry[i];

                modelOffsetDeg[0] = 0;
                modelOffsetDeg[1] = 0;
                modelOffsetDeg[2] = -90; // F4 robot J3 angle -> URDF/MJCF angle.
                modelOffsetDeg[3] = 0;
                modelOffsetDeg[4] = 0;
                modelOffsetDeg[5] = 0;

                // Joint-positive current per generalized gravity torque (A/Nm),
                // calibrated against the four recorded fixed-pose J2/J3 runs.
                currentPerNm[0] = 0;
                currentPerNm[1] = -0.203f;
                currentPerNm[2] = -0.334f;
                currentPerNm[3] = 0;
                currentPerNm[4] = 0;
                currentPerNm[5] = 0;

                currentBiasA[0] = 0;
                currentBiasA[1] = -0.011f;
                currentBiasA[2] = -0.019f;
                currentBiasA[3] = 0;
                currentBiasA[4] = 0;
                currentBiasA[5] = 0;

                const FrictionModel losses[6] = {
                    {0.0771f, 0.00336f},
                    {0.1252f, 0.00578f},
                    {0.1018f, 0.00156f},
                    {0.2499f, 0.00447f},
                    {0.1847f, 0.00366f},
                    {0.1668f, 0.0f}
                };
                for (int i = 0; i < 6; ++i) friction[i] = losses[i];
            }
        } config;

        static Vec3 Add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
        static Vec3 Sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
        static Vec3 Scale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
        static float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
        static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return {a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x};
        }

        static Mat3 Multiply(Mat3 a, Mat3 b)
        {
            Mat3 out;
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    out.a[row][col] = a.a[row][0] * b.a[0][col] +
                                      a.a[row][1] * b.a[1][col] +
                                      a.a[row][2] * b.a[2][col];
            return out;
        }

        static Vec3 Transform(Mat3 r, Vec3 v)
        {
            return {r.a[0][0] * v.x + r.a[0][1] * v.y + r.a[0][2] * v.z,
                    r.a[1][0] * v.x + r.a[1][1] * v.y + r.a[1][2] * v.z,
                    r.a[2][0] * v.x + r.a[2][1] * v.y + r.a[2][2] * v.z};
        }

        static Mat3 RotationRpy(Vec3 rpy)
        {
            const float cr = cosf(rpy.x), sr = sinf(rpy.x);
            const float cp = cosf(rpy.y), sp = sinf(rpy.y);
            const float cy = cosf(rpy.z), sy = sinf(rpy.z);
            Mat3 rx{{{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}}};
            Mat3 ry{{{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}}};
            Mat3 rz{{{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}}};
            return Multiply(Multiply(rz, ry), rx);
        }

        static Mat3 RotationZ(float radians)
        {
            const float c = cosf(radians), s = sinf(radians);
            return Mat3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
        }

        static float Clamp(float value, float limit)
        {
            if (!std::isfinite(value)) return 0;
            if (value > limit) return limit;
            if (value < -limit) return -limit;
            return value;
        }

        void CalculateGravityCurrentA(const float q[6], float currentA[7]) const
        {
            for (int i = 0; i < 7; ++i) currentA[i] = 0;

            Vec3 position{0, 0, 0.015f};
            Mat3 rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
            Vec3 jointOrigin[7]{};
            Vec3 jointAxis[7]{};
            Vec3 linkCom[7]{};

            constexpr float degToRad = 0.017453292519943295f;
            for (int j = 1; j <= 6; ++j)
            {
                const JointGeometry& geometry = config.joints[j - 1];
                const Mat3 fixedRotation = RotationRpy(geometry.rpy);
                jointOrigin[j] = Add(position, Transform(rotation, geometry.origin));
                jointAxis[j] = Transform(rotation, Transform(fixedRotation, Vec3{0, 0, 1}));
                const float angle = (q[j - 1] + config.modelOffsetDeg[j - 1]) * degToRad;
                rotation = Multiply(Multiply(rotation, fixedRotation), RotationZ(angle));
                position = jointOrigin[j];
                linkCom[j] = Add(position, Transform(rotation, geometry.com));
            }

            for (int joint = 1; joint <= 6; ++joint)
            {
                if (config.currentPerNm[joint - 1] == 0) continue;
                float torque = 0;
                for (int link = joint; link <= 6; ++link)
                {
                    const Vec3 arm = Sub(linkCom[link], jointOrigin[joint]);
                    const Vec3 force = {0, 0, -config.joints[link - 1].massKg * config.gravityMS2};
                    torque += Dot(force, Cross(jointAxis[joint], arm));
                }
                const float value = config.currentPerNm[joint - 1] * torque +
                                    config.currentBiasA[joint - 1];
                currentA[joint] = std::isfinite(value) ? value : 0;
            }
        }

        float GravityJointA(uint8_t joint, const float q[6]) const
        {
            if (joint < 1 || joint > 6) return 0;
            for (int i = 0; i < 6; ++i)
                if (!std::isfinite(q[i])) return 0;
            float currents[7]{};
            CalculateGravityCurrentA(q, currents);
            return currents[joint];
        }

        float FrictionLossA(uint8_t joint, float velocityDegS) const
        {
            if (joint < 1 || joint > 6 || !std::isfinite(velocityDegS)) return 0;
            const FrictionModel& model = config.friction[joint - 1];
            return model.coulombA + model.viscousAPerDegS * fabsf(velocityDegS);
        }
    };
}
