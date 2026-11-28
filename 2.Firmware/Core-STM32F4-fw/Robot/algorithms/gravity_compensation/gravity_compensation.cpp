#include "gravity_compensation.h"

#include <cmath>


namespace GravityCompensation
{
    Model::Config_t::Config_t()
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

        // Joint-positive current per generalized gravity torque (A/Nm).
        // Every joint has the same model shape; unused terms are zero.
        gravity[0] = {0.0f, 0.0f, 3.3f, 0.0f, 0.0f, 1.0f};
        gravity[1] = {-0.203f, -0.011f, 3.3f, 0.0f, 0.0f, 1.0f};
        gravity[2] = {-0.334f, -0.019f, 3.3f, 0.0f, 0.0f, 1.0f};
        gravity[3] = {0.901902f, 0.006205f, 0.120f, 0.0f, 0.0f, 1.0f};
        gravity[4] = {1.004686f, -0.003045f, 0.180f, 0.0f, 0.0f, 1.0f};
        gravity[5] = {0.0f, 0.0f, 3.3f, 0.0f, 0.0f, 1.0f};

        const FrictionModel losses[6] = {
            {0.0771f, 0.00336f},
            {0.1252f, 0.00578f},
            {0.1018f, 0.00156f},
            {0.2499f, 0.00447f},
            {0.1847f, 0.00366f},
            {0.1668f, 0.0f}
        };
        for (int i = 0; i < 6; ++i) friction[i] = losses[i];

        gravityMS2 = 9.81f;
    }


    Model::Vec3 Model::Add(Vec3 a, Vec3 b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }


    Model::Vec3 Model::Sub(Vec3 a, Vec3 b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }


    Model::Vec3 Model::Scale(Vec3 a, float s)
    {
        return {a.x * s, a.y * s, a.z * s};
    }


    float Model::Dot(Vec3 a, Vec3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }


    Model::Vec3 Model::Cross(Vec3 a, Vec3 b)
    {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }


    Model::Mat3 Model::Multiply(Mat3 a, Mat3 b)
    {
        Mat3 out{};
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                out.a[row][col] = a.a[row][0] * b.a[0][col] +
                                  a.a[row][1] * b.a[1][col] +
                                  a.a[row][2] * b.a[2][col];
        return out;
    }


    Model::Vec3 Model::Transform(Mat3 r, Vec3 v)
    {
        return {r.a[0][0] * v.x + r.a[0][1] * v.y + r.a[0][2] * v.z,
                r.a[1][0] * v.x + r.a[1][1] * v.y + r.a[1][2] * v.z,
                r.a[2][0] * v.x + r.a[2][1] * v.y + r.a[2][2] * v.z};
    }


    Model::Mat3 Model::RotationRpy(Vec3 rpy)
    {
        const float cr = cosf(rpy.x), sr = sinf(rpy.x);
        const float cp = cosf(rpy.y), sp = sinf(rpy.y);
        const float cy = cosf(rpy.z), sy = sinf(rpy.z);
        Mat3 rx{{{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}}};
        Mat3 ry{{{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}}};
        Mat3 rz{{{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}}};
        return Multiply(Multiply(rz, ry), rx);
    }


    Model::Mat3 Model::RotationZ(float radians)
    {
        const float c = cosf(radians), s = sinf(radians);
        return Mat3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
    }


    float Model::Clamp(float value, float limit)
    {
        if (!std::isfinite(value)) return 0;
        if (value > limit) return limit;
        if (value < -limit) return -limit;
        return value;
    }


    void Model::CalculateGravityTorqueNm(const float q[6], float torqueNm[7]) const
    {
        for (int i = 0; i < 7; ++i) torqueNm[i] = 0;

        JointGeometry geometry[6];
        for (int i = 0; i < 6; ++i) geometry[i] = config.joints[i];
        // Current dummy.xml wrist3 body plus both closed fingers
        // (0.0300368 kg each), represented by their combined mass and COM.
        geometry[5] = {
            {0.14425f, 0, 0}, {1.5708f, 0, 1.5708f},
            {0.0009117143f, -0.0000399355f, -0.043471535f}, 0.0870162f
        };

        Vec3 position{0, 0, 0};
        Mat3 rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        Vec3 jointOrigin[7]{};
        Vec3 jointAxis[7]{};
        Vec3 linkCom[7]{};

        constexpr float degToRad = 0.017453292519943295f;
        for (int j = 1; j <= 6; ++j)
        {
            const JointGeometry& joint = geometry[j - 1];
            const Mat3 fixedRotation = RotationRpy(joint.rpy);
            jointOrigin[j] = Add(position, Transform(rotation, joint.origin));
            jointAxis[j] = Transform(rotation, Transform(fixedRotation, Vec3{0, 0, 1}));
            const float angle = (q[j - 1] + config.modelOffsetDeg[j - 1]) * degToRad;
            rotation = Multiply(Multiply(rotation, fixedRotation), RotationZ(angle));
            position = jointOrigin[j];
            linkCom[j] = Add(position, Transform(rotation, joint.com));
        }

        for (int joint = 1; joint <= 6; ++joint)
        {
            float torque = 0;
            for (int link = joint; link <= 6; ++link)
            {
                const Vec3 arm = Sub(linkCom[link], jointOrigin[joint]);
                const Vec3 force = {0, 0, -geometry[link - 1].massKg * config.gravityMS2};
                torque += Dot(force, Cross(jointAxis[joint], arm));
            }
            // qfrc_bias is the force generalized over joint-positive motion.
            torqueNm[joint] = std::isfinite(torque) ? -torque : 0;
        }
    }


    void Model::CalculateGravityCurrentA(const float q[6], float currentA[7]) const
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
            const JointGeometry& joint = config.joints[j - 1];
            const Mat3 fixedRotation = RotationRpy(joint.rpy);
            jointOrigin[j] = Add(position, Transform(rotation, joint.origin));
            jointAxis[j] = Transform(rotation, Transform(fixedRotation, Vec3{0, 0, 1}));
            const float angle = (q[j - 1] + config.modelOffsetDeg[j - 1]) * degToRad;
            rotation = Multiply(Multiply(rotation, fixedRotation), RotationZ(angle));
            position = jointOrigin[j];
            linkCom[j] = Add(position, Transform(rotation, joint.com));
        }

        for (int joint = 1; joint <= 6; ++joint)
        {
            const JointGravityModel& model = config.gravity[joint - 1];
            float jointCurrentA = 0;
            if (model.currentPerNm != 0)
            {
            float torque = 0;
            for (int link = joint; link <= 6; ++link)
            {
                const Vec3 arm = Sub(linkCom[link], jointOrigin[joint]);
                const Vec3 force = {
                    0, 0, -config.joints[link - 1].massKg * config.gravityMS2
                };
                torque += Dot(force, Cross(jointAxis[joint], arm));
            }
                const float value = model.currentPerNm * torque + model.biasA;
                jointCurrentA = std::isfinite(value) ? value : 0;
            }

            // Joint-angle harmonic terms are a normal per-joint gravity
            // coefficient, not a special J5 code path.
            if (model.sinCoeffA != 0 || model.cosCoeffA != 0)
            {
                const float radians =
                    (q[joint - 1] + config.modelOffsetDeg[joint - 1]) * degToRad;
                const float harmonic = model.sinCoeffA * sinf(radians) +
                                       model.cosCoeffA * cosf(radians);
                jointCurrentA = std::isfinite(harmonic) ? jointCurrentA + harmonic : 0;
            }
            currentA[joint] = Clamp(jointCurrentA * model.scale, model.limitA);
        }
    }


    float Model::CalculateCalibratedJointCurrentA(uint8_t joint, const float q[6]) const
    {
        if (joint < 1 || joint > 6) return 0;
        for (unsigned i = 0; i < 6; ++i)
            if (!std::isfinite(q[i])) return 0;
        float currents[7]{};
        CalculateGravityCurrentA(q, currents);
        return currents[joint];
    }


    float Model::GravityJointA(uint8_t joint, const float q[6]) const
    {
        if (joint < 1 || joint > 6) return 0;
        for (int i = 0; i < 6; ++i)
            if (!std::isfinite(q[i])) return 0;
        float currents[7]{};
        CalculateGravityCurrentA(q, currents);
        return currents[joint];
    }


    float Model::FrictionLossA(uint8_t joint, float velocityDegS) const
    {
        if (joint < 1 || joint > 6 || !std::isfinite(velocityDegS)) return 0;
        const FrictionModel& model = config.friction[joint - 1];
        return model.coulombA + model.viscousAPerDegS * fabsf(velocityDegS);
    }


    const Model::JointGravityModel& Model::GetJointGravityModel(uint8_t joint) const
    {
        static const JointGravityModel invalid{};
        return joint >= 1 && joint <= 6 ? config.gravity[joint - 1] : invalid;
    }


    bool Model::SetJointGravityModel(uint8_t joint, float currentPerNm,
                                     float biasA, float limitA)
    {
        if (joint < 1 || joint > 6 || !std::isfinite(currentPerNm) ||
            !std::isfinite(biasA) || !std::isfinite(limitA) ||
            fabsf(currentPerNm) > 10 ||
            biasA < -1 || biasA > 1 || limitA <= 0 || limitA > 3.3f)
            return false;
        config.gravity[joint - 1].currentPerNm = currentPerNm;
        config.gravity[joint - 1].biasA = biasA;
        config.gravity[joint - 1].limitA = limitA;
        return true;
    }


    bool Model::SetJointGravityHarmonics(uint8_t joint, float sinCoeffA,
                                         float cosCoeffA)
    {
        if (joint < 1 || joint > 6 || !std::isfinite(sinCoeffA) ||
            !std::isfinite(cosCoeffA) || fabsf(sinCoeffA) > 1 ||
            fabsf(cosCoeffA) > 1)
            return false;
        config.gravity[joint - 1].sinCoeffA = sinCoeffA;
        config.gravity[joint - 1].cosCoeffA = cosCoeffA;
        return true;
    }


    bool Model::SetJointGravityScale(uint8_t joint, float scale)
    {
        if (joint < 1 || joint > 6 || !std::isfinite(scale) ||
            scale < 0 || scale > 1)
            return false;
        config.gravity[joint - 1].scale = scale;
        return true;
    }


    void Model::ResetJointGravityModel(uint8_t joint)
    {
        if (joint < 1 || joint > 6) return;
        const Model defaults;
        config.gravity[joint - 1] = defaults.config.gravity[joint - 1];
    }
}
