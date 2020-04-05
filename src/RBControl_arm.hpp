#pragma once

#include <functional>
#include <math.h>
#include <memory>
#include <stdint.h>
#include <vector>

#include "RBControl_angle.hpp"

namespace rb {

class ArmBuilder;
class Bone;

class Arm {
    friend class ArmBuilder;
    friend class Bone;

public:
    typedef int32_t CoordType;

    struct BoneDefinition {
        BoneDefinition(uint8_t servo_id, CoordType length) {
            this->servo_id = servo_id;
            this->length = length;
            rel_min = abs_min = base_rel_min = -Angle::Pi;
            rel_max = abs_max = base_rel_max = Angle::Pi;
            calcServoAng = [](Angle, Angle rel) -> Angle { return rel; };
            calcAbsAng = [](Angle servoAng) -> Angle { return servoAng; };
        }

        uint8_t servo_id;
        CoordType length;
        Angle rel_min, rel_max;
        Angle abs_min, abs_max;
        Angle base_rel_min, base_rel_max;

        std::function<Angle(Angle, Angle)> calcServoAng;
        std::function<Angle(Angle)> calcAbsAng;
    };

    struct Definition {
        Definition() {
            body_height = 0;
            body_radius = 0;

            arm_offset_x = 0;
            arm_offset_y = 0;
        }

        CoordType body_height;
        CoordType body_radius;

        CoordType arm_offset_x;
        CoordType arm_offset_y;

        std::vector<BoneDefinition> bones;
    };

    static Angle clamp(Angle ang);

    ~Arm();

    bool solve(Arm::CoordType target_x, Arm::CoordType target_y);
    void setServos(float speed = 180.f);

    const Definition& definition() const { return m_def; }
    const std::vector<Bone>& bones() const { return m_bones; }

    bool syncBonesWithServos();

private:
    typedef float AngleType;

    static AngleType clamp(AngleType ang);

    Arm(const Definition& def);
    Arm(const Arm&) = delete;

    template <typename T = CoordType>
    static T roundCoord(AngleType val);

    bool solveIteration(CoordType target_x, CoordType target_y, bool& modified);
    AngleType rotateArm(size_t idx, AngleType rot_ang);
    void fixBodyCollision();
    bool isInBody(CoordType x, CoordType y) const;
    void updateBones();

    const Definition m_def;
    std::vector<Bone> m_bones;
};

class Bone {
    friend class Arm;

public:
    const Arm::BoneDefinition& def;

    Angle absAngle, relAngle;
    Arm::CoordType x, y;

    Angle servoAng() const { return def.calcServoAng(absAngle, relAngle); }

private:
    Bone(const Arm::BoneDefinition& def);

    void updatePos(Bone* prev);
};

class ArmBuilder;

class BoneBuilder {
    friend class ArmBuilder;

public:
    BoneBuilder(BoneBuilder&& other);
    ~BoneBuilder();

    BoneBuilder& relStops(Angle min, Angle max);
    BoneBuilder& absStops(Angle min, Angle max);
    BoneBuilder& baseRelStops(Angle min, Angle max);
    BoneBuilder& calcServoAng(std::function<Angle(Angle abs, Angle rel)> func);
    BoneBuilder& calcAbsAng(std::function<Angle(Angle servoAng)> func);

private:
    BoneBuilder(std::shared_ptr<Arm::BoneDefinition> def);

    std::shared_ptr<Arm::BoneDefinition> m_def;
};

class ArmBuilder {
public:
    ArmBuilder();
    ~ArmBuilder();

    ArmBuilder& body(Arm::CoordType height_mm, Arm::CoordType radius_mm);
    ArmBuilder& armOffset(Arm::CoordType x_mm, Arm::CoordType y_mm);
    BoneBuilder bone(uint8_t servo_id, Arm::CoordType length_mm);

    std::unique_ptr<Arm> build();

private:
    Arm::Definition m_def;
    std::vector<std::shared_ptr<Arm::BoneDefinition>> m_bones;
};

}; // namespace rb
