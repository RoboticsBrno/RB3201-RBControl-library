#include "RBControl_arm.hpp"
#include "RBControl_manager.hpp"

#include <math.h>
#include <stdlib.h>

namespace rb {

template <typename T>
T Arm::roundCoord(Arm::AngleType val) {
    return T(round(val));
}

template <>
float Arm::roundCoord(Arm::AngleType val) { return float(val); }
template <>
double Arm::roundCoord(Arm::AngleType val) { return double(val); }

ArmBuilder::ArmBuilder() {
}

ArmBuilder::~ArmBuilder() {
}

ArmBuilder& ArmBuilder::body(Arm::CoordType height_mm, Arm::CoordType radius_mm) {
    m_def.body_height = height_mm;
    m_def.body_radius = radius_mm;
    return *this;
}

ArmBuilder& ArmBuilder::armOffset(Arm::CoordType x_mm, Arm::CoordType y_mm) {
    m_def.arm_offset_x = x_mm;
    m_def.arm_offset_y = y_mm;
    return *this;
}

BoneBuilder ArmBuilder::bone(uint8_t servo_id, Arm::CoordType length_mm) {
    std::shared_ptr<Arm::BoneDefinition> bone(new Arm::BoneDefinition(servo_id, length_mm));
    m_bones.push_back(bone);
    return BoneBuilder(bone);
}

std::unique_ptr<Arm> ArmBuilder::build() {
    m_def.bones.reserve(m_bones.size());
    for (auto bone : m_bones) {
        m_def.bones.push_back(*bone);
    }
    m_bones.clear();
    return std::unique_ptr<Arm>(new Arm(m_def));
}

BoneBuilder::BoneBuilder(std::shared_ptr<Arm::BoneDefinition> bone)
    : m_def(bone) {
}

BoneBuilder::BoneBuilder(BoneBuilder&& other) {
    m_def = std::move(other.m_def);
}

BoneBuilder::~BoneBuilder() {
}

BoneBuilder& BoneBuilder::relStops(Angle min_rad, Angle max_rad) {
    m_def->rel_min = min_rad;
    m_def->rel_max = max_rad;
    return *this;
}

BoneBuilder& BoneBuilder::absStops(Angle min_rad, Angle max_rad) {
    m_def->abs_min = min_rad;
    m_def->abs_max = max_rad;
    return *this;
}

BoneBuilder& BoneBuilder::baseRelStops(Angle min_rad, Angle max_rad) {
    m_def->base_rel_min = min_rad;
    m_def->base_rel_max = max_rad;
    return *this;
}

BoneBuilder& BoneBuilder::calcServoAng(std::function<Angle(Angle abs, Angle rel)> func) {
    m_def->calcServoAng = func;
    return *this;
}

BoneBuilder& BoneBuilder::calcAbsAng(std::function<Angle(Angle servoAng)> func) {
    m_def->calcAbsAng = func;
    return *this;
}

void Bone::updatePos(Bone* prev) {
    if (prev != nullptr) {
        absAngle = Arm::clamp(prev->absAngle + relAngle);
    } else {
        absAngle = relAngle;
    }

    x = Arm::roundCoord(Arm::AngleType(cos(absAngle.rad()) * def.length));
    y = Arm::roundCoord(Arm::AngleType(sin(absAngle.rad()) * def.length));
    if (prev != nullptr) {
        x += prev->x;
        y += prev->y;
    }
}

Arm::AngleType Arm::clamp(Arm::AngleType val) {
    static constexpr auto pi = Arm::AngleType(M_PI);
    val = fmod(val, pi * 2);
    if (val < -pi)
        val += pi * 2;
    else if (val > pi)
        val -= pi * 2;
    return val;
}

Angle Arm::clamp(Angle ang) {
    return Angle::rad(Angle::_T(clamp(Arm::AngleType(ang.rad()))));
}

Arm::Arm(const Arm::Definition& def)
    : m_def(def) {
    m_bones.reserve(m_def.bones.size());
    for (const auto& def : m_def.bones) {
        m_bones.push_back(Bone(def));
    }
}

Arm::~Arm() {
}

void Arm::setServos(float speed) {
    auto& servos = Manager::get().servoBus();
    for (const auto& b : m_bones) {
        servos.set(b.def.servo_id, b.servoAng(), speed);
    }
}

bool Arm::syncBonesWithServos() {
    auto& servos = Manager::get().servoBus();

    Bone* prev = nullptr;
    for (size_t i = 0; i < m_bones.size(); ++i) {
        auto* bone = &m_bones[i];
        auto pos = servos.posOffline(bone->def.servo_id);
        if (pos.isNaN())
            return false;

        if (prev == nullptr) {
            bone->relAngle = bone->def.calcAbsAng(pos);
        } else {
            bone->relAngle = Arm::clamp(bone->def.calcAbsAng(pos) - prev->absAngle);
        }
        bone->updatePos(prev);
        prev = bone;
    }
    return true;
}

bool Arm::solve(Arm::CoordType target_x, Arm::CoordType target_y) {
    bool modified = false;
    bool result = false;
    for (size_t i = 0; i < 20; ++i) {
        if (solveIteration(target_x, target_y, modified)) {
            result = true;
            break;
        }
        if (!modified)
            break;
    }

    fixBodyCollision();

    return result;
}

bool Arm::solveIteration(Arm::CoordType target_x, Arm::CoordType target_y, bool& modified) {
    updateBones();

    // Move the target out of the robot's body
    if (target_x < m_def.body_radius - m_def.arm_offset_x) {
        target_y = std::min(target_y, m_def.arm_offset_y);
    } else {
        target_y = std::min(target_y, CoordType(m_def.arm_offset_y + m_def.body_height));
    }

    auto end_x = m_bones.back().x;
    auto end_y = m_bones.back().y;
    CoordType bx, by;
    modified = false;
    for (int32_t ii = int32_t(m_bones.size()) - 1; ii >= 0; --ii) {
        const size_t i = size_t(ii);
        if (i == 0) {
            bx = by = 0;
        } else {
            bx = m_bones[i - 1].x;
            by = m_bones[i - 1].y;
        }

        // Get the vector from the current bone to the end effector position.
        AngleType to_end_x = end_x - bx;
        AngleType to_end_y = end_y - by;
        AngleType to_end_mag = sqrt(to_end_x * to_end_x + to_end_y * to_end_y);

        // Get the vector from the current bone to the target position.
        AngleType to_target_x = target_x - bx;
        AngleType to_target_y = target_y - by;
        AngleType to_target_mag = sqrt(to_target_x * to_target_x + to_target_y * to_target_y);

        // Get rotation to place the end effector on the line from the current
        // joint position to the target postion.
        AngleType cos_rot_ang, sin_rot_ang;
        AngleType end_target_mag = to_end_mag * to_target_mag;

        if (end_target_mag <= AngleType(0.0001)) {
            cos_rot_ang = 1;
            sin_rot_ang = 0;
        } else {
            cos_rot_ang = (to_end_x * to_target_x + to_end_y * to_target_y) / end_target_mag;
            sin_rot_ang = (to_end_x * to_target_y - to_end_y * to_target_x) / end_target_mag;
        }

        // Clamp the cosine into range when computing the angle (might be out of range
        // due to floating point error).
        AngleType rot_ang = acos(std::max(AngleType(-1), std::min(AngleType(1), cos_rot_ang)));
        if (sin_rot_ang < 0)
            rot_ang = -rot_ang;

        // Rotate the current bone in local space (this value is output to the user)
        rot_ang = rotateArm(i, rot_ang);
        cos_rot_ang = cos(rot_ang);
        sin_rot_ang = sin(rot_ang);

        // Rotate the end effector position.
        end_x = roundCoord(bx + cos_rot_ang * to_end_x - sin_rot_ang * to_end_y);
        end_y = roundCoord(by + sin_rot_ang * to_end_x + cos_rot_ang * to_end_y);

        // Check for termintation
        const auto dist_x = target_x - end_x;
        const auto dist_y = target_y - end_y;
        const auto dist = dist_x * dist_x + dist_y * dist_y;
        if (dist <= 15) {
            return true;
        }

        modified = modified || fabs(rot_ang) * to_end_mag > AngleType(0.000001);
    }
    return false;
}

Arm::AngleType Arm::rotateArm(size_t idx, Arm::AngleType rot_ang) {
    auto& me = m_bones[idx];
    auto& base = m_bones[0];

    AngleType new_rel_ang = clamp(AngleType(me.relAngle.rad()) + rot_ang);
    new_rel_ang = std::max(AngleType(me.def.rel_min.rad()),
        std::min(AngleType(me.def.rel_max.rad()), new_rel_ang));

    CoordType x = 0;
    CoordType y = 0;
    AngleType prev_ang = 0;
    for (size_t i = 0; i < m_bones.size(); ++i) {
        auto& b = m_bones[i];
        auto angle = AngleType(b.relAngle.rad());
        if (i == idx) {
            angle = new_rel_ang;
        }
        angle = clamp(prev_ang + angle);

        // Check collision of the back helper arms with the body.
        if (i == idx) {
            if (angle < AngleType(b.def.abs_min.rad())) {
                angle = AngleType(b.def.abs_min.rad());
                new_rel_ang = clamp(angle - prev_ang);
            } else if (angle > AngleType(b.def.abs_max.rad())) {
                angle = AngleType(b.def.abs_max.rad());
                new_rel_ang = clamp(angle - prev_ang);
            }
        }

        auto nx = roundCoord(x + (cos(angle) * b.def.length));
        auto ny = roundCoord(y + (sin(angle) * b.def.length));

        // Check collision with the base arm
        if (i > 0) {
            const auto diff = Angle::_T(angle) - base.absAngle.rad();
            if (diff < b.def.base_rel_min.rad()) {
                base.absAngle = Angle::rad(Angle::_T(clamp(angle - AngleType(b.def.base_rel_min.rad()))));
            } else if (diff > b.def.base_rel_max.rad()) {
                base.absAngle = Angle::rad(Angle::_T(clamp(angle - AngleType(b.def.base_rel_max.rad()))));
            }
        }

        x = nx;
        y = ny;
        prev_ang = angle;
    }

    auto res = clamp(new_rel_ang - AngleType(me.relAngle.rad()));
    me.relAngle = Angle::rad(Angle::_T(new_rel_ang));
    return res;
}

void Arm::fixBodyCollision() {
    auto& end = m_bones.back();
    auto& base = m_bones.front();
    if (base.relAngle.rad() > base.def.rel_max.rad())
        base.relAngle = base.def.rel_max;
    else if (base.relAngle.rad() < base.def.rel_min.rad())
        base.relAngle = base.def.rel_min;

    updateBones();

    while (isInBody(end.x, end.y)) {
        Angle newang = clamp(base.relAngle - 0.01_rad);
        if (newang.rad() > base.def.rel_max.rad() || newang.rad() < base.def.rel_min.rad())
            return;
        base.relAngle = newang;
        updateBones();
    }
}

bool Arm::isInBody(Arm::CoordType x, Arm::CoordType y) const {
    return abs(x) <= m_def.body_radius && y >= m_def.arm_offset_y;
}

void Arm::updateBones() {
    Bone* prev = nullptr;
    for (size_t i = 0; i < m_bones.size(); ++i) {
        m_bones[i].updatePos(prev);
        prev = &m_bones[i];
    }
}

Bone::Bone(const Arm::BoneDefinition& def)
    : def(def) {
    relAngle = -Angle::Pi / 2;
    x = y = 0;
}

}; // namespace rb
