#include "LegIK.hpp"
#include <cmath>
#include <algorithm>

namespace ik {

static float wrapAngle(float a) {
    const float PI = 3.14159265f;
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

void solveLeg(Segment::Leg &leg, float coxaAttachX, float coxaAttachY, float bodyZ, float yawRef) {
    const float targetZ = 0.0f;
    const float dz = targetZ - bodyZ; // negative => down

    float dx = leg.footHoldX - coxaAttachX;
    float dy = leg.footHoldY - coxaAttachY;
    float r = std::sqrt(dx * dx + dy * dy);
    float dist = std::sqrt(r * r + dz * dz);

    const float L1 = leg.hipLength;
    const float L2 = leg.kneeLength + leg.footLength;

    const float maxDist = (L1 + L2) - 0.05f;
    const float minDist = std::fabs(L1 - L2) + 0.05f;
    float clampedDist = std::clamp(dist, minDist, maxDist);

    if (dist > 1e-4f && std::fabs(clampedDist - dist) > 1e-5f) {
        float desiredR = std::sqrt(std::max(0.0f, clampedDist * clampedDist - dz * dz));
        float scale = (r > 1e-4f) ? (desiredR / r) : 0.0f;
        dx *= scale;
        dy *= scale;

        // Only adjust the stored foot hold while swinging; stance feet should stay planted.
        if (!leg.onGround) {
            leg.footHoldX = coxaAttachX + dx;
            leg.footHoldY = coxaAttachY + dy;
        }

        r = desiredR;
        dist = clampedDist;
    }

    float yaw = (r > 1e-6f) ? std::atan2(dy, dx) : leg.hipAngle;

    float cosKnee = (r * r + dz * dz - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cosKnee = std::clamp(cosKnee, -0.999f, 0.999f);
    // Choose the "elbow-down" solution (knee bends toward ground): use a signed knee angle.
    float knee = -std::acos(cosKnee);

    float hipPitch = std::atan2(dz, r) - std::atan2(L2 * std::sin(knee), L1 + L2 * std::cos(knee));

    // Enforce hard joint limits.
    hipPitch = std::clamp(hipPitch, kHipPitchMin, kHipPitchMax);
    knee = std::clamp(knee, kKneeMin, kKneeMax);

    // Hard yaw clamp around the provided reference direction (horizontal plane).
    float yawDelta = wrapAngle(yaw - yawRef);
    yawDelta = std::clamp(yawDelta, -kHipYawMaxDelta, kHipYawMaxDelta);
    yaw = wrapAngle(yawRef + yawDelta);

    // Smooth angles with wrap-aware delta so we never jump across ±pi.
    float dyaw = wrapAngle(yaw - leg.hipAngle);
    leg.hipAngle = wrapAngle(leg.hipAngle + dyaw * 0.20f);
    leg.kneeAngle += (hipPitch - leg.kneeAngle) * 0.20f;
    leg.footAngle += (knee - leg.footAngle) * 0.20f;

    // Clamp state too (so smoothing can never overshoot past limits).
    float stateYawDelta = wrapAngle(leg.hipAngle - yawRef);
    stateYawDelta = std::clamp(stateYawDelta, -kHipYawMaxDelta, kHipYawMaxDelta);
    leg.hipAngle = wrapAngle(yawRef + stateYawDelta);
    leg.kneeAngle = std::clamp(leg.kneeAngle, kHipPitchMin, kHipPitchMax);
    leg.footAngle = std::clamp(leg.footAngle, kKneeMin, kKneeMax);
}

} // namespace ik
