#pragma once

#include "../../include/Centipede.hpp"

namespace ik {
    // Solve IK for a single leg. Updates `leg.hipAngle`, `leg.kneeAngle`, `leg.footAngle`.
    // - `coxaAttachX/Y` are the hip joint world position (after coxa offset).
    // - `bodyZ` is the hip Z (negative downwards is handled by solver as in original code).
    void solveLeg(Segment::Leg &leg, float coxaAttachX, float coxaAttachY, float bodyZ, float yawRef);
}
