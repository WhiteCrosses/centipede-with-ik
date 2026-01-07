#pragma once

#include "../../include/Centipede.hpp"

namespace gait {
    // Update gait state (swing/stance and foot holds) for all segments.
    // - `gaitTime` is the global phase accumulator (radians).
    // - `bodyZ` is current body height used to compute reach.
    // - `lastMoveDx/lastMoveDy` are last applied movement deltas to bias forward direction.
    void updateGait(std::vector<Segment> &segments, float gaitTime, float bodyZ, float lastMoveDx, float lastMoveDy);
}
