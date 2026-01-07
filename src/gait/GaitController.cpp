#include "GaitController.hpp"
#include <cmath>
#include "GaitController.hpp"
#include <cmath>
#include <algorithm>

namespace gait {

void updateGait(std::vector<Segment> &segments, float gaitTime, float bodyZ, float lastMoveDx, float lastMoveDy) {
    const float PI = 3.14159265f;
    const float stanceFrac = 0.55f;
    const float stanceWidth = kStanceWidth;
    const float desiredSweepDeg = 150.0f;
    const float desiredHalfSweep = (desiredSweepDeg * (PI / 180.0f)) * 0.5f;

    float baseSpineX = 1.f, baseSpineY = 0.f;
    if (segments.size() >= 2) {
        baseSpineX = segments[1].x - segments[0].x;
        baseSpineY = segments[1].y - segments[0].y;
        float bl = std::sqrt(baseSpineX * baseSpineX + baseSpineY * baseSpineY);
        if (bl > 1e-4f) { baseSpineX /= bl; baseSpineY /= bl; } else { baseSpineX = 1.f; baseSpineY = 0.f; }
    }

    float moveDirX = lastMoveDx;
    float moveDirY = lastMoveDy;
    float moveMag = std::sqrt(moveDirX * moveDirX + moveDirY * moveDirY);
    if (moveMag > 1e-4f) {
        moveDirX /= moveMag;
        moveDirY /= moveMag;
    } else {
        moveDirX = baseSpineX;
        moveDirY = baseSpineY;
    }

    const float kMoveBias = 0.85f;
    float forwardX = baseSpineX * (1.0f - kMoveBias) + moveDirX * kMoveBias;
    float forwardY = baseSpineY * (1.0f - kMoveBias) + moveDirY * kMoveBias;
    float forwardLen = std::sqrt(forwardX * forwardX + forwardY * forwardY);
    if (forwardLen < 1e-4f) { forwardX = 1.f; forwardY = 0.f; forwardLen = 1.f; }
    forwardX /= forwardLen;
    forwardY /= forwardLen;

    for (size_t i = 0; i < segments.size(); ++i) {
        auto &seg = segments[i];

        float spineX = 0.f, spineY = 0.f;
        if (i < segments.size() - 1) {
            spineX = segments[i + 1].x - segments[i].x;
            spineY = segments[i + 1].y - segments[i].y;
        } else if (i > 0) {
            spineX = segments[i].x - segments[i - 1].x;
            spineY = segments[i].y - segments[i - 1].y;
        } else {
            spineX = forwardX;
            spineY = forwardY;
        }
        float spineLen = std::sqrt(spineX * spineX + spineY * spineY);
        if (spineLen < 1e-4f) { spineX = forwardX; spineY = forwardY; spineLen = 1.f; }
        spineX /= spineLen;
        spineY /= spineLen;
        float perpX = -spineY;
        float perpY = spineX;

        float midX = (i < segments.size() - 1) ? (segments[i].x + segments[i + 1].x) * 0.5f : segments[i].x;
        float midY = (i < segments.size() - 1) ? (segments[i].y + segments[i + 1].y) * 0.5f : segments[i].y;

        for (auto &leg : seg.legs) {
            const bool wasOnGround = leg.onGround;

            float phase = std::fmod(gaitTime + leg.phaseOffset, 2.0f * PI);
            if (phase < 0.f) phase += 2.0f * PI;

            const float stanceEnd = stanceFrac * 2.0f * PI;
            const bool inSwing = (phase >= stanceEnd);

            float attachX = midX + perpX * (stanceWidth * static_cast<float>(leg.side));
            float attachY = midY + perpY * (stanceWidth * static_cast<float>(leg.side));

            float coxaAttachX = attachX + perpX * leg.coxaLength * static_cast<float>(leg.side);
            float coxaAttachY = attachY + perpY * leg.coxaLength * static_cast<float>(leg.side);

            const float L1 = leg.hipLength;
            const float L2 = leg.kneeLength + leg.footLength;
            const float maxDist = (L1 + L2) - 0.05f;
            const float dzAbs = std::fabs(bodyZ);
            float maxReachR = 0.0f;
            if (dzAbs < maxDist) {
                maxReachR = std::sqrt(std::max(0.0f, maxDist * maxDist - dzAbs * dzAbs));
            }

            const float outDirX = perpX * static_cast<float>(leg.side);
            const float outDirY = perpY * static_cast<float>(leg.side);

            const float baseOutR = maxReachR * std::cos(desiredHalfSweep);
            const float forwardAmp = maxReachR * std::sin(desiredHalfSweep);

            float restX = coxaAttachX + outDirX * baseOutR;
            float restY = coxaAttachY + outDirY * baseOutR;

            float landX = restX + forwardX * forwardAmp;
            float landY = restY + forwardY * forwardAmp;

            {
                float toTX = landX - coxaAttachX;
                float toTY = landY - coxaAttachY;
                float outComp = toTX * outDirX + toTY * outDirY;
                float minOut = baseOutR * 0.95f;
                if (outComp < minOut) {
                    float add = (minOut - outComp);
                    landX += outDirX * add;
                    landY += outDirY * add;
                }
            }

            if (inSwing) {
                leg.onGround = false;
                if (wasOnGround) {
                    leg.swingStartX = leg.footHoldX;
                    leg.swingStartY = leg.footHoldY;
                }

                float swingT = (phase - stanceEnd) / (2.0f * PI - stanceEnd);
                swingT = std::clamp(swingT, 0.0f, 1.0f);
                leg.swingPhase = swingT;

                float t = swingT * swingT * (3.0f - 2.0f * swingT);
                leg.footHoldX = leg.swingStartX + (landX - leg.swingStartX) * t;
                leg.footHoldY = leg.swingStartY + (landY - leg.swingStartY) * t;
            } else {
                leg.onGround = true;
                leg.swingPhase = 0.f;

                if (!wasOnGround) {
                    leg.footHoldX = landX;
                    leg.footHoldY = landY;
                }
            }
        }
    }
}

} // namespace gait

