#include "Centipede.hpp"
#include <cmath>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include <SFML/Graphics.hpp>
// Render helpers
#include "render/Projection.hpp"
#include "render/GridRenderer.hpp"
// Gait controller (extracted)
#include "gait/GaitController.hpp"
#include "ik/LegIK.hpp"
#include "render/DrawHelpers.hpp"

// static member definitions
const int Centipede::moveDelay = 2;

// Body suspension: rest height and current height above ground plane (z=0)
// Lower rest height so the body rides low (belly sliding)
static constexpr float kBodyRestZ = 0.6f;
static float g_bodyZ = kBodyRestZ;

// Leg attachment geometry (constants moved to include/Centipede.hpp)

// Hard joint limits (radians). These are enforced as absolute clamps, so joints can never
// bend beyond these angles even with smoothing/damping.
// Convention:
// - hipPitch is stored in Leg::kneeAngle and used as pitch in FK (negative = down).
// - knee bend is stored in Leg::footAngle and added to hipPitch (negative = bends down).
// joint limits moved to include/Centipede.hpp

// Build a centipede with evenly spaced segments, voxels, and initial leg phase offsets.
Centipede::Centipede(int startX, int startY, int length) : dirX(1), dirY(0), moveCounter(0) {
    this->gaitTime = 0.f;
    this->lastHeadX = static_cast<float>(startX);
    this->lastHeadY = static_cast<float>(startY);
    const int SEG_W = 3;
    for (int i = 0; i < length; i++) {
        Segment seg;
        seg.x = startX - i * SEG_W;
        seg.y = startY;
        seg.px = seg.x; seg.py = seg.y;
        seg.color = sf::Color(50,200,50);
        seg.voxW = SEG_W; seg.voxH = SEG_W;
        seg.voxels.clear(); seg.voxels.reserve(seg.voxW*seg.voxH);
        for (int yy=0; yy<seg.voxH; ++yy) for (int xx=0; xx<seg.voxW; ++xx) {
            Voxel v; int cx = seg.voxW/2, cy = seg.voxH/2; int ddx = xx-cx, ddy = yy-cy;
            // Simple diamond mask (smaller segment)
            v.filled = (std::abs(ddx)+std::abs(ddy) <= 1) ? 1 : 0;
            v.baseOx = static_cast<float>(xx); v.baseOy = static_cast<float>(yy);
            v.wx = seg.x + v.baseOx; v.wy = seg.y + v.baseOy; v.vx = v.vy = 0.f;
            seg.voxels.push_back(v);
        }
        seg.legs.clear(); seg.legs.reserve(2);
        for (int side=-1; side<=1; side+=2) {
            Segment::Leg L;
            L.hipOx = (seg.voxW*0.5f) + (side*1.2f);
            L.hipOy = seg.voxH*0.5f;
            L.side = side;
            L.hipAngle = L.kneeAngle = L.footAngle = 0.f;
            // Metachronal wave: fixed phase offset per segment (rear legs lead front legs).
            const float PI = 3.14159265f;
            const float phaseStep = PI / 4.0f; // 45 degrees per segment
            float sidePhase = (side==-1) ? 0.f : PI; // opposite side out of phase
            L.phaseOffset = static_cast<float>(i) * phaseStep + sidePhase;
            L.cycle = 0.f;
            L.footHoldX = seg.x + L.hipOx;
            L.footHoldY = seg.y + L.hipOy;
            L.swingPhase = 0.f;
            L.swingStartX = L.footHoldX;
            L.swingStartY = L.footHoldY;
            // Leg proportions: 3/2/1 (hip/knee/foot), keeping total length ~unchanged.
            const float totalLen = 2.0f + 2.0f + 1.5f;
            const float unit = totalLen / 6.0f;
            L.hipLength = 3.0f * unit;
            L.kneeLength = 2.0f * unit;
            L.footLength = 1.0f * unit;
            L.pushStrength = 0.06f;
            L.onGround = true;
            L.coxaLength = kCoxaLength;
            seg.legs.push_back(L);
        }
        segments.push_back(seg);
    }
}

// Rate-limited head move request with a safety clamp for huge mouse deltas.
void Centipede::tryMove(float dx, float dy) {
    // Clamp very large mouse moves: length is Euclidean norm sqrt(dx^2 + dy^2)
    float mag = std::sqrt(dx*dx + dy*dy);
    if (mag > Centipede::maxMovePerTry) { dx = dx/mag*Centipede::maxMovePerTry; dy = dy/mag*Centipede::maxMovePerTry; }
    moveCounter++; if (moveCounter < moveDelay) return; moveCounter = 0; moveBy(dx,dy);
}

void Centipede::moveBy(float dx, float dy) {
    if (segments.empty()) return;

    // Remember previous logical positions so followers can chase where the leader used to be.
    std::vector<std::pair<float,float>> prev; prev.reserve(segments.size());
    for (auto &s : segments) prev.emplace_back(s.x, s.y);
    for (auto &s : segments) s.moved = false;

    // Quick overlap test for hypothetical offsets against non-head voxels.
    auto wouldCollide = [&](float ox, float oy) -> bool {
        const Segment &head = segments[0];
        for (const auto &hv : head.voxels) {
            if (!hv.filled) continue;
            float vwx = hv.wx + ox; float vwy = hv.wy + oy;
            int igx = static_cast<int>(std::floor(vwx + 0.5f));
            int igy = static_cast<int>(std::floor(vwy + 0.5f));
            for (size_t si=1; si<segments.size(); ++si) {
                const Segment &other = segments[si];
                for (const auto &ov : other.voxels) {
                    if (!ov.filled) continue;
                    int ogx = static_cast<int>(std::floor(ov.wx + 0.5f));
                    int ogy = static_cast<int>(std::floor(ov.wy + 0.5f));
                    if (ogx==igx && ogy==igy) return true;
                }
            }
        }
        return false;
    };

    // Occupancy map lets us relocate or push away blocking voxels.
    std::unordered_map<uint64_t, std::pair<int,int>> occ;
    auto cellKey = [](int gx, int gy) -> uint64_t { return (static_cast<uint64_t>(static_cast<uint32_t>(gx))<<32) | static_cast<uint32_t>(gy); };
    for (int si=0; si<static_cast<int>(segments.size()); ++si) {
        const Segment &s = segments[si];
        for (int vi=0; vi<static_cast<int>(s.voxels.size()); ++vi) {
            const Voxel &v = s.voxels[vi]; if (!v.filled) continue;
            int igx = static_cast<int>(std::floor(v.wx + 0.5f));
            int igy = static_cast<int>(std::floor(v.wy + 0.5f));
            occ[cellKey(igx,igy)] = {si, vi};
        }
    }

    // Try to move a single voxel to the nearest free ring of cells.
    auto relocateVoxel = [&](int ownerSeg, int ownerVox) -> bool {
        Voxel &ov = segments[ownerSeg].voxels[ownerVox];
        int igx = static_cast<int>(std::floor(ov.wx + 0.5f));
        int igy = static_cast<int>(std::floor(ov.wy + 0.5f));
        bool placed = false;
        for (int radius=1; radius<=6 && !placed; ++radius) {
            for (int dxr=-radius; dxr<=radius && !placed; ++dxr) for (int dyr=-radius; dyr<=radius && !placed; ++dyr) {
                if (std::abs(dxr)!=radius && std::abs(dyr)!=radius) continue;
                int nx = igx + dxr; int ny = igy + dyr; uint64_t nk = cellKey(nx,ny);
                if (occ.find(nk)==occ.end()) { occ.erase(cellKey(igx,igy)); ov.wx = static_cast<float>(nx); ov.wy = static_cast<float>(ny); occ[nk] = {ownerSeg, ownerVox}; placed = true; }
            }
        }
        return placed;
    };

    // Compute the integer grid cells the head wants to occupy after this move.
    std::vector<uint64_t> headTargets;
    {
        const Segment &head = segments[0];
        for (const auto &hv : head.voxels) {
            if (!hv.filled) continue;
            float vwx = hv.wx + dx; float vwy = hv.wy + dy;
            int igx = static_cast<int>(std::floor(vwx + 0.5f)); int igy = static_cast<int>(std::floor(vwy + 0.5f));
            headTargets.push_back(cellKey(igx,igy));
        }
    }

    // Iteratively clear head targets by relocating or pushing blocking segments.
    const int maxIterations = 5; bool headFree = false;
    for (int iter=0; iter<maxIterations && !headFree; ++iter) {
        headFree = true;
        occ.clear();
        for (int si=0; si<static_cast<int>(segments.size()); ++si) {
            const Segment &s = segments[si];
            for (int vi=0; vi<static_cast<int>(s.voxels.size()); ++vi) {
                const Voxel &v = s.voxels[vi]; if (!v.filled) continue;
                int igx = static_cast<int>(std::floor(v.wx + 0.5f)); int igy = static_cast<int>(std::floor(v.wy + 0.5f)); occ[cellKey(igx,igy)] = {si,vi};
            }
        }
        for (auto tk : headTargets) {
            auto it = occ.find(tk);
            if (it!=occ.end()) {
                int osi = it->second.first; int ovi = it->second.second; if (osi==0) continue;
                bool ok = relocateVoxel(osi, ovi);
                if (!ok) {
                    Segment &ownerSeg = segments[osi]; const Segment &head = segments[0];
                    float vx = ownerSeg.x - head.x; float vy = ownerSeg.y - head.y; float vlen = std::sqrt(vx*vx+vy*vy);
                    if (vlen < 0.001f) { vx = 1.f; vy = 0.f; vlen = 1.f; }
                    vx/=vlen; vy/=vlen; // normalize
                    float pushBase = 1.5f; float pushDist = pushBase * (1.0f + iter * 0.7f); // progressively stronger pushes per iteration
                    for (int vii=0; vii<static_cast<int>(ownerSeg.voxels.size()); ++vii) { Voxel &ov = ownerSeg.voxels[vii]; int oxg = static_cast<int>(std::floor(ov.wx+0.5f)); int oyg = static_cast<int>(std::floor(ov.wy+0.5f)); occ.erase(cellKey(oxg, oyg)); }
                    ownerSeg.x += vx * pushDist; ownerSeg.y += vy * pushDist; ownerSeg.moved = true;
                    for (auto &ov : ownerSeg.voxels) { ov.wx += vx * pushDist; ov.wy += vy * pushDist; }
                    for (int vii=0; vii<static_cast<int>(ownerSeg.voxels.size()); ++vii) { Voxel &ov = ownerSeg.voxels[vii]; int nx = static_cast<int>(std::floor(ov.wx+0.5f)); int ny = static_cast<int>(std::floor(ov.wy+0.5f)); occ[cellKey(nx,ny)] = {osi, vii}; }
                    headFree = false;
                }
            }
        }
    }

    // Final check: are any head target cells still occupied by others?
    bool blocked = false;
    for (auto tk : headTargets) { auto it = occ.find(tk); if (it!=occ.end()) { if (it->second.first != 0) { blocked = true; break; } } }

    float applyDx = 0.f, applyDy = 0.f;
    if (!blocked) { applyDx = dx; applyDy = dy; }
    else {
        bool colX = wouldCollide(dx, 0.f); bool colY = wouldCollide(0.f, dy);
        if (!colX) { applyDx = dx; applyDy = 0.f; } else if (!colY) { applyDx = 0.f; applyDy = dy; } else { applyDx = 0.f; applyDy = 0.f; }
    }

    // Boundary checking in screen space (account for isometric projection)
    float resf = 10.0f;  // resolution factor (window 800x800, grid 80x80)
    float newHeadX = segments[0].x + applyDx;
    float newHeadY = segments[0].y + applyDy;
    
    // Convert to screen space using isometric projection
    float halfW = resf * 0.5f;
    float halfH = resf * 0.25f;
    float cx = 400.0f;  // window width/2
    float cy = 50.0f;   // base screen Y offset
    
    float screenX = (newHeadX - newHeadY) * halfW + cx;
    float screenY = (newHeadX + newHeadY) * halfH + cy;
    
    // Check if in screen bounds with margin
    float margin = 20.0f;
    bool inBoundsX = (screenX >= margin && screenX <= (800.0f - margin));
    bool inBoundsY = (screenY >= margin && screenY <= (800.0f - margin));
    
    if (!inBoundsX || !inBoundsY) {
        // Clamp to boundary: try X only, then Y only, then neither
        newHeadX = segments[0].x + (inBoundsX ? applyDx : 0.f);
        newHeadY = segments[0].y + (inBoundsY ? applyDy : 0.f);
        applyDx = newHeadX - segments[0].x;
        applyDy = newHeadY - segments[0].y;
    }

    segments[0].x += applyDx; segments[0].y += applyDy;
    for (auto &hv : segments[0].voxels) { hv.wx += applyDx; hv.wy += applyDy; }
    segments[0].moved = (std::abs(applyDx) > 1e-6f || std::abs(applyDy) > 1e-6f);
    if (segments[0].moved) segments[0].angle = std::atan2(applyDy, applyDx);

    // Remember last movement so gait can align to the destination direction.
    this->lastMoveDx = segments[0].moved ? applyDx : 0.0f;
    this->lastMoveDy = segments[0].moved ? applyDy : 0.0f;

    for (size_t i=1; i<segments.size(); ++i) {
        if (!segments[i-1].moved) { segments[i].moved = false; continue; }
        float targetX = prev[i-1].first; float targetY = prev[i-1].second;
        float oldx = segments[i].x; float oldy = segments[i].y;
        segments[i].x += (targetX - segments[i].x) * Centipede::followSpeed * 0.9f;
        segments[i].y += (targetY - segments[i].y) * Centipede::followSpeed * 0.9f;
        segments[i].moved = (std::abs(segments[i].x - oldx) > 1e-4f || std::abs(segments[i].y - oldy) > 1e-4f);
        float dx_to_pred = segments[i-1].x - segments[i].x; float dy_to_pred = segments[i-1].y - segments[i].y;
        float dist_to_pred = std::sqrt(dx_to_pred*dx_to_pred + dy_to_pred*dy_to_pred);
        if (dist_to_pred > 0.1f) {
            float target_angle = std::atan2(dy_to_pred, dx_to_pred);
            float da = target_angle - segments[i].angle; if (da > 3.14159f) da -= 6.28318f; if (da < -3.14159f) da += 6.28318f;
            segments[i].angle += da * 0.15f;
        }
        // Pull follower voxels toward their logical centers with damping.
        for (auto &v : segments[i].voxels) {
            if (!v.filled) continue;
            float targetWx = segments[i].x + v.baseOx; float targetWy = segments[i].y + v.baseOy;
            float k = 0.22f; v.vx += (targetWx - v.wx) * k; v.vy += (targetWy - v.wy) * k; v.vx *= 0.82f; v.vy *= 0.82f; v.wx += v.vx; v.wy += v.vy;
        }
        // Simple overlap push-off so followers do not sit inside others.
        for (size_t sj=0; sj<segments.size(); ++sj) {
            if (sj == i) continue; const Segment &other = segments[sj]; bool pushed = false;
            for (const auto &ov : other.voxels) { if (!ov.filled) continue; int ogx = static_cast<int>(std::floor(ov.wx+0.5f)); int ogy = static_cast<int>(std::floor(ov.wy+0.5f));
                for (auto &fv : segments[i].voxels) { if (!fv.filled) continue; int fgx = static_cast<int>(std::floor(fv.wx+0.5f)); int fgy = static_cast<int>(std::floor(fv.wy+0.5f)); if (fgx==ogx && fgy==ogy) {
                    float pushX = (segments[i].x - other.x) * 0.2f; float pushY = (segments[i].y - other.y) * 0.2f; segments[i].x += (pushX==0.f?0.2f:pushX); segments[i].y += (pushY==0.f?0.2f:pushY);
                    for (auto &fv2 : segments[i].voxels) { fv2.wx += (pushX==0.f?0.2f:pushX); fv2.wy += (pushY==0.f?0.2f:pushY); }
                    pushed = true; break; }
                }
                if (pushed) break;
            }
            if (pushed) break;
        }
    }

    for (auto &s : segments) { s.px = s.x; s.py = s.y; }
}

void Centipede::update() {
    // Gallop-style gait: legs move in coordinated bursts
    // Like a horse but with many legs - creates powerful pushing motion
    
    // Advance gait time: scale with real movement so legs "walk" toward the mouse destination.
    // (Idle is slow; moving faster increases cadence.)
    const float kIdleGait = 0.015f;
    const float kGaitPerUnit = 5.55f; // radians per grid-unit moved
    
    // Track movement for reference
    float headMove = 0.f;
    if (!segments.empty()) {
        headMove = std::sqrt((segments[0].x - this->lastHeadX)*(segments[0].x - this->lastHeadX) + 
                             (segments[0].y - this->lastHeadY)*(segments[0].y - this->lastHeadY));
        this->lastHeadX = segments[0].x;
        this->lastHeadY = segments[0].y;
    }

    const float gaitAdvance = kIdleGait + headMove * kGaitPerUnit;
    this->gaitTime += gaitAdvance;

    // Delegate gait/step planning to the gait controller module.
    gait::updateGait(segments, this->gaitTime, g_bodyZ, this->lastMoveDx, this->lastMoveDy);

    // Estimate supported body height from planted legs.
    float supportedZSum = 0.0f;
    int supportedZCount = 0;
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
            spineX = 1.f;
            spineY = 0.f;
        }
        float spineLen = std::sqrt(spineX * spineX + spineY * spineY);
        if (spineLen < 0.001f) { spineX = 1.f; spineY = 0.f; spineLen = 1.f; }
        spineX /= spineLen;
        spineY /= spineLen;
        float perpX = -spineY;
        float perpY = spineX;

        float midX = (i < segments.size() - 1) ? (segments[i].x + segments[i + 1].x) * 0.5f : segments[i].x;
        float midY = (i < segments.size() - 1) ? (segments[i].y + segments[i + 1].y) * 0.5f : segments[i].y;

        for (auto &leg : seg.legs) {
            if (!leg.onGround) continue;

            float attachX = midX + perpX * (kStanceWidth * static_cast<float>(leg.side));
            float attachY = midY + perpY * (kStanceWidth * static_cast<float>(leg.side));
            float coxaAttachX = attachX + perpX * leg.coxaLength * static_cast<float>(leg.side);
            float coxaAttachY = attachY + perpY * leg.coxaLength * static_cast<float>(leg.side);

            float dxHold = leg.footHoldX - coxaAttachX;
            float dyHold = leg.footHoldY - coxaAttachY;
            float rHold = std::sqrt(dxHold * dxHold + dyHold * dyHold);

            const float L1 = leg.hipLength;
            const float L2 = leg.kneeLength + leg.footLength;
            const float total = L1 + L2;
            const float preferredExt = 0.75f;
            float preferredDist = preferredExt * total;
            preferredDist = std::clamp(preferredDist, std::fabs(L1 - L2) + 0.05f, total - 0.05f);

            float zFromLeg = 0.0f;
            if (rHold < preferredDist) {
                zFromLeg = std::sqrt(std::max(0.0f, preferredDist * preferredDist - rHold * rHold));
            } else {
                zFromLeg = 0.05f;
            }

            float zMin = 0.15f;
            float zMax = 2.0f;
            zFromLeg = std::clamp(zFromLeg, zMin, zMax);

            supportedZSum += zFromLeg;
            supportedZCount += 1;
        }
    }

    // Update body height from supports; if no legs are planted, relax back toward rest height.
    float targetBodyZ = (supportedZCount > 0) ? (supportedZSum / static_cast<float>(supportedZCount)) : kBodyRestZ;
    // Smooth to avoid bobbing
    g_bodyZ += (targetBodyZ - g_bodyZ) * 0.12f;
    g_bodyZ = std::clamp(g_bodyZ, 0.15f, 2.0f);

    // Pass 2: solve fully-3D leg IK (yaw + pitch + knee) using the current suspended body height
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
            spineX = 1.f;
            spineY = 0.f;
        }
        float spineLen = std::sqrt(spineX * spineX + spineY * spineY);
        if (spineLen < 0.001f) {
            spineX = 1.f;
            spineY = 0.f;
            spineLen = 1.f;
        }
        spineX /= spineLen;
        spineY /= spineLen;
        float perpX = -spineY;
        float perpY = spineX;

        const float stanceWidth = kStanceWidth;

        float midX = (i < segments.size() - 1) ? (segments[i].x + segments[i + 1].x) * 0.5f : segments[i].x;
        float midY = (i < segments.size() - 1) ? (segments[i].y + segments[i + 1].y) * 0.5f : segments[i].y;

        for (auto &leg : seg.legs) {
            float attachX = midX + perpX * (stanceWidth * static_cast<float>(leg.side));
            float attachY = midY + perpY * (stanceWidth * static_cast<float>(leg.side));

            float coxaAttachX = attachX + perpX * leg.coxaLength * static_cast<float>(leg.side);
            float coxaAttachY = attachY + perpY * leg.coxaLength * static_cast<float>(leg.side);

            const float outDirX = perpX * static_cast<float>(leg.side);
            const float outDirY = perpY * static_cast<float>(leg.side);
            const float yawRef = std::atan2(outDirY, outDirX);

            ik::solveLeg(leg, coxaAttachX, coxaAttachY, g_bodyZ, yawRef);
        }
    }

    // Update follower positions
    for (size_t i = 0; i < segments.size(); ++i) {
        float targetX = static_cast<float>(segments[i].x);
        float targetY = static_cast<float>(segments[i].y);
        segments[i].px += (targetX - segments[i].px) * Centipede::followSpeed;
        segments[i].py += (targetY - segments[i].py) * Centipede::followSpeed;
    }

    // Soft-body: every voxel drifts toward its segment center; stronger when the segment moved.
    for (auto &seg : segments) {
        for (auto &v : seg.voxels) {
            if (!v.filled) continue;
            float targetWx = seg.x + v.baseOx; float targetWy = seg.y + v.baseOy; float k_center = 0.04f; v.vx += (targetWx - v.wx) * k_center; v.vy += (targetWy - v.wy) * k_center;
            if (seg.moved) { float k_move = 0.12f; v.vx += (targetWx - v.wx) * k_move; v.vy += (targetWy - v.wy) * k_move; } // stronger spring after movement
            v.vx *= 0.85f; v.vy *= 0.85f; v.wx += v.vx; v.wy += v.vy;
        }
    }

    // Rebuild occupancy to eject any overlapping voxels after dynamics.
    std::unordered_map<uint64_t, std::pair<int,int>> occ; auto cellKey = [](int gx, int gy) -> uint64_t { return (static_cast<uint64_t>(static_cast<uint32_t>(gx))<<32) | static_cast<uint32_t>(gy); };
    for (int si=0; si<static_cast<int>(segments.size()); ++si) {
        auto &seg = segments[si]; for (int vi=0; vi<static_cast<int>(seg.voxels.size()); ++vi) { Voxel &v = seg.voxels[vi]; if (!v.filled) continue; int igx = static_cast<int>(std::floor(v.wx+0.5f)); int igy = static_cast<int>(std::floor(v.wy+0.5f)); uint64_t k = cellKey(igx,igy); if (occ.find(k)==occ.end()) { occ[k] = {si,vi}; continue; }
            bool placed = false; const float step = 0.25f; for (int radius=1; radius<=6 && !placed; ++radius) { for (int dx=-radius; dx<=radius && !placed; ++dx) { for (int dy=-radius; dy<=radius && !placed; ++dy) { if (std::abs(dx)!=radius && std::abs(dy)!=radius) continue; int nx = igx + dx; int ny = igy + dy; uint64_t nk = cellKey(nx,ny); if (occ.find(nk)==occ.end()) { v.wx = static_cast<float>(nx); v.wy = static_cast<float>(ny); occ[nk] = {si,vi}; placed = true; } } } }
            if (!placed) { for (int attempt=0; attempt<8 && !placed; ++attempt) { v.wx += v.vx * step; v.wy += v.vy * step; int nx = static_cast<int>(std::floor(v.wx+0.5f)); int ny = static_cast<int>(std::floor(v.wy+0.5f)); uint64_t nk = cellKey(nx,ny); if (occ.find(nk)==occ.end()) { occ[nk] = {si,vi}; placed = true; } } }
            if (!placed) occ[k] = {si,vi};
        }
    }
}

// Projection functions are implemented in src/render/Projection.cpp

// Draw spine sticks, leg attachments, articulated legs, and segment joints.
void Centipede::render(sf::RenderWindow* window, float resf) {
    // Draw isometric grid background (delegated to GridRenderer)
    drawGrid(window, resf);
    
    // First pass: Draw all spine sticks
    for (size_t i = 0; i < segments.size() - 1; ++i) {
        sf::Vector2f pos1 = gridToIsoZ(segments[i].x, segments[i].y, g_bodyZ, resf, window);
        sf::Vector2f pos2 = gridToIsoZ(segments[i+1].x, segments[i+1].y, g_bodyZ, resf, window);
        float stickLen = std::sqrt((pos2.x - pos1.x)*(pos2.x - pos1.x) + (pos2.y - pos1.y)*(pos2.y - pos1.y));
        if (stickLen > 0.1f) { 
            sf::RectangleShape stick(sf::Vector2f(stickLen, resf * 0.2f)); 
            float stickAngle = std::atan2(pos2.y - pos1.y, pos2.x - pos1.x) * 180.f / 3.14159f; 
            stick.setRotation(stickAngle); 
            stick.setPosition(pos1.x, pos1.y); 
            stick.setFillColor(sf::Color::Red); 
            window->draw(stick); 
        }
        sf::Vector2f midpoint = sf::Vector2f((pos1.x + pos2.x) * 0.5f, (pos1.y + pos2.y) * 0.5f);
        float legJointRadius = resf * 0.2f; 
        sf::CircleShape legJoint(legJointRadius); 
        legJoint.setFillColor(sf::Color::Green); 
        legJoint.setOrigin(legJointRadius, legJointRadius); 
        legJoint.setPosition(midpoint.x, midpoint.y); 
        window->draw(legJoint);
    }
    
    // Second pass: Draw all coxae
    for (size_t i = 0; i < segments.size() - 1; ++i) {
        float spineX = segments[i+1].x - segments[i].x; 
        float spineY = segments[i+1].y - segments[i].y; 
        float spineLen = std::sqrt(spineX*spineX + spineY*spineY); 
        if (spineLen < 0.001f) { spineX = 1.f; spineY = 0.f; spineLen = 1.f; } 
        spineX /= spineLen; 
        spineY /= spineLen;
        float perpX = -spineY; 
        float perpY = spineX;
        
        float midX = (segments[i].x + segments[i+1].x) * 0.5f; 
        float midY = (segments[i].y + segments[i+1].y) * 0.5f;

        const float stanceWidth = kStanceWidth;
        
        for (int side = -1; side <= 1; side += 2) {
            const auto &leg = segments[i].legs[side == -1 ? 0 : 1];
            float hipAttachX = midX + perpX * (stanceWidth * static_cast<float>(side)); 
            float hipAttachY = midY + perpY * (stanceWidth * static_cast<float>(side));
            
            // Coxa line: extends perpendicular from the spine to the hip joint
            float coxaEndX = hipAttachX + perpX * leg.coxaLength * static_cast<float>(side);
            float coxaEndY = hipAttachY + perpY * leg.coxaLength * static_cast<float>(side);
            sf::Vector2f coxaStart = gridToIsoZ(hipAttachX, hipAttachY, g_bodyZ, resf, window);
            sf::Vector2f coxaEnd = gridToIsoZ(coxaEndX, coxaEndY, g_bodyZ, resf, window);
            float coxaDist = std::sqrt((coxaEnd.x - coxaStart.x)*(coxaEnd.x - coxaStart.x) + (coxaEnd.y - coxaStart.y)*(coxaEnd.y - coxaStart.y));
            if (coxaDist > 0.1f) {
                sf::RectangleShape coxaSeg(sf::Vector2f(coxaDist, resf * 0.1f));
                float coxaAngle = std::atan2(coxaEnd.y - coxaStart.y, coxaEnd.x - coxaStart.x) * 180.f / 3.14159f;
                coxaSeg.setRotation(coxaAngle);
                coxaSeg.setPosition(coxaStart.x, coxaStart.y);
                coxaSeg.setFillColor(sf::Color::White);
                window->draw(coxaSeg);
            }
        }
    }
    
    // Third pass: Draw all leg joints and segments
    drawhelpers::drawCentipede(window, segments, resf, g_bodyZ);
}
const std::vector<Segment>& Centipede::getSegments() const { return segments; }
