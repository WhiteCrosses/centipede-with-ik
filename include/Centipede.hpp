#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

// Joint limits (radians) shared across modules.
// These are enforced by the IK solver and gait code. Values are in radians.
// - `kHipPitch*` limit the leg's pitch (up/down) around the hip joint.
// - `kKnee*` limit the knee joint (typically a negative range since the
//   model measures knee flexion as a negative rotation from the hip pitch).
// - `kHipYawMaxDelta` determines allowable yaw deviation from the outward
//   facing direction (total yaw range = ±kHipYawMaxDelta).
// Examples:
//   kHipPitchMin = -1.25 rad ≈ -71.6° (points downward)
//   kHipPitchMax =  0.15 rad ≈  8.6°  (slightly upward)
inline constexpr float kHipPitchMin = -1.25f;
inline constexpr float kHipPitchMax =  0.15f;
inline constexpr float kKneeMin     = -2.00f;
inline constexpr float kKneeMax     = -0.05f;
// Hip yaw (horizontal plane) limit: allow up to 120° sweep total (±60°)
inline constexpr float kHipYawMaxDelta = 1.04719755f; // pi/3 (≈ 60°)

// Geometry defaults shared across modules
// Units: grid/tile units for X/Y/Z distances. These are nominal lengths used
// by the IK and drawing code. `resf` (tile pixel size) scales the visual
// rendering but the kinematic lengths operate in grid units.
inline constexpr float kStanceWidth = 1.0f;   // lateral offset from spine to hip attach
inline constexpr float kCoxaLength = 1.4f;    // distance from hip attach to hip joint

struct Voxel {
    float baseOx, baseOy;
    float wx, wy;
    float vx, vy;
    uint8_t filled;
};

struct Segment {
    float x, y;
    float px, py;
    float angle;
    sf::Color color;
    int voxW, voxH;
    std::vector<Voxel> voxels;
    bool moved;
    struct Leg {
        // hipOx/hipOy : local offset of hip attachment relative to the segment
        float hipOx, hipOy;
        // side : -1 or +1 for left/right legs (helps orient coxa offset)
        int side;

        // Current joint angles (radians): these are the solved output from IK
        // - `hipAngle` : yaw (rotation around vertical axis) or primary hip angle
        // - `kneeAngle`: pitch of the hip link (positive upward in our convention)
        // - `footAngle`: additional pitch added at the foot/ankle joint
        float hipAngle, kneeAngle, footAngle;

        // Target angles that the IK/gait attempt to approach (used for smoothing)
        float targetHipAngle, targetKneeAngle, targetFootAngle;

        // Gait timing parameters
        float phaseOffset; // per-leg phase offset (0..1) for metachronal waves
        float cycle;       // current normalized cycle phase (0..1)

        // world-space foot anchor used when the foot is planted on ground
        float footHoldX, footHoldY;

        // swingPhase: 0..1 progress through the swing motion (1 -> landing)
        float swingPhase;

        // Cached positions used to prevent foot sliding while planted:
        // - swingStartX/Y : foot position at the moment the leg entered swing
        // - swingLandX/Y  : precomputed landing target for the current swing.
        float swingStartX, swingStartY;
        float swingLandX, swingLandY;

        // Link lengths in grid units (used by IK and for drawing FK)
        float hipLength, kneeLength, footLength;

        // Ground interaction and pushing
        float pushStrength; // how strongly this leg pushes the body when planted
        bool onGround;      // true when foot is considered planted

        // Coxa: short link from the body/spine out to the hip joint.
        // This is separate from `hipLength` which is the first major leg segment.
        float coxaLength;
    };
    std::vector<Leg> legs;
};

class Centipede {
private:
    std::vector<Segment> segments;
    int dirX, dirY;
    int moveCounter;
    static const int moveDelay;
    static constexpr float followSpeed = 0.28f;
    static constexpr float maxMovePerTry = 1.2f;
    float gaitTime;
    float lastHeadX, lastHeadY;
    // Last applied head movement delta (grid units). Used to align gait to travel direction.
    float lastMoveDx = 0.0f;
    float lastMoveDy = 0.0f;
public:
    Centipede(int startX, int startY, int length);
    void update();
    void tryMove(float dx, float dy);
    void moveBy(float dx, float dy);
    void render(sf::RenderWindow* window, float resf);
    const std::vector<Segment>& getSegments() const;
};
