
#include "DrawHelpers.hpp"
#include <cmath>
#include <algorithm>

namespace drawhelpers {

void drawCentipede(sf::RenderWindow* window, const std::vector<Segment> &segments, float resf, float bodyZ) {
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
            // Draw one leg on each side of the segment pair (left/right)
            const auto &leg = segments[i].legs[side == -1 ? 0 : 1];

            // Hip attachment point on the spine (midpoint offset by perpendicular)
            float hipAttachX = midX + perpX * (stanceWidth * static_cast<float>(side));
            float hipAttachY = midY + perpY * (stanceWidth * static_cast<float>(side));

            // Coxa end: short link from body to the hip joint. `coxaLength` is in
            // grid units; the 'side' multiplier places it left/right of the spine.
            float coxaEndX = hipAttachX + perpX * leg.coxaLength * static_cast<float>(side);
            float coxaEndY = hipAttachY + perpY * leg.coxaLength * static_cast<float>(side);
            float hipZ = bodyZ; // body elevation in grid units

            // Current joint angles (radians) computed by IK/gait:
            // - yaw: rotation around vertical axis, used to compute horizontal dir
            // - hipPitch: angle of first leg link in pitch (up/down)
            // - knee: additional pitch contributed by the knee joint
            float yaw = leg.hipAngle;
            float hipPitch = leg.kneeAngle;
            float knee = leg.footAngle;

            // Direction vector in the horizontal plane for the leg foot projection.
            // We interpret `yaw` such that (cos(yaw), sin(yaw)) points along the
            // leg's forward direction in grid-space.
            float dirX = std::cos(yaw);
            float dirY = std::sin(yaw);

            // Hip joint position (end of the coxa)
            float hipX = coxaEndX;
            float hipY = coxaEndY;

            // Compute knee position using standard forward kinematics for a planar
            // chain where pitch controls vertical lift and cos(pitch) scales horizontal reach.
            // Formulas:
            //   knee_xy = hip_xy + dir * (hipLength * cos(hipPitch))
            //   knee_z  = hip_z  +       (hipLength * sin(hipPitch))
            float kneeX = hipX + dirX * (leg.hipLength * std::cos(hipPitch));
            float kneeY = hipY + dirY * (leg.hipLength * std::cos(hipPitch));
            float kneeZ = hipZ + (leg.hipLength * std::sin(hipPitch));

            // Total pitch for the second link (knee + hipPitch)
            float link2Pitch = hipPitch + knee;

            // Ankle (end of second link)
            //   ankle_xy = knee_xy + dir * (kneeLength * cos(link2Pitch))
            //   ankle_z  = knee_z  +       (kneeLength * sin(link2Pitch))
            float ankleX = kneeX + dirX * (leg.kneeLength * std::cos(link2Pitch));
            float ankleY = kneeY + dirY * (leg.kneeLength * std::cos(link2Pitch));
            float ankleZ = kneeZ + (leg.kneeLength * std::sin(link2Pitch));

            // Foot (end of third link). We use the same pitch (link2Pitch) for the
            // foot link in this visualization; the real model may have more DOF.
            float footX = ankleX + dirX * (leg.footLength * std::cos(link2Pitch));
            float footY = ankleY + dirY * (leg.footLength * std::cos(link2Pitch));
            float footZ = ankleZ + (leg.footLength * std::sin(link2Pitch));
            // Prevent foot from floating above the visible ground plane (clip to z<=0)
            footZ = std::min(footZ, 0.0f);

            // Project 3D joint positions into screen space for drawing.
            sf::Vector2f coxaEndS = gridToIsoZ(coxaEndX, coxaEndY, hipZ, resf, window);
            sf::Vector2f kneeS = gridToIsoZ(kneeX, kneeY, kneeZ, resf, window);
            sf::Vector2f ankleS = gridToIsoZ(ankleX, ankleY, ankleZ, resf, window);
            sf::Vector2f footS = gridToIsoZ(footX, footY, footZ, resf, window);

            // Draw each link as a rotated rectangle from joint A to joint B.
            // The visual length is computed in screen space to ensure consistent
            // thickness regardless of projection.
            float hipKneeDist = std::sqrt((kneeS.x - coxaEndS.x)*(kneeS.x - coxaEndS.x) + (kneeS.y - coxaEndS.y)*(kneeS.y - coxaEndS.y));
            if (hipKneeDist > 0.1f) {
                sf::RectangleShape seg0(sf::Vector2f(hipKneeDist, resf * 0.12f));
                float seg0Angle = std::atan2(kneeS.y - coxaEndS.y, kneeS.x - coxaEndS.x) * 180.f / 3.14159f;
                seg0.setRotation(seg0Angle);
                seg0.setPosition(coxaEndS.x, coxaEndS.y);
                seg0.setFillColor(sf::Color::Yellow);
                window->draw(seg0);
            }

            float kneeAnkleDist = std::sqrt((ankleS.x - kneeS.x)*(ankleS.x - kneeS.x) + (ankleS.y - kneeS.y)*(ankleS.y - kneeS.y));
            if (kneeAnkleDist > 0.1f) {
                sf::RectangleShape seg1(sf::Vector2f(kneeAnkleDist, resf * 0.12f));
                float seg1Angle = std::atan2(ankleS.y - kneeS.y, ankleS.x - kneeS.x) * 180.f / 3.14159f;
                seg1.setRotation(seg1Angle);
                seg1.setPosition(kneeS.x, kneeS.y);
                seg1.setFillColor(sf::Color::Yellow);
                window->draw(seg1);
            }

            float ankleFootDist = std::sqrt((footS.x - ankleS.x)*(footS.x - ankleS.x) + (footS.y - ankleS.y)*(footS.y - ankleS.y));
            if (ankleFootDist > 0.1f) {
                sf::RectangleShape seg2(sf::Vector2f(ankleFootDist, resf * 0.12f));
                float seg2Angle = std::atan2(footS.y - ankleS.y, footS.x - ankleS.x) * 180.f / 3.14159f;
                seg2.setRotation(seg2Angle);
                seg2.setPosition(ankleS.x, ankleS.y);
                seg2.setFillColor(sf::Color::Yellow);
                window->draw(seg2);
            }

            // Draw visual joints and foot marker. Sizes are scaled by `resf` so that
            // visuals remain consistent as the zoom/res changes.
            float hjr = resf * 0.15f;
            sf::CircleShape hipJoint(hjr);
            hipJoint.setFillColor(sf::Color::Cyan);
            hipJoint.setOrigin(hjr, hjr);
            hipJoint.setPosition(coxaEndS.x, coxaEndS.y);
            window->draw(hipJoint);

            float kjr = resf * 0.15f;
            sf::CircleShape kneeJoint(kjr);
            kneeJoint.setFillColor(sf::Color::Cyan);
            kneeJoint.setOrigin(kjr, kjr);
            kneeJoint.setPosition(kneeS.x, kneeS.y);
            window->draw(kneeJoint);

            float ajr = resf * 0.15f;
            sf::CircleShape ankleJoint(ajr);
            ankleJoint.setFillColor(sf::Color::Cyan);
            ankleJoint.setOrigin(ajr, ajr);
            ankleJoint.setPosition(ankleS.x, ankleS.y);
            window->draw(ankleJoint);

            float fr = resf * 0.25f;
            sf::CircleShape footShape(fr);
            footShape.setFillColor(sf::Color::Magenta);
            footShape.setOrigin(fr, fr);
            footShape.setPosition(footS.x, footS.y);
            window->draw(footShape);
        }
    }

    for (size_t i=0;i<segments.size();++i) {
        sf::Vector2f pos = gridToIsoZ(segments[i].x, segments[i].y, bodyZ, resf, window);
        float radius = resf * 0.3f;
        sf::CircleShape joint(radius);
        joint.setFillColor(sf::Color::Blue);
        joint.setOrigin(radius, radius);
        joint.setPosition(pos.x, pos.y);
        window->draw(joint);
    }
}

} // namespace drawhelpers
