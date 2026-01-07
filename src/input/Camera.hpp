#pragma once

#include <SFML/Graphics.hpp>

namespace input {
    extern float g_camOffX;
    extern float g_camOffY;

    // Handle camera-related events: middle-button drag and wheel zoom anchoring.
    void handleCameraEvent(const sf::Event &ev, sf::RenderWindow* window, float &zoom, bool &middleDragging, sf::Vector2i &middleLastMouse);
}
