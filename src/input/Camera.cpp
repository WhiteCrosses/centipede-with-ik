#include "Camera.hpp"
#include "../render/Projection.hpp"

namespace input {

float g_camOffX = 0.0f;
float g_camOffY = 0.0f;

void handleCameraEvent(const sf::Event &ev, sf::RenderWindow* window, float &zoom, bool &middleDragging, sf::Vector2i &middleLastMouse) {
    if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Middle) {
        middleDragging = true;
        middleLastMouse = sf::Mouse::getPosition(*window);
    }
    if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Middle) {
        middleDragging = false;
    }
    if (ev.type == sf::Event::MouseMoved && middleDragging) {
        sf::Vector2i now = sf::Mouse::getPosition(*window);
        sf::Vector2i delta = now - middleLastMouse;
        g_camOffX += static_cast<float>(delta.x);
        g_camOffY += static_cast<float>(delta.y);
        middleLastMouse = now;
    }

    if (ev.type == sf::Event::MouseWheelScrolled) {
        if (ev.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel) {
            float delta = ev.mouseWheelScroll.delta;
            sf::Vector2i mpos = sf::Mouse::getPosition(*window);
            // caller's resf is res * zoom; compute before/after similarly to previous logic
            const float res = 32.0f; // keep same nominal grid base as original code expects; caller uses same `res`
            float resfBefore = res * zoom;
            sf::Vector2f gridBefore = screenToGrid(static_cast<float>(mpos.x), static_cast<float>(mpos.y), resfBefore, window);

            const float zoomStep = 1.05f;
            if (delta > 0) zoom *= zoomStep; else if (delta < 0) zoom /= zoomStep;
            if (zoom < 0.2f) zoom = 0.2f; if (zoom > 4.0f) zoom = 4.0f;
            float resfAfter = res * zoom;

            float halfW = resfAfter * 0.5f;
            float halfH = resfAfter * 0.25f;
            float cxNeeded = static_cast<float>(mpos.x) - (gridBefore.x - gridBefore.y) * halfW;
            float cyNeeded = static_cast<float>(mpos.y) - (gridBefore.x + gridBefore.y) * halfH;
            g_camOffX = cxNeeded - (window->getSize().x * 0.5f);
            g_camOffY = cyNeeded - 50.0f;
        }
    }
}

} // namespace input
