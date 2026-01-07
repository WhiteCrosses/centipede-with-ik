#include "GridRenderer.hpp"
#include "Projection.hpp"
#include <cmath>

void drawGrid(sf::RenderWindow* window, float resf) {
    const auto sz = window->getSize();
    sf::Vector2f g00 = screenToGrid(0.0f, 0.0f, resf, window);
    sf::Vector2f g10 = screenToGrid(static_cast<float>(sz.x), 0.0f, resf, window);
    sf::Vector2f g01 = screenToGrid(0.0f, static_cast<float>(sz.y), resf, window);
    sf::Vector2f g11 = screenToGrid(static_cast<float>(sz.x), static_cast<float>(sz.y), resf, window);

    float minGX = std::min(std::min(g00.x, g10.x), std::min(g01.x, g11.x));
    float maxGX = std::max(std::max(g00.x, g10.x), std::max(g01.x, g11.x));
    float minGY = std::min(std::min(g00.y, g10.y), std::min(g01.y, g11.y));
    float maxGY = std::max(std::max(g00.y, g10.y), std::max(g01.y, g11.y));

    const float gridMargin = 6.0f; // extra grid units beyond the visible area
    int gx0 = static_cast<int>(std::floor(minGX - gridMargin));
    int gx1 = static_cast<int>(std::ceil (maxGX + gridMargin));
    int gy0 = static_cast<int>(std::floor(minGY - gridMargin));
    int gy1 = static_cast<int>(std::ceil (maxGY + gridMargin));

    sf::Color gridColor(120, 120, 120, 150);
    const float thicknessPx = 2.0f;
    const int gridStep = 1; // draw every `gridStep`th line

    auto drawLine = [&](const sf::Vector2f& p1, const sf::Vector2f& p2) {
        float lineLen = std::sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
        if (lineLen <= 0.1f) return;
        sf::RectangleShape line(sf::Vector2f(lineLen, thicknessPx));
        float angle = std::atan2(p2.y - p1.y, p2.x - p1.x) * 180.f / 3.14159265f;
        line.setRotation(angle);
        line.setPosition(p1);
        line.setFillColor(gridColor);
        window->draw(line);
    };

    // Constant gx lines.
    for (int gx = gx0; gx <= gx1; gx += gridStep) {
        sf::Vector2f p1 = gridToIso(static_cast<float>(gx), static_cast<float>(gy0), resf, window);
        sf::Vector2f p2 = gridToIso(static_cast<float>(gx), static_cast<float>(gy1), resf, window);
        drawLine(p1, p2);
    }

    // Constant gy lines.
    for (int gy = gy0; gy <= gy1; gy += gridStep) {
        sf::Vector2f p1 = gridToIso(static_cast<float>(gx0), static_cast<float>(gy), resf, window);
        sf::Vector2f p2 = gridToIso(static_cast<float>(gx1), static_cast<float>(gy), resf, window);
        drawLine(p1, p2);
    }
}
