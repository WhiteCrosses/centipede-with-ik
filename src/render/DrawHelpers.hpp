#pragma once

#include "../../include/Centipede.hpp"
#include "Projection.hpp"
#include <SFML/Graphics.hpp>

namespace drawhelpers {
    // Draw articulated legs and spine joints for the given segments.
    void drawCentipede(sf::RenderWindow* window, const std::vector<Segment> &segments, float resf, float bodyZ);
}
