#include "Projection.hpp"
#include <cmath>

// This module converts between grid coordinates (gx, gy, z) and isometric
// screen coordinates (pixels). The mapping treats each grid cell as an
// isometric diamond whose screen-space extents depend on `resf` (tile size).

// Conventions used here:
// - `resf` is the tile size in pixels at the current zoom (res * zoom).
// - halfW = resf * 0.5 -> horizontal half-size of a diamond tile.
// - halfH = resf * 0.25 -> vertical half-size of a diamond tile.
// - `input::g_camOffX` / `input::g_camOffY` are camera offsets in pixels.
// - The screen origin (0,0) is top-left; `cx, cy` are the camera-centered offsets
//   used to position the world on screen.

// Projection formulas:
//   sx = (gx - gy) * halfW + cx
//   sy = (gx + gy) * halfH - z * resf * 0.3f + cy
// where:
//   - gx, gy are fractional grid coordinates (tile indices may be non-integer)
//   - z is height/elevation in tile-units
// The z-term subtracts vertical pixels to raise objects visually; the 0.3 factor
// is an empirical scale chosen so vertical extrusion looks visually pleasing
// relative to the tile size.

// Inverse mapping (screen -> grid) ignores z (we cannot recover z from 2D screen
// without additional info). Let A = sx - cx, B = sy - cy (screen coords relative to
// camera center). From the two linear equations above (ignoring z):
//   (1) A = (gx - gy) * halfW
//   (2) B = (gx + gy) * halfH
// Solve the 2x2 system:
//   gx = ( (A/halfW) + (B/halfH) ) / 2
//   gy = ( (B/halfH) - (A/halfW) ) / 2


sf::Vector2f gridToIsoZ(float gx, float gy, float z, float resf, sf::RenderWindow* window) {
    float halfW = resf * 0.5f;
    float halfH = resf * 0.25f;
    // Camera center in pixels: horizontally centered in window plus camera X offset;
    // vertically offset by a constant (50 pixels) plus camera Y offset.
    float cx = window->getSize().x * 0.5f + input::g_camOffX;
    float cy = 50.0f + input::g_camOffY;

    // Map grid coordinates to isometric screen space.
    float sx = (gx - gy) * halfW + cx;
    // The z term reduces the screen Y to give the illusion of height:
    // sy = (gx + gy) * halfH - (z * resf * 0.3) + cy
    float sy = (gx + gy) * halfH - z * resf * 0.3f + cy;
    return sf::Vector2f(sx, sy);
}


sf::Vector2f gridToIso(float gx, float gy, float resf, sf::RenderWindow* window) {
    return gridToIsoZ(gx, gy, 0.0f, resf, window);
}


sf::Vector2f screenToGrid(float sx, float sy, float resf, sf::RenderWindow* window) {
    float halfW = resf * 0.5f;
    float halfH = resf * 0.25f;
    float cx = window->getSize().x * 0.5f + input::g_camOffX;
    float cy = 50.0f + input::g_camOffY;

    float A = sx - cx; // equals (gx - gy) * halfW
    float B = sy - cy; // equals (gx + gy) * halfH (+/- z term ignored)

    // Solve the linear system (see notes above):
    // gx = (A/halfW + B/halfH) / 2
    // gy = (B/halfH - A/halfW) / 2
    float gx = (A / halfW + B / halfH) * 0.5f;
    float gy = (B / halfH - A / halfW) * 0.5f;
    return sf::Vector2f(gx, gy);
}
