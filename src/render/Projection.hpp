#pragma once

#include <SFML/Graphics.hpp>

// Camera offsets are provided by the input module. They represent pixel
// translations applied to the world before projection:
//   - input::g_camOffX : horizontal camera offset in pixels
//   - input::g_camOffY : vertical camera offset in pixels
// These are declared here as externs in the `input` namespace; the
// corresponding definitions live in `src/input/Camera.cpp`.
namespace input { extern float g_camOffX; extern float g_camOffY; }


// gridToIsoZ
//  - Purpose: convert fractional grid coordinates (gx, gy) and elevation z
//    into screen-space pixel coordinates for isometric rendering.
//  - Parameters:
//      gx, gy : fractional grid coordinates (tile-space; may be non-integer)
//      z      : elevation in tile units (adds vertical extrusion)
//      resf   : tile size in pixels at current zoom (base `res * zoom`)
//      window : pointer to the render window (used to compute camera center)
//  - Returns: sf::Vector2f(sx, sy) in screen pixels.
//  - Notes: this function reads `input::g_camOffX/Y` to apply camera translation.
sf::Vector2f gridToIsoZ(float gx, float gy, float z, float resf, sf::RenderWindow* window);


// gridToIso
//  - Convenience wrapper for gridToIsoZ with z==0 (ground-level projection).
//  - Parameters and return value as above; performance is trivial.
sf::Vector2f gridToIso(float gx, float gy, float resf, sf::RenderWindow* window);


// screenToGrid
//  - Purpose: inverse of the (gx,gy) portion of the isometric projection.
//  - Parameters:
//      sx, sy : screen pixel coordinates
//      resf   : tile size in pixels at current zoom (base `res * zoom`)
//      window : pointer to the render window (used to compute camera center)
//  - Returns: fractional grid coordinates (gx, gy) corresponding to the
//    provided screen point. NOTE: z cannot be recovered from a single
//    2D projection, so this mapping ignores elevation.
//  - Use case: picking (mouse -> grid cell), culling (determine visible grid
//    rectangle), etc.
sf::Vector2f screenToGrid(float sx, float sy, float resf, sf::RenderWindow* window);
