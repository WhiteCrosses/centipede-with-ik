#pragma once

#include <SFML/Graphics.hpp>
#include "Centipede.hpp"

class Game {
private:
    sf::RenderWindow* window;
    sf::VideoMode vMode;
    sf::Event ev;
    static const int height = 800, width = 800;
    static const int res = 10;
    std::vector<sf::RectangleShape> rects;
    Centipede* centipede;
    bool mouseClicked;
    float zoom;
    bool middleDragging;
    sf::Vector2i middleLastMouse;

    // Right-click destination marker (grid space)
    bool hasMoveTarget;
    sf::Vector2f moveTargetGrid;
    void initVar();
    void initWindow();
public:
    Game();
    virtual ~Game();
    bool getWinIsOpen();
    void update();
    void render();
};
