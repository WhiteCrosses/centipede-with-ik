#include "Game.hpp"
#include <iostream>
#include <cmath>
#include <SFML/Window.hpp>

#include "render/Projection.hpp"
#include "input/Camera.hpp"

void Game::initVar() {
    this->window = nullptr;
    this->mouseClicked = false;
    this->zoom = 1.0f;
    this->middleDragging = false;
    this->middleLastMouse = sf::Vector2i(0, 0);
    this->hasMoveTarget = false;
    this->moveTargetGrid = sf::Vector2f(0.f, 0.f);
}

void Game::initWindow() {
    this->vMode.height = height; this->vMode.width = width;
    this->window = new sf::RenderWindow(vMode, "Centipede Game", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);
}

Game::Game() { initVar(); initWindow(); centipede = new Centipede(40,10,14); }
Game::~Game() { delete window; delete centipede; }
bool Game::getWinIsOpen() { return window->isOpen(); }

void Game::update() {
    while (window->pollEvent(ev)) {
        if (ev.type == ev.Closed) window->close();
        if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left) mouseClicked = true;
        if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Left) mouseClicked = false;

        // Right-click: set a destination on the floor (grid space)
        if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Right) {
            sf::Vector2i mpos = sf::Mouse::getPosition(*window);
            float resf = static_cast<float>(res) * this->zoom;
            this->moveTargetGrid = screenToGrid(static_cast<float>(mpos.x), static_cast<float>(mpos.y), resf, window);
            this->hasMoveTarget = true;
        }

        // Camera pan/zoom handled by camera module
        input::handleCameraEvent(ev, window, this->zoom, this->middleDragging, this->middleLastMouse);
    }

    float step = 4.0f * 0.1f;
    float dx=0.f, dy=0.f;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) dx = -step;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) dx = step;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) dy = -step;
    else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) dy = step;

    bool mouseHeld = sf::Mouse::isButtonPressed(sf::Mouse::Left);
    if (mouseHeld) {
        sf::Vector2i mpos = sf::Mouse::getPosition(*window);
        float resf = static_cast<float>(res) * this->zoom;
        sf::Vector2f gridTarget = screenToGrid(static_cast<float>(mpos.x), static_cast<float>(mpos.y), resf, window);
        const auto &head = centipede->getSegments()[0];
        float hx = head.px, hy = head.py; float dirx = gridTarget.x - hx, diry = gridTarget.y - hy;
        float len = std::sqrt(dirx*dirx + diry*diry);
        if (len > 0.001f) { dirx/=len; diry/=len; float speedMult=3.5f; centipede->tryMove(dirx*step*speedMult, diry*step*speedMult); }
    } else {
        // If a RMB destination exists, walk toward it; otherwise fall back to keyboard.
        if (this->hasMoveTarget) {
            const auto &head = centipede->getSegments()[0];
            float hx = head.px, hy = head.py;
            float dirx = this->moveTargetGrid.x - hx;
            float diry = this->moveTargetGrid.y - hy;
            float len = std::sqrt(dirx*dirx + diry*diry);
            if (len < 0.6f) {
                this->hasMoveTarget = false;
            } else if (len > 0.001f) {
                dirx /= len;
                diry /= len;
                float speedMult = 3.0f;
                centipede->tryMove(dirx * step * speedMult, diry * step * speedMult);
            }
        } else {
            if (dx!=0.f || dy!=0.f) centipede->tryMove(dx,dy);
        }
    }

    centipede->update();
}

void Game::render() {
    window->clear(sf::Color::Black);
    
    float resf = static_cast<float>(res) * this->zoom;
    centipede->render(window, resf);

    // Draw right-click destination marker (pink circle on the floor)
    if (this->hasMoveTarget) {
        sf::Vector2f pos = gridToIso(this->moveTargetGrid.x, this->moveTargetGrid.y, resf, window);
        float radius = std::max(3.0f, resf * 0.25f);
        sf::CircleShape marker(radius);
        marker.setOrigin(radius, radius);
        marker.setPosition(pos);
        marker.setFillColor(sf::Color(255, 105, 180));
        window->draw(marker);
    }

    window->display();
}
