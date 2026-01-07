#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <iostream>
#include <vector>
#include <ctime>

struct Voxel {
    // base offset inside segment (grid units)
    float baseOx, baseOy;
    // world position (grid units)
    float wx, wy;
    // velocity for dynamics (grid units per frame)
    float vx, vy;
    // filled flag
    uint8_t filled;
};

struct Segment {
    float x, y;
    // floating positions used for smooth rendering (in grid units)
    float px, py;
    // heading angle in radians
    float angle;
    sf::Color color;
    // voxel grid for this segment (width x height)
    int voxW, voxH;
    std::vector<Voxel> voxels;
    // whether this segment moved during the last applied move (used to gate dragging)
    bool moved;
    // simple leg pairs for locomotion
    struct Leg {
        // hip position relative to segment center (grid units)
        float hipOx, hipOy;
        // side: -1 = left, +1 = right
        int side;
        // 3-segment leg: hip angle, knee angle, foot angle (radians)
        float hipAngle, kneeAngle, footAngle;
        float targetHipAngle, targetKneeAngle, targetFootAngle;
        // phase offset along body to create metachronal wave
        float phaseOffset;
        // cycle accumulator
        float cycle;
        // segment lengths for 3-part leg
        float hipLength, kneeLength, footLength;
        float pushStrength;
        // contact state
        bool onGround;
    };
    std::vector<Leg> legs;
};

class Centipede {
    private:
        std::vector<Segment> segments;
        int dirX, dirY;
        int moveCounter;
        static const int moveDelay = 2;  // Update every 2 frames (faster, for smoother small steps)
        static constexpr float followSpeed = 0.28f; // interpolation factor for dragging effect
        static constexpr float maxMovePerTry = 1.2f; // clamp large mouse jumps
        // gait time accumulator for global wave-based gait
        float gaitTime;
        // track previous head position to match gait speed to body movement
        float lastHeadX, lastHeadY;
        
    public:
        Centipede(int startX, int startY, int length);
        void update();
        // Try to move by (dx,dy) respecting internal move delay (floats)
        void tryMove(float dx, float dy);
        // Immediate move without delay (insert head, pop tail) (floats)
        void moveBy(float dx, float dy);
        void render(sf::RenderWindow* window, float resf);
        const std::vector<Segment>& getSegments() const;
};

class Game {
    private:
        sf::RenderWindow* window;
        sf::VideoMode vMode;
        sf::Event ev;

        static const int height = 800, width = 800;
        static const int res = 10;
        static const int col = height / res, row = width / res;

        std::vector<sf::RectangleShape> rects;
        Centipede* centipede;

        bool mouseClicked;
        float zoom;

        // private functions
        void initVar();
        void initWindow();

    public:
        // CONSTRUCTOR AND DESTRUCTOR
        Game();
        virtual ~Game();

        // Functions
        bool getWinIsOpen();
        void update();
        void render();
};