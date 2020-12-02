#pragma once
struct Vec2i {
public:
    Vec2i() { x = 0, y = 0; };
    Vec2i(int x, int y) { this->x = x; this->y = y; };
    int x, y;
};

struct Vec2 {
public:
    Vec2() { x = 0, y = 0; };
    Vec2(float x, float y) { this->x = x; this->y = y; };

    float x, y;
};
