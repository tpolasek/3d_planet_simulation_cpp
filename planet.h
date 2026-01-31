#ifndef PLANET_H
#define PLANET_H

#include <cstdint>

struct planet {
    double x, y, z;               // current position
    double prev_x, prev_y, prev_z;// previous position (for Verlet)
    double ax, ay, az;            // accumulated acceleration
    double mass;                  // mass
    double radius;                // radius for sphere rendering (derived from mass)
    uint8_t r, g, b;              // color
};

#endif // PLANET_H
