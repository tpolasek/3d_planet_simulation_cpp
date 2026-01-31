#include "simulation.h"
#include "utility.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>

int Simulation::s_octreeNodeCount = 0;

Simulation::Simulation(int width, int height, int depth, int num_planets, double elasticity)
    : width(width), height(height), depth(depth), num_planets(num_planets), elasticity(elasticity) {
    spawnplanets();
}

void Simulation::spawnplanets() {
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Spawn within a 200x200x200 box at the center
    const double margin = 100.0;
    std::uniform_real_distribution<> dis_x(margin, width - margin);
    std::uniform_real_distribution<> dis_y(margin, height - margin);
    std::uniform_real_distribution<> dis_z(margin, depth - margin);
    std::uniform_real_distribution<> dis_v(-0.5, 0.5);
    std::uniform_real_distribution<> dis_radius(1, 10);

    double centerX = width * 0.5;
    double centerY = height * 0.5;
    double centerZ = depth * 0.5;

    for (int i = 0; i < num_planets; ++i) {
        double x = dis_x(gen);
        double y = dis_y(gen);
        double z = dis_z(gen);

        double dx = x - centerX;
        double dy = y - centerY;
        double dz = z - centerZ;
        double dist_sq = dx*dx + dy*dy + dz*dz;
        double vx, vy, vz;
        if (dist_sq > 1e-12) {
            // Tangential velocity around Z axis (clockwise when viewed from above)
            double tx = dy;
            double ty = -dx;
            double tz = 0.0;
            double inv_len = 1.0 / std::sqrt(dist_sq);
            double speed = 0.3; // constant tangential speed
            vx = tx * inv_len * speed;
            vy = ty * inv_len * speed;
            vz = tz * inv_len * speed;
        } else {
            // planet at center: give random direction
            vx = dis_v(gen);
            vy = dis_v(gen);
            vz = dis_v(gen);
        }

        double radius = dis_radius(gen);
        planets.push_back({
            x, y, z,
            x - vx, y - vy, z - vz, // prev_x, prev_y, prev_z for initial velocity
            0.0, 0.0, 0.0, // ax, ay, az
            radius*radius, // mass (proportional to radius^2)
            radius, // radius
            255, 0, 0 // color (red)
        });
    }
}

double FastInvSqrt(double x) {
    double xhalf = 0.5 * x;
    long long i = *(long long*)&x;         // Treat bits as integer
    i = 0x5fe6eb50c7b537a9LL - (i >> 1);   // The magic constant for double
    x = *(double*)&i;                      // Treat bits back as double
    x = x * (1.5 - xhalf * x * x);         // Newton-Raphson iteration
    // x = x * (1.5 - xhalf * x * x);      // Optional: second iteration
    return x;
}

void Simulation::applyGravity() {
    const size_t n = planets.size();
    const double G_local = G;
    const double EPS_local = EPS;

    // Zero accelerations
    for (size_t i = 0; i < n; ++i) {
        planet& pi = planets[i];
        pi.ax = 0.0;
        pi.ay = 0.0;
        pi.az = 0.0;
    }

    // Pairwise gravitational interaction
    for (size_t i = 0; i < n; ++i) {
        planet& pi = planets[i];
        double xi = pi.x, yi = pi.y, zi = pi.z;
        double mi = pi.mass;
        for (size_t j = i + 1; j < n; ++j) {
            planet& pj = planets[j];
            double dx = pj.x - xi;
            double dy = pj.y - yi;
            double dz = pj.z - zi;
            double dist_sq = dx*dx + dy*dy + dz*dz + EPS_local*EPS_local;
            double inv_dist = 1.0 / std::sqrt(dist_sq);
            double inv_dist_cubed = inv_dist * inv_dist * inv_dist;
            double force_mag = G_local * mi * pj.mass * inv_dist_cubed;
            double fx = force_mag * dx;
            double fy = force_mag * dy;
            double fz = force_mag * dz;
            pi.ax += fx / mi;
            pi.ay += fy / mi;
            pi.az += fz / mi;
            pj.ax -= fx / pj.mass;
            pj.ay -= fy / pj.mass;
            pj.az -= fz / pj.mass;
        }
    }
}

void Simulation::verletStep(double dt) {
    const size_t n = planets.size();
    const double dt2 = dt * dt;

    for (size_t i = 0; i < n; ++i) {
        planet& p = planets[i];
        double temp_x = p.x;
        double temp_y = p.y;
        double temp_z = p.z;

        double new_x = 2.0 * p.x - p.prev_x + p.ax * dt2;
        double new_y = 2.0 * p.y - p.prev_y + p.ay * dt2;
        double new_z = 2.0 * p.z - p.prev_z + p.az * dt2;

        // Validate finite positions; if invalid, revert to previous position and zero velocity
        if (!std::isfinite(new_x) || !std::isfinite(new_y) || !std::isfinite(new_z)) {
            new_x = p.x;
            new_y = p.y;
            new_z = p.z;
            p.prev_x = p.x;
            p.prev_y = p.y;
            p.prev_z = p.z;
        }

        p.x = new_x;
        p.y = new_y;
        p.z = new_z;

        p.prev_x = temp_x;
        p.prev_y = temp_y;
        p.prev_z = temp_z;

        p.ax = 0.0;
        p.ay = 0.0;
        p.az = 0.0;
    }
}

void Simulation::resolvePixelCollisions() {
    const size_t n = planets.size();
    const double e = elasticity;

    for (size_t i = 0; i < n; ++i) {
        planet& pi = planets[i];
        double xi = pi.x, yi = pi.y, zi = pi.z;
        double ri = pi.radius;
        for (size_t j = i + 1; j < n; ++j) {
            planet& pj = planets[j];
            double dx = pj.x - xi;
            double dy = pj.y - yi;
            double dz = pj.z - zi;
            double dist_sq = dx*dx + dy*dy + dz*dz;
            double rsum = ri + pj.radius;
            if (dist_sq >= rsum*rsum || dist_sq < 1e-24) continue;

            double inv_dist = 1.0 / std::sqrt(dist_sq);
            double nx = dx * inv_dist;
            double ny = dy * inv_dist;
            double nz = dz * inv_dist;

            // Velocities (Verlet)
            double v1x = xi - pi.prev_x;
            double v1y = yi - pi.prev_y;
            double v1z = zi - pi.prev_z;
            double v2x = pj.x - pj.prev_x;
            double v2y = pj.y - pj.prev_y;
            double v2z = pj.z - pj.prev_z;

            // Normal velocities
            double v1n = v1x * nx + v1y * ny + v1z * nz;
            double v2n = v2x * nx + v2y * ny + v2z * nz;

            // Masses
            double m1 = pi.mass;
            double m2 = pj.mass;
            double total_mass = m1 + m2;
            double inv_total_mass = 1.0 / total_mass;

            // Elastic collision response
            double v1n_after = ((m1 - e * m2) * v1n + (1.0 + e) * m2 * v2n) * inv_total_mass;
            double v2n_after = ((m2 - e * m1) * v2n + (1.0 + e) * m1 * v1n) * inv_total_mass;

            double dv1n = v1n_after - v1n;
            double dv2n = v2n_after - v2n;

            // Update previous positions (which encode velocity)
            pi.prev_x = xi - (v1x + dv1n * nx);
            pi.prev_y = yi - (v1y + dv1n * ny);
            pi.prev_z = zi - (v1z + dv1n * nz);
            pj.prev_x = pj.x - (v2x + dv2n * nx);
            pj.prev_y = pj.y - (v2y + dv2n * ny);
            pj.prev_z = pj.z - (v2z + dv2n * nz);

            // Separate positions by half the overlap along normal
            double dist = std::sqrt(dist_sq);
            double overlap = rsum - dist;
            double separation = overlap * 0.5;
            double sx = separation * nx;
            double sy = separation * ny;
            double sz = separation * nz;

            pi.x = xi - sx;
            pi.y = yi - sy;
            pi.z = zi - sz;
            pi.prev_x -= sx;
            pi.prev_y -= sy;
            pi.prev_z -= sz;

            pj.x = pj.x + sx;
            pj.y = pj.y + sy;
            pj.z = pj.z + sz;
            pj.prev_x += sx;
            pj.prev_y += sy;
            pj.prev_z += sz;
        }
    }
}

void Simulation::render(SDL_Renderer* renderer) {
    const double maxSpeed = 1.0; // expected maximum speed for hue scaling
    const double saturation = 1.0;
    const double value = 1.0;
    for (const auto& p : planets) {
        double vx = p.x - p.prev_x;
        double vy = p.y - p.prev_y;
        double speed = std::sqrt(vx*vx + vy*vy);
        double hue = (speed / maxSpeed) * 360.0;
        // clamp hue to [0,360)
        if (hue < 0.0) hue = 0.0;
        if (hue >= 360.0) hue = 359.999;
        uint8_t r, g, b;
        HSVtoRGB(hue, saturation, value, r, g, b);
        SDL_SetRenderDrawColor(renderer, r, g, b, 255);
        SDL_RenderDrawPoint(renderer, (int)std::round(p.x), (int)std::round(p.y));
    }
}

// OctreeNode definitions (stubs)
Simulation::OctreeNode::OctreeNode(double cx, double cy, double cz, double sz)
    : centerX(cx), centerY(cy), centerZ(cz), size(sz), totalMass(0.0),
      centerOfMassX(0.0), centerOfMassY(0.0), centerOfMassZ(0.0),
      planetIndex(-1), planetX(0.0), planetY(0.0), planetZ(0.0), planetMass(0.0) {
    Simulation::s_octreeNodeCount++;
    for (int i = 0; i < 8; ++i) children[i].reset();
}

Simulation::OctreeNode::~OctreeNode() {
    Simulation::s_octreeNodeCount--;
}

void Simulation::OctreeNode::insert(int pIdx, double px, double py, double pz, double pmass, int depth) {
    if (depth >= MAX_DEPTH) {
        return; // safety
    }

    // If this node is empty (no planet, no children)
    if (planetIndex == -1 && !children[0]) {
        planetIndex = pIdx;
        planetX = px;
        planetY = py;
        planetZ = pz;
        planetMass = pmass;
        return;
    }

    // If this node contains a planet and has no children, we need to subdivide
    if (planetIndex != -1 && !children[0]) {
        double childSize = size * 0.5;
        double quarter = size * 0.25;
        // Create all eight children
        for (int i = 0; i < 8; ++i) {
            double cx = centerX + ((i & 1) ? quarter : -quarter);
            double cy = centerY + ((i & 2) ? quarter : -quarter);
            double cz = centerZ + ((i & 4) ? quarter : -quarter);
            children[i] = std::make_unique<OctreeNode>(cx, cy, cz, childSize);
        }
        // Re-insert the existing planet
        int existingIdx = planetIndex;
        double ex = planetX, ey = planetY, ez = planetZ, emass = planetMass;
        planetIndex = -1; // clear leaf data
        int octant = ((ex >= centerX) ? 1 : 0) |
                     ((ey >= centerY) ? 2 : 0) |
                     ((ez >= centerZ) ? 4 : 0);
        children[octant]->insert(existingIdx, ex, ey, ez, emass, depth + 1);
        // Now insert the new planet
        int newOctant = ((px >= centerX) ? 1 : 0) |
                        ((py >= centerY) ? 2 : 0) |
                        ((pz >= centerZ) ? 4 : 0);
        children[newOctant]->insert(pIdx, px, py, pz, pmass, depth + 1);
        return;
    }

    // At this point, the node must have children (internal node)
    int octant = ((px >= centerX) ? 1 : 0) |
                 ((py >= centerY) ? 2 : 0) |
                 ((pz >= centerZ) ? 4 : 0);
    children[octant]->insert(pIdx, px, py, pz, pmass, depth + 1);
}

void Simulation::OctreeNode::computeMassDistribution() {
    // If this node is a leaf containing a planet
    if (planetIndex != -1) {
        totalMass = planetMass;
        centerOfMassX = planetX;
        centerOfMassY = planetY;
        centerOfMassZ = planetZ;
        return;
    }

    // Internal node or empty node
    double total = 0.0;
    double comX = 0.0, comY = 0.0, comZ = 0.0;
    for (int i = 0; i < 8; ++i) {
        if (children[i]) {
            children[i]->computeMassDistribution();
            double childMass = children[i]->totalMass;
            if (childMass > 0.0) {
                total += childMass;
                comX += children[i]->centerOfMassX * childMass;
                comY += children[i]->centerOfMassY * childMass;
                comZ += children[i]->centerOfMassZ * childMass;
            }
        }
    }
    if (total > 0.0) {
        totalMass = total;
        centerOfMassX = comX / total;
        centerOfMassY = comY / total;
        centerOfMassZ = comZ / total;
    } else {
        totalMass = 0.0;
        centerOfMassX = centerX;
        centerOfMassY = centerY;
        centerOfMassZ = centerZ;
    }
}

void Simulation::OctreeNode::applyGravityToplanet(int pIdx, double px, double py, double pz, double pmass,
                                                    double& ax, double& ay, double& az, double theta, double G, double EPS) const {
    // Empty node, nothing to contribute
    if (totalMass == 0.0) return;

    // If this node is a leaf containing the same planet, skip self-interaction
    if (planetIndex == pIdx) return;

    double dx = centerOfMassX - px;
    double dy = centerOfMassY - py;
    double dz = centerOfMassZ - pz;
    double dist_sq = dx*dx + dy*dy + dz*dz;
    double dist = std::sqrt(dist_sq);

    // Avoid division by zero (should not happen for distinct particles)
    if (dist == 0.0) return;

    // Determine if we can approximate this node as a single mass
    if (planetIndex != -1 || (size / dist < theta)) {
        // Compute softened distance
        double softened = dist_sq + EPS*EPS;
        double inv_dist = 1.0 / std::sqrt(softened);
        double inv_dist_cubed = inv_dist * inv_dist * inv_dist;
        double force_mag = G * totalMass * inv_dist_cubed;
        ax += force_mag * dx;
        ay += force_mag * dy;
        az += force_mag * dz;
    } else {
        // Otherwise, recurse into children
        for (int i = 0; i < 8; ++i) {
            if (children[i]) {
                children[i]->applyGravityToplanet(pIdx, px, py, pz, pmass, ax, ay, az, theta, G, EPS);
            }
        }
    }
}

void Simulation::buildOctree() {
    // Determine root cube that encloses the whole simulation space
    double rootSize = static_cast<double>(std::max({width, height, depth}));
    double centerX = width * 0.5;
    double centerY = height * 0.5;
    double centerZ = depth * 0.5;
    
    octreeRoot = std::make_unique<OctreeNode>(centerX, centerY, centerZ, rootSize);
    
    // Insert all planets
    for (size_t i = 0; i < planets.size(); ++i) {
        const planet& p = planets[i];
        octreeRoot->insert(static_cast<int>(i), p.x, p.y, p.z, p.mass, 0);
    }
    
    // Compute mass distribution for the whole tree
    octreeRoot->computeMassDistribution();
}

void Simulation::applyGravityUsingOctree() {
    // Zero accelerations
    for (auto& p : planets) {
        p.ax = 0.0;
        p.ay = 0.0;
        p.az = 0.0;
    }

    // Build octree from current positions
    buildOctree();

    // Compute gravity for each planet using the octree
    for (size_t i = 0; i < planets.size(); ++i) {
        planet& p = planets[i];
        octreeRoot->applyGravityToplanet(static_cast<int>(i),
                                         p.x, p.y, p.z, p.mass,
                                         p.ax, p.ay, p.az,
                                         theta, G, EPS);
    }
}

void Simulation::logMemoryStats() const {
    size_t planets_capacity = planets.capacity() * sizeof(planet);
    std::cerr << "Memory stats: "
              << "planets capacity=" << planets_capacity << "B"
              << std::endl;
}

double Simulation::getMaxSpeed() const {
    double maxSpeedSq = 0.0;
    for (const auto& p : planets) {
        double vx = p.x - p.prev_x;
        double vy = p.y - p.prev_y;
        double vz = p.z - p.prev_z;
        double speedSq = vx*vx + vy*vy + vz*vz;
        if (speedSq > maxSpeedSq) maxSpeedSq = speedSq;
    }
    return std::sqrt(maxSpeedSq);
}
