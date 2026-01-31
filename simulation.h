#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <SDL2/SDL.h>
#include "planet.h"
#include <memory>
#include <cmath>

class Simulation {
public:
    Simulation(int width, int height, int depth, int num_planets, double elasticity = 1.0);
    void spawnplanets();
    void applyGravity();
    void applyGravityUsingOctree();
    void verletStep(double dt);
    void resolvePixelCollisions();
    void render(SDL_Renderer* renderer);
    static int getOctreeNodeCount() { return s_octreeNodeCount; }
    void logMemoryStats() const;
    double getMaxSpeed() const;
    const std::vector<planet>& getplanets() const { return planets; }

private:
    int width, height, depth; // world dimensions (for spawning region)
    int num_planets;
    std::vector<planet> planets;
    double elasticity;

    const double G = 100.0; // Gravitational constant (tuned for visual effect)
    const double EPS = 1.0; // Softening factor

    // Barnes-Hut octree for gravity
    struct OctreeNode {
        static constexpr int MAX_DEPTH = 100;
        double centerX, centerY, centerZ; // center of this node (cube)
        double size; // width, height, depth of this node (cube)
        double totalMass;
        double centerOfMassX, centerOfMassY, centerOfMassZ;
        int planetIndex = -1; // if leaf with a single planet
        double planetX, planetY, planetZ, planetMass; // stored copy for subdivision
        std::unique_ptr<OctreeNode> children[8]; // octants

        OctreeNode(double cx, double cy, double cz, double sz);
        ~OctreeNode();
        void insert(int pIdx, double px, double py, double pz, double pmass, int depth = 0);
        void computeMassDistribution();
        void applyGravityToplanet(int pIdx, double px, double py, double pz, double pmass,
                                    double& ax, double& ay, double& az, double theta, double G, double EPS) const;
    };
    std::unique_ptr<OctreeNode> octreeRoot;
    const double theta = 0.5; // Barnes-Hut opening angle
    void buildOctree();

    static int s_octreeNodeCount;
};

#endif // SIMULATION_H
