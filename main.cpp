#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <cmath>
#include "simulation.h"
#include "utility.h"

// OpenGL headers (legacy)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

// Point distance attenuation constants
#ifndef GL_POINT_DISTANCE_ATTENUATION
#define GL_POINT_DISTANCE_ATTENUATION 0x8129
#endif
#ifndef GL_POINT_SIZE_MIN
#define GL_POINT_SIZE_MIN 0x8126
#endif
#ifndef GL_POINT_SIZE_MAX
#define GL_POINT_SIZE_MAX 0x8127
#endif
extern "C" {
void glPointParameterf(GLenum pname, GLfloat param);
void glPointParameterfv(GLenum pname, const GLfloat *params);
}

// Camera variables
float cameraDistance = 10.0f;
float cameraAngleX = 0.0f;
float cameraAngleY = 0.0f;
float cameraZoom = 1000.0f;
bool mouseCaptured = false;
int lastMouseX = 0, lastMouseY = 0;
GLUquadric* sphereQuadric = nullptr;

void handleMouseMotion(int dx, int dy) {
    cameraAngleY += dx * 0.2f;
    cameraAngleX += dy * 0.2f;
    if (cameraAngleX > 89.0f) cameraAngleX = 89.0f;
    if (cameraAngleX < -89.0f) cameraAngleX = -89.0f;
}

void handleMouseScroll(int yoffset) {
    cameraDistance -= yoffset*10.0;
    if (cameraDistance < 1.0f) cameraDistance = 1.0f;
}

int main(int argc, char* argv[]) {
    int num_planets = 8000; // default

    if (argc > 1) {
        char* endptr;
        long n = std::strtol(argv[1], &endptr, 10);
        if (endptr == argv[1] || n <= 0) {
            std::cerr << "Usage: " << argv[0] << " <N>\n";
            std::cerr << "  N: number of planets (1-500)\n";
            return 1;
        }
        num_planets = static_cast<int>(n);
    }

    const int WIDTH = 1000;
    const int HEIGHT = 1000;
    const int DEPTH = 1000;
    const double DT = 0.016; // ~60 FPS

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    if (TTF_Init() != 0) {
        std::cerr << "TTF_Init failed: " << TTF_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    // Use OpenGL (legacy)
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow(
        "3D Gravity Simulation",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WIDTH,
        HEIGHT,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN
    );

    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    if (!glContext) {
        std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Enable depth test and create sphere quadric
    glEnable(GL_DEPTH_TEST);
    sphereQuadric = gluNewQuadric();
    gluQuadricNormals(sphereQuadric, GLU_SMOOTH);
    gluQuadricOrientation(sphereQuadric, GLU_OUTSIDE);
    gluQuadricDrawStyle(sphereQuadric, GLU_FILL);

    Simulation sim(WIDTH, HEIGHT, DEPTH, num_planets, 0.5);

    // Load font for FPS display (still using SDL_ttf for overlay)
    TTF_Font* font = TTF_OpenFont("/System/Library/Fonts/Helvetica.ttc", 16);
    if (!font) {
        std::cerr << "Failed to load font: " << TTF_GetError() << "\n";
        TTF_Quit();
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bool running = true;
    Uint32 frame_start;
    Uint32 frame_time;
    Uint32 last_fps_update = 0;
    int frame_count = 0;
    double fps = 0.0;

    int frame_counter = 0;
    while (running) {
        frame_start = SDL_GetTicks();

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
                // Camera movement keys (WASD for panning)
                // Not implemented for simplicity
            } else if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    mouseCaptured = true;
                    SDL_SetRelativeMouseMode(SDL_TRUE);
                }
            } else if (event.type == SDL_MOUSEBUTTONUP) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    mouseCaptured = false;
                    SDL_SetRelativeMouseMode(SDL_FALSE);
                }
            } else if (event.type == SDL_MOUSEMOTION && mouseCaptured) {
                handleMouseMotion(event.motion.xrel, event.motion.yrel);
            } else if (event.type == SDL_MOUSEWHEEL) {
                handleMouseScroll(event.wheel.y);
            }
        }

        sim.applyGravityUsingOctree();
        sim.verletStep(DT);
        sim.resolvePixelCollisions();

        // Log octree node count every 60 frames (optional)
        frame_counter++;
        if (frame_counter % 60 == 0) {
            int node_count = Simulation::getOctreeNodeCount();
            double maxSpeed = sim.getMaxSpeed();
            std::cerr << "Frame " << frame_counter << ": Octree nodes = " << node_count
                      << ", max speed = " << maxSpeed << std::endl;
            sim.logMemoryStats();
        }

        // Clear screen
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set up projection matrix
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(cameraZoom, (float)WIDTH / (float)HEIGHT, 0.1f, 100000.0f);

        // Set up modelview matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -cameraDistance);
        glRotatef(cameraAngleX, 1.0f, 0.0f, 0.0f);
        glRotatef(cameraAngleY, 0.0f, 1.0f, 0.0f);
        // Center the simulation box
        glTranslatef(-WIDTH/2.0f, -HEIGHT/2.0f, -DEPTH/2.0f);

        // Draw planets as spheres
        const auto& planets = sim.getplanets();
        double maxSpeed = sim.getMaxSpeed();
        for (const auto& p : planets) {
            double vx = p.x - p.prev_x;
            double vy = p.y - p.prev_y;
            double vz = p.z - p.prev_z;
            double speed = std::sqrt(vx*vx + vy*vy + vz*vz);
            double hue = (maxSpeed > 0.0) ? (speed / maxSpeed) * 360.0 : 0.0;
            if (hue < 0.0) hue = 0.0;
            if (hue >= 360.0) hue = 359.999;
            double saturation = 1.0;
            double value = 1.0;
            uint8_t r, g, b;
            HSVtoRGB(hue, saturation, value, r, g, b);
            glColor3f(r / 255.0f, g / 255.0f, b / 255.0f);
            glPushMatrix();
            glTranslatef(p.x, p.y, p.z);
            gluSphere(sphereQuadric, p.radius, 8, 8);
            glPopMatrix();
        }

        // Render FPS text using SDL_ttf (over OpenGL) - skip for simplicity

        SDL_GL_SwapWindow(window);

        // Calculate FPS
        Uint32 current_time = SDL_GetTicks();
        frame_count++;
        if (current_time - last_fps_update >= 500) { // Update every 500ms
            fps = frame_count * 1000.0 / (current_time - last_fps_update);
            // Update window title with FPS
            std::ostringstream title;
            title << "3D Gravity Simulation - FPS: " << static_cast<int>(fps);
            SDL_SetWindowTitle(window, title.str().c_str());
            frame_count = 0;
            last_fps_update = current_time;
        }

        frame_time = SDL_GetTicks() - frame_start;
        if (frame_time < 16) {
            SDL_Delay(16 - frame_time);
        }
    }

    // Cleanup
    TTF_CloseFont(font);
    if (sphereQuadric) gluDeleteQuadric(sphereQuadric);
    TTF_Quit();
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
