#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <string>
#include <algorithm>

#include "PhysicsWorld.h"
#include "Renderer.h"

// ─── Constants ───────────────────────────────────────────────────────────────

static constexpr int WINDOW_W = 1280;
static constexpr int WINDOW_H = 720;
static const char* WINDOW_TITLE = "Phylib — C++ / OpenGL Physics Engine";

// ─── Global pointers (needed in callbacks) ───────────────────────────────────

static PhysicsWorld* g_world    = nullptr;
static Renderer*     g_renderer = nullptr;

// ─── GLFW callbacks ───────────────────────────────────────────────────────────

/// Keyboard callback — handles all keyboard controls.
static void keyCallback(GLFWwindow* window,
                        int key, int /*scancode*/, int action, int /*mods*/)
{
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    switch (key)
    {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        break;

    case GLFW_KEY_SPACE:
        // Pause / Resume simulation
        g_world->togglePause();
        break;

    case GLFW_KEY_R:
        // Reset entire simulation to initial conditions
        g_world->reset();
        std::cout << "[Input] Simulation reset.\n";
        break;

    case GLFW_KEY_G:
        // Toggle gravity on/off
        g_world->toggleGravity();
        break;

    case GLFW_KEY_P:
        // Spawn a new simple pendulum
        g_world->spawnPendulum();
        std::cout << "[Input] New pendulum spawned.\n";
        break;

    case GLFW_KEY_S:
    {
        // Spawn a spring-mass system at a random horizontal position
        float x = (float)(rand() % (WINDOW_W - 200) + 100);
        g_world->spawnSpringSystem(glm::vec2(x, (float)WINDOW_H));
        std::cout << "[Input] New spring system spawned.\n";
        break;
    }
    default: break;
    }
}

/// Mouse-button callback — left-click spawns a body at the cursor position.
static void mouseButtonCallback(GLFWwindow* window,
                                int button, int action, int /*mods*/)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);

        // GLFW gives Y from top; OpenGL world has Y from bottom → flip
        float worldX = (float)mx;
        float worldY = (float)WINDOW_H - (float)my;

        g_world->spawnBody(glm::vec2(worldX, worldY));
    }
}

/// Scroll callback — adjust gravity strength.
static void scrollCallback(GLFWwindow* /*window*/, double /*xoff*/, double yoff)
{
    // Scroll up = stronger gravity, scroll down = weaker
    float factor = (yoff > 0.0) ? 1.15f : 0.87f;
    g_world->scaleGravity(factor);
}

/// Framebuffer-resize callback — update viewport and projection.
static void framebufferSizeCallback(GLFWwindow* /*window*/, int w, int h)
{
    if (g_renderer && w > 0 && h > 0)
        g_renderer->setViewport(w, h);
}

// ─── Entry point ─────────────────────────────────────────────────────────────

int main()
{
    // ── GLFW init ──────────────────────────────────────────────────────────────
    if (!glfwInit())
    {
        std::cerr << "[GLFW] Initialisation failed.\n";
        return 1;
    }

    // Request OpenGL 3.3 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // macOS requirement
#endif
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4); // request 4× MSAA

    GLFWwindow* window = glfwCreateWindow(WINDOW_W, WINDOW_H,
                                          WINDOW_TITLE, nullptr, nullptr);
    if (!window)
    {
        std::cerr << "[GLFW] Window creation failed.\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // VSync — cap at monitor refresh rate

    // ── GLEW init ──────────────────────────────────────────────────────────────
    glewExperimental = GL_TRUE; // required for core profile on some drivers
    GLenum glewErr = glewInit();
    if (glewErr != GLEW_OK)
    {
        std::cerr << "[GLEW] Init failed: " << glewGetErrorString(glewErr) << "\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // Enable MSAA
    glEnable(GL_MULTISAMPLE);

    // Print GPU info
    std::cout << "Renderer : " << glGetString(GL_RENDERER) << "\n";
    std::cout << "OpenGL   : " << glGetString(GL_VERSION)  << "\n\n";
    std::cout << "Controls:\n"
              << "  SPACE      — Pause / Resume\n"
              << "  R          — Reset simulation\n"
              << "  G          — Toggle gravity\n"
              << "  P          — Spawn pendulum\n"
              << "  S          — Spawn spring system\n"
              << "  LMB Click  — Spawn rigid body at cursor\n"
              << "  Scroll     — Adjust gravity strength\n"
              << "  ESC        — Quit\n\n";

    // ── Create simulation objects ──────────────────────────────────────────────
    PhysicsWorld world(WINDOW_W, WINDOW_H);
    Renderer     renderer;

    g_world    = &world;
    g_renderer = &renderer;

    if (!renderer.init(WINDOW_W, WINDOW_H))
    {
        std::cerr << "[Renderer] Initialisation failed. "
                     "Are the shaders/ folder and GLEW installed?\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // ── Register callbacks ─────────────────────────────────────────────────────
    glfwSetKeyCallback(window,             keyCallback);
    glfwSetMouseButtonCallback(window,     mouseButtonCallback);
    glfwSetScrollCallback(window,          scrollCallback);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // ── Main loop ─────────────────────────────────────────────────────────────
    double prevTime = glfwGetTime();

    // FPS counter state
    double fpsTimer  = 0.0;
    int    frameCount = 0;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // ── Delta time ─────────────────────────────────────────────────────────
        double now = glfwGetTime();
        float  dt  = (float)(now - prevTime);
        prevTime   = now;

        // Guard against absurdly large dt (e.g., on first frame or after resize)
        dt = std::min(dt, 0.1f);

        // ── FPS title update (once per second) ────────────────────────────────
        fpsTimer += dt;
        ++frameCount;
        if (fpsTimer >= 1.0)
        {
            std::string title = std::string(WINDOW_TITLE)
                              + "  |  " + std::to_string(frameCount) + " FPS"
                              + "  |  bodies: "
                              + std::to_string(world.bodies.size())
                              + (world.paused ? "  [PAUSED]" : "")
                              + (!world.gravityEnabled ? "  [NO GRAVITY]" : "");
            glfwSetWindowTitle(window, title.c_str());
            fpsTimer   = 0.0;
            frameCount = 0;
        }

        // ── Physics step ───────────────────────────────────────────────────────
        world.update(dt);

        // ── Render ─────────────────────────────────────────────────────────────
        renderer.renderWorld(world);

        glfwSwapBuffers(window);
    }

    // ── Cleanup ────────────────────────────────────────────────────────────────
    g_world    = nullptr;
    g_renderer = nullptr;

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
