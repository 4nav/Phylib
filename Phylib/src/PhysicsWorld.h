#pragma once

#include "RigidBody.h"
#include "Collision.h"
#include "Pendulum.h"
#include "Spring.h"

#include <vector>
#include <memory>
#include <random>
#include <glm/glm.hpp>

/// Manages the entire physics simulation.
///
/// Owns all RigidBodies, Springs, SimplePendulums, and DoublePendulums.
/// Each call to update() advances the simulation by dt seconds:
///   1. Apply gravity to all dynamic bodies
///   2. Apply spring forces (Hooke's law)
///   3. Integrate bodies (semi-implicit Euler)
///   4. Broad-phase AABB collision detection between bodies
///   5. Impulse-based collision resolution + positional correction
///   6. World boundary collision
///   7. Update pendulums
class PhysicsWorld
{
public:
    // ── Owned objects (public for Renderer access) ────────────────────────────
    std::vector<std::unique_ptr<RigidBody>>      bodies;
    std::vector<std::unique_ptr<Spring>>          springs;
    std::vector<std::unique_ptr<SimplePendulum>>  simplePendulums;
    std::vector<std::unique_ptr<DoublePendulum>>  doublePendulums;

    // ── World settings ────────────────────────────────────────────────────────
    int   worldW = 1280;   ///< World / window width  (pixels)
    int   worldH = 720;    ///< World / window height (pixels)
    float gravity = 500.0f; ///< Gravitational acceleration (pixels/s², positive = downward)
    bool  gravityEnabled = true;
    bool  paused = false;

    // ─────────────────────────────────────────────────────────────────────────

    PhysicsWorld(int w, int h);

    /// Advance the simulation by dt seconds.
    /// dt is clamped to 1/30 s to avoid instability on slow frames.
    void update(float dt);

    /// Destroy all objects and recreate the initial demo scene.
    void reset();

    /// Toggle gravity on/off (affects bodies and pendulums).
    void toggleGravity();

    /// Toggle pause state.
    void togglePause();

    // ── Spawn helpers (called by keyboard/mouse input) ────────────────────────

    /// Spawn a free rigid body at the given world position.
    RigidBody* spawnBody(glm::vec2 pos);

    /// Spawn a simple pendulum at a random anchor point near the top.
    SimplePendulum* spawnPendulum();

    /// Spawn a random spring-mass system.
    Spring* spawnSpringSystem(glm::vec2 pos);

    /// Adjust gravity strength by a multiplier (for scroll-wheel).
    void scaleGravity(float factor);

private:
    // Pseudo-random number generator (seeded for determinism)
    std::mt19937 rng_;

    float randomFloat(float lo, float hi);
    glm::vec4 randomBrightColor();

    /// Build the initial demo scene.
    void setupScene();

    /// Build three spring systems at predetermined positions.
    void buildSpringScene();
};
