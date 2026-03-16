#pragma once

#include "RigidBody.h"
#include <glm/glm.hpp>

/// Data describing a detected collision between two bodies.
struct CollisionManifold
{
    bool      colliding   = false; ///< True if the bodies are overlapping
    glm::vec2 normal      = {};   ///< Collision normal (points from B → A)
    float     penetration = 0.0f; ///< Overlap depth along the normal
    RigidBody* a          = nullptr;
    RigidBody* b          = nullptr;
};

/// Namespace-style utility class for collision detection and resolution.
/// All methods are static — no state.
class Collision
{
public:
    Collision() = delete;

    /// Test two AABB bodies for overlap.
    /// Fills manifold with contact data if they collide.
    static CollisionManifold testAABB(RigidBody& a, RigidBody& b);

    /// Apply an impulse-based collision response.
    /// Separates overlapping bodies and reflects velocities according to
    /// the average restitution coefficient of the two materials.
    static void resolve(CollisionManifold& manifold);

    /// Positional correction (Baumgarte) — nudge bodies apart to prevent
    /// sinking due to floating-point accumulation over many frames.
    static void positionalCorrection(CollisionManifold& manifold);

    /// Bounce a body off the rectangular world boundary.
    /// @param worldW  Width  of the world in pixels
    /// @param worldH  Height of the world in pixels
    static void resolveBoundary(RigidBody& body, float worldW, float worldH);
};
