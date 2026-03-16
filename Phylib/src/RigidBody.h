#pragma once

#include <glm/glm.hpp>

/// Axis-Aligned Bounding Box used for collision detection.
struct AABB
{
    glm::vec2 min; ///< Bottom-left corner (world space)
    glm::vec2 max; ///< Top-right corner   (world space)
};

/// A 2D rigid body represented as a rectangle.
///
/// Physics integration uses the Semi-Implicit (Symplectic) Euler method:
///   velocity += acceleration * dt   (update velocity first)
///   position += velocity * dt       (then use the *new* velocity)
/// This conserves energy better than forward Euler for oscillatory systems.
class RigidBody
{
public:
    // ── State ─────────────────────────────────────────────────────────────────
    glm::vec2 position;     ///< Center of the body in world space (pixels)
    glm::vec2 velocity;     ///< Linear velocity (pixels / second)
    glm::vec2 acceleration; ///< Accumulated acceleration this frame (pixels / second²)

    // ── Material properties ───────────────────────────────────────────────────
    float mass;             ///< Mass (arbitrary units, > 0)
    float invMass;          ///< Cached 1/mass for impulse calculations (0 if static)

    glm::vec2 halfSize;     ///< Half-extents of the rectangle (pixels)

    float restitution;      ///< Bounciness coefficient [0 = inelastic, 1 = fully elastic]
    float linearDamping;    ///< Velocity multiplier applied each second [0..1]; 1 = no damping

    // ── Visual ────────────────────────────────────────────────────────────────
    glm::vec4 color;        ///< RGBA render color

    // ── Flags ─────────────────────────────────────────────────────────────────
    bool isStatic;          ///< True → body never moves (infinite mass)
    bool isAlive;           ///< False → body will be removed next frame

    // ─────────────────────────────────────────────────────────────────────────

    /// Construct a rigid body centred at pos with the given full size (width × height).
    RigidBody(glm::vec2 pos,
              glm::vec2 size,
              float     mass,
              glm::vec4 color,
              float     restitution  = 0.65f,
              float     linearDamp   = 0.999f);

    /// Add a force (N) to this body's acceleration accumulator for this frame.
    void applyForce(glm::vec2 force);

    /// Semi-implicit Euler integration: update vel then pos.
    void update(float dt);

    /// Zero out the acceleration accumulator — called after integration each frame.
    void clearForces();

    /// Compute and return the world-space AABB of this body.
    AABB getAABB() const;
};
