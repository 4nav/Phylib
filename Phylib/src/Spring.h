#pragma once

#include "RigidBody.h"
#include <glm/glm.hpp>

/// A spring connecting either two RigidBodies or one RigidBody to a fixed anchor.
///
/// Force law (Hooke + viscous damping):
///   F = -k * (x - restLength)   [restoring force]
///   F_d = -d * v_relative        [damping force along spring axis]
///
/// where x is the current spring length, k is stiffness, and d is the damping
/// coefficient.  The net force is applied equally and oppositely to both ends.
class Spring
{
public:
    RigidBody* bodyA  = nullptr; ///< First connected body  (never null)
    RigidBody* bodyB  = nullptr; ///< Second connected body (null → use anchor)
    glm::vec2  anchor = {};      ///< Fixed world-space anchor when bodyB is null

    float stiffness;   ///< Spring constant k (force per unit displacement)
    float restLength;  ///< Natural length of the spring at rest (pixels)
    float dampCoeff;   ///< Viscous damping coefficient d

    glm::vec4 color;   ///< Render color for the spring coil

    /// @param bodyA      Body at end A
    /// @param bodyB      Body at end B (pass nullptr to pin to anchor)
    /// @param anchor     World-space anchor point (used only if bodyB == nullptr)
    /// @param restLen    Natural length of the spring
    /// @param stiffness  Spring constant k
    /// @param dampCoeff  Damping coefficient d
    /// @param color      Coil render color
    Spring(RigidBody* bodyA,
           RigidBody* bodyB,
           glm::vec2  anchor,
           float      restLen,
           float      stiffness,
           float      dampCoeff,
           glm::vec4  color);

    /// Compute and apply Hooke + damping forces to the connected body / bodies.
    /// Call this once per physics step, before body integration.
    void applyForces();

    /// World-space position of the A end (centre of bodyA).
    glm::vec2 getPointA() const;

    /// World-space position of the B end (centre of bodyB or fixed anchor).
    glm::vec2 getPointB() const;
};
