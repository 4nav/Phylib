#pragma once

#include <glm/glm.hpp>
#include <deque>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
/// Simple single-bob pendulum.
///
/// Equation of motion:
///   α = -(g / L) * sin(θ)
/// where θ is the angle from vertical, L is the rod length, g is gravity.
///
/// Coordinates: angle 0 = straight down, positive = CCW from vertical.
/// ─────────────────────────────────────────────────────────────────────────────
class SimplePendulum
{
public:
    glm::vec2 anchor;       ///< Fixed pivot point in world space
    float     length;       ///< Rod length (pixels)

    float     angle;        ///< Current angle from vertical (radians)
    float     angularVel;   ///< Angular velocity (radians / second)
    float     angularAcc;   ///< Angular acceleration (radians / second²)

    float     bobMass;      ///< Mass of the bob (arbitrary units)
    float     bobRadius;    ///< Visual radius of the bob (pixels)
    float     damping;      ///< Angular damping per second [0..1] — 1 = no damping

    glm::vec4 color;        ///< Bob and rod color

    /// @param anchor   World-space fixed pivot
    /// @param length   Rod length in pixels
    /// @param angle0   Initial angle from vertical (radians)
    /// @param bobMass  Bob mass
    /// @param color    Render color
    SimplePendulum(glm::vec2 anchor,
                   float     length,
                   float     angle0,
                   float     bobMass,
                   glm::vec4 color,
                   float     damping = 0.9995f);

    /// Advance the simulation by dt seconds given gravitational acceleration g.
    void update(float dt, float g);

    /// World-space position of the bob.
    glm::vec2 getBobPosition() const;
};


// ─────────────────────────────────────────────────────────────────────────────
/// Chaotic double pendulum — two rods connected at a hinge.
///
/// Uses the exact Lagrangian equations of motion for a double pendulum:
///   Both bobs exhibit sensitive dependence on initial conditions (chaos).
///
/// A position trail of the second bob is recorded to visualise the chaotic path.
/// ─────────────────────────────────────────────────────────────────────────────
class DoublePendulum
{
public:
    static constexpr int TRAIL_MAX = 400; ///< Maximum trail length (positions)

    glm::vec2 anchor;   ///< Fixed pivot of the first rod

    float L1, L2;       ///< Rod lengths (pixels)
    float m1, m2;       ///< Bob masses  (arbitrary units)

    float theta1;       ///< Angle of rod 1 from vertical (radians)
    float theta2;       ///< Angle of rod 2 from vertical (radians)
    float omega1;       ///< Angular velocity of rod 1 (rad/s)
    float omega2;       ///< Angular velocity of rod 2 (rad/s)

    float bobRadius;    ///< Visual radius of each bob (pixels)
    float damping;      ///< Angular damping per second

    glm::vec4 color;    ///< Base color (rod 1 + bob 1); trail uses same hue

    /// Last TRAIL_MAX positions of the second bob, oldest first.
    std::deque<glm::vec2> trail;

    DoublePendulum(glm::vec2 anchor,
                   float L1, float L2,
                   float m1, float m2,
                   float theta1, float theta2,
                   glm::vec4 color,
                   float     damping = 0.9998f);

    /// Advance using the Lagrangian double-pendulum equations of motion.
    /// Uses RK4 integration for accurate chaotic behaviour.
    void update(float dt, float g);

    /// World-space position of the first bob (end of rod 1).
    glm::vec2 getBob1Position() const;

    /// World-space position of the second bob (end of rod 2, tip of chain).
    glm::vec2 getBob2Position() const;
};
