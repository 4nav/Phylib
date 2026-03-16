#include "Pendulum.h"

#include <cmath>
#include <algorithm>

// ═════════════════════════════════════════════════════════════════════════════
// SimplePendulum
// ═════════════════════════════════════════════════════════════════════════════

SimplePendulum::SimplePendulum(glm::vec2 anch,
                               float     len,
                               float     angle0,
                               float     mass,
                               glm::vec4 col,
                               float     damp)
    : anchor(anch)
    , length(len)
    , angle(angle0)
    , angularVel(0.0f)
    , angularAcc(0.0f)
    , bobMass(mass)
    , bobRadius(12.0f)
    , damping(damp)
    , color(col)
{}

void SimplePendulum::update(float dt, float g)
{
    // Equation of motion:  α = -(g / L) * sin(θ)
    // Semi-implicit Euler: update ω first, then θ
    angularAcc  = -(g / length) * std::sin(angle);
    angularVel += angularAcc * dt;

    // Angular damping (approximate per-second decay)
    float dampFactor = 1.0f - (1.0f - damping) * dt;
    angularVel *= std::max(0.0f, std::min(1.0f, dampFactor));

    angle += angularVel * dt;
}

glm::vec2 SimplePendulum::getBobPosition() const
{
    // Angle 0 = straight down; positive CCW.
    // In screen coords (Y-up) this becomes: x = sin(θ), y = -cos(θ)
    return anchor + glm::vec2(std::sin(angle) * length,
                             -std::cos(angle) * length);
}


// ═════════════════════════════════════════════════════════════════════════════
// DoublePendulum
// ═════════════════════════════════════════════════════════════════════════════

DoublePendulum::DoublePendulum(glm::vec2 anch,
                               float l1, float l2,
                               float mass1, float mass2,
                               float t1, float t2,
                               glm::vec4 col,
                               float     damp)
    : anchor(anch)
    , L1(l1), L2(l2)
    , m1(mass1), m2(mass2)
    , theta1(t1), theta2(t2)
    , omega1(0.0f), omega2(0.0f)
    , bobRadius(10.0f)
    , damping(damp)
    , color(col)
{}

// ─── RK4 state derivative ─────────────────────────────────────────────────────
/// State vector: [theta1, omega1, theta2, omega2]
/// Returns the derivative [dtheta1, dalpha1, dtheta2, dalpha2]
static void doublePendulumDerivative(
    float t1, float w1, float t2, float w2,
    float l1, float l2, float mass1, float mass2, float g,
    float& dt1, float& dw1, float& dt2, float& dw2)
{
    // Precompute shared terms
    float delta = t2 - t1; // angle difference
    float cosd  = std::cos(delta);
    float sind  = std::sin(delta);

    // Denominator common to both angular acceleration formulas
    float denom = l1 * (2.0f * mass1 + mass2 - mass2 * std::cos(2.0f * delta));

    // Angular acceleration of rod 1 (Lagrangian derivation)
    float alpha1 = (-g * (2.0f * mass1 + mass2) * std::sin(t1)
                    - mass2 * g * std::sin(t1 - 2.0f * t2)
                    - 2.0f * sind * mass2
                      * (w2 * w2 * l2 + w1 * w1 * l1 * cosd))
                   / denom;

    // Angular acceleration of rod 2
    float alpha2 = (2.0f * sind
                    * (w1 * w1 * l1 * (mass1 + mass2)
                       + g * (mass1 + mass2) * std::cos(t1)
                       + w2 * w2 * l2 * mass2 * cosd))
                   / (l2 * (2.0f * mass1 + mass2 - mass2 * std::cos(2.0f * delta)));

    // Derivatives of state
    dt1 = w1;
    dw1 = alpha1;
    dt2 = w2;
    dw2 = alpha2;
}

void DoublePendulum::update(float dt, float g)
{
    // ── RK4 integration ───────────────────────────────────────────────────────
    // k1
    float k1_t1, k1_w1, k1_t2, k1_w2;
    doublePendulumDerivative(theta1, omega1, theta2, omega2,
                             L1, L2, m1, m2, g,
                             k1_t1, k1_w1, k1_t2, k1_w2);
    // k2
    float k2_t1, k2_w1, k2_t2, k2_w2;
    doublePendulumDerivative(theta1 + k1_t1 * dt * 0.5f,
                             omega1 + k1_w1 * dt * 0.5f,
                             theta2 + k1_t2 * dt * 0.5f,
                             omega2 + k1_w2 * dt * 0.5f,
                             L1, L2, m1, m2, g,
                             k2_t1, k2_w1, k2_t2, k2_w2);
    // k3
    float k3_t1, k3_w1, k3_t2, k3_w2;
    doublePendulumDerivative(theta1 + k2_t1 * dt * 0.5f,
                             omega1 + k2_w1 * dt * 0.5f,
                             theta2 + k2_t2 * dt * 0.5f,
                             omega2 + k2_w2 * dt * 0.5f,
                             L1, L2, m1, m2, g,
                             k3_t1, k3_w1, k3_t2, k3_w2);
    // k4
    float k4_t1, k4_w1, k4_t2, k4_w2;
    doublePendulumDerivative(theta1 + k3_t1 * dt,
                             omega1 + k3_w1 * dt,
                             theta2 + k3_t2 * dt,
                             omega2 + k3_w2 * dt,
                             L1, L2, m1, m2, g,
                             k4_t1, k4_w1, k4_t2, k4_w2);

    // Combine: state += dt/6 * (k1 + 2k2 + 2k3 + k4)
    theta1 += (dt / 6.0f) * (k1_t1 + 2.0f*k2_t1 + 2.0f*k3_t1 + k4_t1);
    omega1 += (dt / 6.0f) * (k1_w1 + 2.0f*k2_w1 + 2.0f*k3_w1 + k4_w1);
    theta2 += (dt / 6.0f) * (k1_t2 + 2.0f*k2_t2 + 2.0f*k3_t2 + k4_t2);
    omega2 += (dt / 6.0f) * (k1_w2 + 2.0f*k2_w2 + 2.0f*k3_w2 + k4_w2);

    // Angular damping
    float dampFactor = 1.0f - (1.0f - damping) * dt;
    dampFactor = std::max(0.0f, std::min(1.0f, dampFactor));
    omega1 *= dampFactor;
    omega2 *= dampFactor;

    // ── Record trail ──────────────────────────────────────────────────────────
    trail.push_back(getBob2Position());
    while ((int)trail.size() > TRAIL_MAX)
        trail.pop_front();
}

glm::vec2 DoublePendulum::getBob1Position() const
{
    return anchor + glm::vec2(std::sin(theta1) * L1,
                             -std::cos(theta1) * L1);
}

glm::vec2 DoublePendulum::getBob2Position() const
{
    glm::vec2 bob1 = getBob1Position();
    return bob1 + glm::vec2(std::sin(theta2) * L2,
                           -std::cos(theta2) * L2);
}
