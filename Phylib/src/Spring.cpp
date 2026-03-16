#include "Spring.h"
#include <cmath>

// ─── Constructor ─────────────────────────────────────────────────────────────

Spring::Spring(RigidBody* a,
               RigidBody* b,
               glm::vec2  anch,
               float      rest,
               float      k,
               float      d,
               glm::vec4  col)
    : bodyA(a)
    , bodyB(b)
    , anchor(anch)
    , stiffness(k)
    , restLength(rest)
    , dampCoeff(d)
    , color(col)
{}

// ─── Force application ───────────────────────────────────────────────────────

void Spring::applyForces()
{
    glm::vec2 posA = getPointA();
    glm::vec2 posB = getPointB();

    glm::vec2 delta  = posB - posA;
    float     length = glm::length(delta);

    // Avoid division by zero when the spring is collapsed
    if (length < 0.001f) return;

    glm::vec2 dir = delta / length; // unit vector A → B

    // ── Hooke's Law: F = -k * (x - rest_length) ─────────────────────────────
    float displacement = length - restLength;
    glm::vec2 springForce = stiffness * displacement * dir;

    // ── Damping: F_d = -d * (v_A - v_B) · dir  (projected on spring axis) ──
    glm::vec2 velA = bodyA ? bodyA->velocity : glm::vec2(0.0f);
    glm::vec2 velB = bodyB ? bodyB->velocity : glm::vec2(0.0f);
    float     relVelAlong = glm::dot(velA - velB, dir);
    glm::vec2 dampForce   = -dampCoeff * relVelAlong * dir;

    glm::vec2 totalForce = springForce + dampForce;

    // Apply: force on A is toward B (positive dir), force on B is toward A
    if (bodyA && !bodyA->isStatic)
        bodyA->applyForce( totalForce);

    if (bodyB && !bodyB->isStatic)
        bodyB->applyForce(-totalForce);
}

// ─── Endpoint queries ─────────────────────────────────────────────────────────

glm::vec2 Spring::getPointA() const
{
    return bodyA ? bodyA->position : anchor;
}

glm::vec2 Spring::getPointB() const
{
    return bodyB ? bodyB->position : anchor;
}
