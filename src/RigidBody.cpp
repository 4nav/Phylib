#include "RigidBody.h"
#include <algorithm>

// ─── Constructor ─────────────────────────────────────────────────────────────

RigidBody::RigidBody(glm::vec2 pos,
                     glm::vec2 size,
                     float     m,
                     glm::vec4 col,
                     float     rest,
                     float     damp)
    : position(pos)
    , velocity(0.0f, 0.0f)
    , acceleration(0.0f, 0.0f)
    , mass(m)
    , invMass(m > 0.0f ? 1.0f / m : 0.0f)
    , halfSize(size * 0.5f)
    , restitution(rest)
    , linearDamping(damp)
    , color(col)
    , isStatic(m <= 0.0f)
    , isAlive(true)
{}

// ─── Physics ─────────────────────────────────────────────────────────────────

void RigidBody::applyForce(glm::vec2 force)
{
    if (isStatic) return;
    // F = m*a  →  a += F/m
    acceleration += force * invMass;
}

void RigidBody::update(float dt)
{
    if (isStatic) return;

    //   1. Update velocity using current acceleration
    //   2. Update position using the new velocity
    velocity += acceleration * dt;

    // Apply linear damping: exponential decay is approximated as pow(factor, dt).
    // For small dt this is essentially vel *= factor once per second.
    float dampFactor = 1.0f - (1.0f - linearDamping) * dt;
    dampFactor = std::max(0.0f, std::min(1.0f, dampFactor));
    velocity *= dampFactor;

    position += velocity * dt;
}

void RigidBody::clearForces()
{
    acceleration = glm::vec2(0.0f, 0.0f);
}

// ─── Geometry ────────────────────────────────────────────────────────────────

AABB RigidBody::getAABB() const
{
    return { position - halfSize, position + halfSize };
}
