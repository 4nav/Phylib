#include "Collision.h"
#include <algorithm>
#include <cmath>

// ─── AABB overlap test ───────────────────────────────────────────────────────

CollisionManifold Collision::testAABB(RigidBody& a, RigidBody& b)
{
    CollisionManifold m;
    m.a = &a;
    m.b = &b;

    AABB aabb_a = a.getAABB();
    AABB aabb_b = b.getAABB();

    // Separation along each axis
    float dx = b.position.x - a.position.x;
    float dy = b.position.y - a.position.y;

    float overlapX = (a.halfSize.x + b.halfSize.x) - std::abs(dx);
    float overlapY = (a.halfSize.y + b.halfSize.y) - std::abs(dy);

    // No overlap on either axis → no collision
    if (overlapX <= 0.0f || overlapY <= 0.0f)
        return m; // colliding remains false

    m.colliding = true;

    // Choose the axis of minimum penetration for the contact normal
    if (overlapX < overlapY)
    {
        m.normal      = glm::vec2(dx < 0.0f ? -1.0f : 1.0f, 0.0f);
        m.penetration = overlapX;
    }
    else
    {
        m.normal      = glm::vec2(0.0f, dy < 0.0f ? -1.0f : 1.0f);
        m.penetration = overlapY;
    }

    return m;
}


void Collision::resolve(CollisionManifold& manifold)
{
    if (!manifold.colliding) return;

    RigidBody& a = *manifold.a;
    RigidBody& b = *manifold.b;

    // Relative velocity along the collision normal
    glm::vec2 relVel   = b.velocity - a.velocity;
    float     velAlong = glm::dot(relVel, manifold.normal);

    // Bodies already separating — skip impulse (prevents sticking)
    if (velAlong > 0.0f) return;

    // Combined restitution — use the smaller of the two (more energy absorbed)
    float e = std::min(a.restitution, b.restitution);

    // Impulse scalar:  j = -(1+e) * v_rel·n / (1/mA + 1/mB)
    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass == 0.0f) return; // both static → nothing to resolve

    float j = -(1.0f + e) * velAlong / totalInvMass;

    // Apply impulse along the contact normal
    glm::vec2 impulse = j * manifold.normal;

    if (!a.isStatic) a.velocity -= impulse * a.invMass;
    if (!b.isStatic) b.velocity += impulse * b.invMass;

    // ── Simple friction (tangential damping) ──────────────────────────────────
    // Re-compute relative velocity after normal impulse
    relVel = b.velocity - a.velocity;

    // Tangential (friction) direction
    glm::vec2 tangent = relVel - glm::dot(relVel, manifold.normal) * manifold.normal;
    float     tLen    = glm::length(tangent);
    if (tLen > 0.001f)
        tangent /= tLen;
    else
        return; // no tangential motion

    // Friction impulse magnitude (Coulomb: μ * normal impulse)
    constexpr float mu = 0.25f; // friction coefficient
    float jt = -glm::dot(relVel, tangent) / totalInvMass;

    // Clamp to Coulomb cone: tangential impulse ≤ μ * normal impulse
    float clampedJt = std::max(-mu * j, std::min(jt, mu * j));

    glm::vec2 frictionImpulse = clampedJt * tangent;
    if (!a.isStatic) a.velocity -= frictionImpulse * a.invMass;
    if (!b.isStatic) b.velocity += frictionImpulse * b.invMass;
}

// ─── Positional correction (Baumgarte stabilisation) ─────────────────────────

void Collision::positionalCorrection(CollisionManifold& manifold)
{
    if (!manifold.colliding) return;

    RigidBody& a = *manifold.a;
    RigidBody& b = *manifold.b;

    // Only correct if penetration exceeds a small slop to avoid jitter
    constexpr float slop    = 0.5f;  // pixels
    constexpr float percent = 0.4f;  // correction strength

    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass == 0.0f) return;

    float correction = (std::max(manifold.penetration - slop, 0.0f) / totalInvMass)
                       * percent;
    glm::vec2 corrVec = correction * manifold.normal;

    if (!a.isStatic) a.position -= corrVec * a.invMass;
    if (!b.isStatic) b.position += corrVec * b.invMass;
}

// ─── World boundary bounce ────────────────────────────────────────────────────

void Collision::resolveBoundary(RigidBody& body, float worldW, float worldH)
{
    AABB box = body.getAABB();

    // ── Left wall ──────────────────────────────────────────────────────────────
    if (box.min.x < 0.0f)
    {
        body.position.x = body.halfSize.x;
        if (body.velocity.x < 0.0f)
            body.velocity.x = -body.velocity.x * body.restitution;
    }

    // ── Right wall ────────────────────────────────────────────────────────────
    if (box.max.x > worldW)
    {
        body.position.x = worldW - body.halfSize.x;
        if (body.velocity.x > 0.0f)
            body.velocity.x = -body.velocity.x * body.restitution;
    }

    // ── Floor ─────────────────────────────────────────────────────────────────
    if (box.min.y < 0.0f)
    {
        body.position.y = body.halfSize.y;
        if (body.velocity.y < 0.0f)
            body.velocity.y = -body.velocity.y * body.restitution;

        // Apply floor friction when sliding along the bottom
        body.velocity.x *= 0.97f;
    }

    // ── Ceiling ───────────────────────────────────────────────────────────────
    if (box.max.y > worldH)
    {
        body.position.y = worldH - body.halfSize.y;
        if (body.velocity.y > 0.0f)
            body.velocity.y = -body.velocity.y * body.restitution;
    }
}
