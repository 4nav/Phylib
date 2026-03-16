#include "PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <iostream>

// ─── Constructor ─────────────────────────────────────────────────────────────

PhysicsWorld::PhysicsWorld(int w, int h)
    : worldW(w)
    , worldH(h)
    , rng_(42) // Fixed seed → deterministic simulation
{
    // Reserve to avoid pointer invalidation for springs that hold raw pointers
    bodies.reserve(256);
    setupScene();
}

// ─── Public API ───────────────────────────────────────────────────────────────

void PhysicsWorld::update(float dt)
{
    if (paused) return;

    // Cap dt to avoid instability on very slow frames / debugging
    dt = std::min(dt, 1.0f / 30.0f);

    float g = gravityEnabled ? gravity : 0.0f;

    // ── 1. Apply gravity (downward = negative Y in our Y-up system) ───────────
    for (auto& body : bodies)
    {
        if (!body->isStatic && body->isAlive)
            body->applyForce(glm::vec2(0.0f, -body->mass * g));
    }

    // ── 2. Apply spring forces ─────────────────────────────────────────────────
    for (auto& spring : springs)
        spring->applyForces();

    // ── 3. Integrate bodies ────────────────────────────────────────────────────
    for (auto& body : bodies)
    {
        if (body->isAlive)
        {
            body->update(dt);
            body->clearForces();
        }
    }

    // ── 4 & 5. Body–body AABB collision detection + resolution ────────────────
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (!bodies[i]->isAlive) continue;
        for (size_t j = i + 1; j < bodies.size(); ++j)
        {
            if (!bodies[j]->isAlive) continue;

            CollisionManifold m = Collision::testAABB(*bodies[i], *bodies[j]);
            if (m.colliding)
            {
                Collision::resolve(m);
                Collision::positionalCorrection(m);
            }
        }
    }

    // ── 6. World boundary collision ────────────────────────────────────────────
    for (auto& body : bodies)
    {
        if (!body->isStatic && body->isAlive)
            Collision::resolveBoundary(*body, (float)worldW, (float)worldH);
    }

    // ── 7. Update pendulums ────────────────────────────────────────────────────
    for (auto& p : simplePendulums)
        p->update(dt, g);

    for (auto& dp : doublePendulums)
        dp->update(dt, g);
}

void PhysicsWorld::reset()
{
    // Clear in safe order (springs hold raw body pointers)
    springs.clear();
    simplePendulums.clear();
    doublePendulums.clear();
    bodies.clear();
    bodies.reserve(256);

    // Re-seed for identical reproduction
    rng_.seed(42);

    setupScene();
}

void PhysicsWorld::toggleGravity()
{
    gravityEnabled = !gravityEnabled;
    std::cout << "[World] Gravity " << (gravityEnabled ? "ON" : "OFF") << "\n";
}

void PhysicsWorld::togglePause()
{
    paused = !paused;
    std::cout << "[World] Simulation " << (paused ? "PAUSED" : "RUNNING") << "\n";
}

void PhysicsWorld::scaleGravity(float factor)
{
    gravity = std::max(50.0f, std::min(2000.0f, gravity * factor));
    std::cout << "[World] Gravity = " << gravity << " px/s²\n";
}

// ─── Spawn helpers ────────────────────────────────────────────────────────────

RigidBody* PhysicsWorld::spawnBody(glm::vec2 pos)
{
    float size  = randomFloat(16.0f, 32.0f);
    auto  color = randomBrightColor();

    auto body = std::make_unique<RigidBody>(pos, glm::vec2(size), 1.5f, color);
    body->velocity = glm::vec2(randomFloat(-200.0f, 200.0f),
                               randomFloat(-100.0f,  200.0f));
    RigidBody* ptr = body.get();
    bodies.push_back(std::move(body));
    return ptr;
}

SimplePendulum* PhysicsWorld::spawnPendulum()
{
    float ax    = randomFloat(100.0f, (float)worldW - 100.0f);
    float ay    = (float)worldH - 10.0f;
    float len   = randomFloat(80.0f, 200.0f);
    float ang   = randomFloat(-1.2f, 1.2f);
    auto  color = randomBrightColor();

    auto p = std::make_unique<SimplePendulum>(glm::vec2(ax, ay), len, ang, 2.0f, color);
    SimplePendulum* ptr = p.get();
    simplePendulums.push_back(std::move(p));
    return ptr;
}

Spring* PhysicsWorld::spawnSpringSystem(glm::vec2 pos)
{
    // A single mass hung from a fixed anchor
    glm::vec2 anchor(pos.x, (float)worldH - 10.0f);
    auto color = randomBrightColor();

    auto mass = std::make_unique<RigidBody>(
        glm::vec2(anchor.x, anchor.y - 100.0f),
        glm::vec2(22.0f),
        2.0f, color);
    RigidBody* massPtr = mass.get();
    bodies.push_back(std::move(mass));

    float restLen = 80.0f;
    auto spring = std::make_unique<Spring>(massPtr, nullptr, anchor,
                                           restLen, 220.0f, 4.0f, color);
    Spring* ptr = spring.get();
    springs.push_back(std::move(spring));
    return ptr;
}

// ─── Private helpers ──────────────────────────────────────────────────────────

float PhysicsWorld::randomFloat(float lo, float hi)
{
    std::uniform_real_distribution<float> dist(lo, hi);
    return dist(rng_);
}

glm::vec4 PhysicsWorld::randomBrightColor()
{
    // Bright, saturated colors — use HSV with S=1, V=1 and random hue
    float hue = randomFloat(0.0f, 360.0f);
    float h   = hue / 60.0f;
    int   i   = (int)h;
    float f   = h - (float)i;
    float q   = 1.0f - f;

    glm::vec3 rgb;
    switch (i % 6)
    {
        case 0: rgb = {1.0f,    f,    0.0f}; break;
        case 1: rgb = {   q, 1.0f,    0.0f}; break;
        case 2: rgb = {0.0f, 1.0f,       f}; break;
        case 3: rgb = {0.0f,    q,    1.0f}; break;
        case 4: rgb = {   f, 0.0f,    1.0f}; break;
        default: rgb = {1.0f, 0.0f,       q}; break;
    }
    return glm::vec4(rgb, 1.0f);
}

// ─── Scene setup ──────────────────────────────────────────────────────────────

void PhysicsWorld::setupScene()
{
    // ── 15 bouncing boxes ─────────────────────────────────────────────────────
    for (int i = 0; i < 15; ++i)
    {
        float x    = randomFloat(40.0f, 600.0f);
        float y    = randomFloat(40.0f, 500.0f);
        float side = randomFloat(18.0f, 38.0f);

        auto body = std::make_unique<RigidBody>(
            glm::vec2(x, y), glm::vec2(side), 1.0f + side * 0.03f,
            randomBrightColor());
        body->velocity = glm::vec2(randomFloat(-250.0f, 250.0f),
                                   randomFloat(-200.0f, 200.0f));
        bodies.push_back(std::move(body));
    }

    // ── Simple pendulums ──────────────────────────────────────────────────────

    // Pendulum 1 — orange, medium length
    simplePendulums.push_back(std::make_unique<SimplePendulum>(
        glm::vec2(230.0f, (float)worldH - 5.0f),
        150.0f,
        0.9f,   // initial angle (radians from vertical)
        2.5f,
        glm::vec4(1.0f, 0.55f, 0.1f, 1.0f)));

    // Pendulum 2 — cyan, shorter
    simplePendulums.push_back(std::make_unique<SimplePendulum>(
        glm::vec2(430.0f, (float)worldH - 5.0f),
        110.0f,
        -0.75f,
        1.8f,
        glm::vec4(0.1f, 0.85f, 0.95f, 1.0f)));

    // ── Double pendulum — purple, starts near horizontal for maximum chaos ────
    doublePendulums.push_back(std::make_unique<DoublePendulum>(
        glm::vec2(660.0f, (float)worldH - 5.0f),
        120.0f, 100.0f,   // lengths
        2.0f,   2.0f,     // masses
        1.4f,   2.0f,     // initial angles (radians) — near horizontal = chaos
        glm::vec4(0.75f, 0.35f, 1.0f, 1.0f)));

    // ── Spring systems ────────────────────────────────────────────────────────
    buildSpringScene();
}

void PhysicsWorld::buildSpringScene()
{
    // ── System 1: single mass on a spring anchored to top wall ───────────────
    {
        glm::vec2 anchor(870.0f, (float)worldH);
        glm::vec4 col(0.3f, 0.95f, 0.45f, 1.0f);
        float restLen = 90.0f;

        auto mass = std::make_unique<RigidBody>(
            glm::vec2(anchor.x, anchor.y - restLen - 20.0f),
            glm::vec2(24.0f), 2.0f, col);
        mass->velocity.x = 60.0f; // slight push to start oscillation
        RigidBody* mPtr = mass.get();
        bodies.push_back(std::move(mass));

        springs.push_back(std::make_unique<Spring>(
            mPtr, nullptr, anchor, restLen, 250.0f, 5.0f, col));
    }

    // ── System 2: two masses connected in series ──────────────────────────────
    {
        glm::vec2 anchor(1010.0f, (float)worldH);
        glm::vec4 col1(0.95f, 0.85f, 0.15f, 1.0f);
        glm::vec4 col2(1.0f,  0.45f, 0.15f, 1.0f);
        float restLen = 80.0f;

        auto mass1 = std::make_unique<RigidBody>(
            glm::vec2(anchor.x, anchor.y - restLen - 10.0f),
            glm::vec2(22.0f), 2.0f, col1);
        auto mass2 = std::make_unique<RigidBody>(
            glm::vec2(anchor.x, anchor.y - restLen * 2.0f - 20.0f),
            glm::vec2(22.0f), 2.0f, col2);

        mass2->velocity.x = -50.0f;

        RigidBody* m1 = mass1.get();
        RigidBody* m2 = mass2.get();
        bodies.push_back(std::move(mass1));
        bodies.push_back(std::move(mass2));

        // Anchor → mass1
        springs.push_back(std::make_unique<Spring>(
            m1, nullptr, anchor, restLen, 230.0f, 5.0f, col1));
        // mass1 → mass2
        springs.push_back(std::make_unique<Spring>(
            m2, m1, glm::vec2(0.0f), restLen, 230.0f, 5.0f, col2));
    }

    // ── System 3: spring chain of 4 masses ───────────────────────────────────
    {
        glm::vec2 anchor(1165.0f, (float)worldH);
        float     restLen  = 70.0f;
        float     stiff    = 200.0f;
        float     damp     = 4.0f;

        // Hue progression for the chain — green → teal → blue → purple
        std::vector<glm::vec4> chainColors = {
            {0.2f, 0.9f, 0.4f, 1.0f},
            {0.1f, 0.8f, 0.8f, 1.0f},
            {0.2f, 0.4f, 1.0f, 1.0f},
            {0.7f, 0.2f, 0.95f, 1.0f},
        };

        std::vector<RigidBody*> chain;
        for (int i = 0; i < 4; ++i)
        {
            float y = anchor.y - restLen * (float)(i + 1) - 10.0f * (float)i;
            auto mass = std::make_unique<RigidBody>(
                glm::vec2(anchor.x, y), glm::vec2(20.0f),
                2.0f, chainColors[i]);
            // Small lateral nudge on alternating masses for visual interest
            mass->velocity.x = (i % 2 == 0) ? 40.0f : -40.0f;
            chain.push_back(mass.get());
            bodies.push_back(std::move(mass));
        }

        // Anchor → chain[0]
        springs.push_back(std::make_unique<Spring>(
            chain[0], nullptr, anchor, restLen, stiff, damp, chainColors[0]));
        // chain[i-1] → chain[i]
        for (int i = 1; i < 4; ++i)
            springs.push_back(std::make_unique<Spring>(
                chain[i], chain[i-1], glm::vec2(0.0f),
                restLen, stiff, damp, chainColors[i]));
    }
}
