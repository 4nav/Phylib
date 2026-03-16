#include "Renderer.h"
#include "PhysicsWorld.h"

#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─── Destructor ───────────────────────────────────────────────────────────────

Renderer::~Renderer()
{
    if (circleVAO_) { glDeleteVertexArrays(1, &circleVAO_); glDeleteBuffers(1, &circleVBO_); }
    if (quadVAO_)   { glDeleteVertexArrays(1, &quadVAO_);   glDeleteBuffers(1, &quadVBO_);   }
    if (lineVAO_)   { glDeleteVertexArrays(1, &lineVAO_);   glDeleteBuffers(1, &lineVBO_);   }
}

// ─── Initialisation ───────────────────────────────────────────────────────────

bool Renderer::init(int w, int h)
{
    // Enable blending so semi-transparent trails look correct
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Load shader from the shaders/ directory (relative to executable)
    if (!shader_.load("shaders/vertex.glsl", "shaders/fragment.glsl"))
        return false;

    buildCircleMesh(48);
    buildQuadMesh();
    buildLineMesh();

    setViewport(w, h);
    return true;
}

void Renderer::setViewport(int w, int h)
{
    glViewport(0, 0, w, h);
    // Orthographic: (0,0) bottom-left, (w,h) top-right, depth [-1,1]
    projection_ = glm::ortho(0.0f, (float)w, 0.0f, (float)h, -1.0f, 1.0f);
}

// ─── Static mesh builders ────────────────────────────────────────────────────

void Renderer::buildCircleMesh(int segments)
{
    // Triangle-fan for a unit circle centered at origin.
    // vertex[0] = center, then segments+1 perimeter vertices.
    std::vector<glm::vec2> verts;
    verts.reserve(segments + 2);

    verts.emplace_back(0.0f, 0.0f); // center

    for (int i = 0; i <= segments; ++i)
    {
        float angle = (float)i / (float)segments * 2.0f * (float)M_PI;
        verts.emplace_back(std::cos(angle), std::sin(angle));
    }
    circleVertCount_ = (int)verts.size();

    glGenVertexArrays(1, &circleVAO_);
    glGenBuffers(1, &circleVBO_);
    glBindVertexArray(circleVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, circleVBO_);
    glBufferData(GL_ARRAY_BUFFER,
                 verts.size() * sizeof(glm::vec2), verts.data(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Renderer::buildQuadMesh()
{
    // Unit square centred at origin, two triangles
    // (-0.5,-0.5) to (0.5,0.5)
    glm::vec2 verts[] = {
        {-0.5f, -0.5f}, { 0.5f, -0.5f}, { 0.5f,  0.5f},
        {-0.5f, -0.5f}, { 0.5f,  0.5f}, {-0.5f,  0.5f}
    };

    glGenVertexArrays(1, &quadVAO_);
    glGenBuffers(1, &quadVBO_);
    glBindVertexArray(quadVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void Renderer::buildLineMesh()
{
    // Empty dynamic buffer — data is uploaded per draw call
    glGenVertexArrays(1, &lineVAO_);
    glGenBuffers(1, &lineVBO_);
    glBindVertexArray(lineVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO_);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), nullptr);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

// ─── Primitive draw calls ────────────────────────────────────────────────────

void Renderer::clear() const
{
    glClearColor(0.07f, 0.07f, 0.10f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

void Renderer::drawCircle(glm::vec2 centre, float radius, glm::vec4 color)
{
    shader_.use();

    // Model: translate to centre, scale by radius
    glm::mat4 model = glm::translate(glm::mat4(1.0f),
                                     glm::vec3(centre, 0.0f));
    model = glm::scale(model, glm::vec3(radius, radius, 1.0f));

    shader_.setMat4("uProjection", projection_);
    shader_.setMat4("uModel",      model);
    shader_.setVec4("uColor",      color);

    glBindVertexArray(circleVAO_);
    glDrawArrays(GL_TRIANGLE_FAN, 0, circleVertCount_);
    glBindVertexArray(0);
}

void Renderer::drawRect(glm::vec2 centre, glm::vec2 size, glm::vec4 color)
{
    shader_.use();

    glm::mat4 model = glm::translate(glm::mat4(1.0f),
                                     glm::vec3(centre, 0.0f));
    model = glm::scale(model, glm::vec3(size, 1.0f));

    shader_.setMat4("uProjection", projection_);
    shader_.setMat4("uModel",      model);
    shader_.setVec4("uColor",      color);

    glBindVertexArray(quadVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}

void Renderer::drawLine(glm::vec2 p1, glm::vec2 p2, glm::vec4 color, float width)
{
    glLineWidth(width);
    uploadAndDraw({p1, p2}, GL_LINES, color, width);
    glLineWidth(1.0f);
}

void Renderer::drawPolyline(const std::vector<glm::vec2>& pts, glm::vec4 color, float width)
{
    if (pts.size() < 2) return;
    glLineWidth(width);
    uploadAndDraw(pts, GL_LINE_STRIP, color, width);
    glLineWidth(1.0f);
}

// ─── Spring rendering (procedural zigzag) ────────────────────────────────────

std::vector<glm::vec2> Renderer::makeSpringPoints(glm::vec2 p1, glm::vec2 p2,
                                                   int numCoils, float amplitude)
{
    std::vector<glm::vec2> pts;

    glm::vec2 delta = p2 - p1;
    float     len   = glm::length(delta);
    if (len < 1.0f) return pts;

    glm::vec2 dir  = delta / len;
    glm::vec2 perp(-dir.y, dir.x); // perpendicular to spring axis

    // Straight connector sections at each end
    float margin     = std::min(15.0f, len * 0.15f);
    float springLen  = len - 2.0f * margin;
    glm::vec2 sStart = p1 + dir * margin;
    glm::vec2 sEnd   = p2 - dir * margin;

    pts.push_back(p1);
    pts.push_back(sStart);

    int totalZags = numCoils * 2;
    for (int i = 0; i < totalZags; ++i)
    {
        float     t    = (float)(i + 1) / (float)(totalZags + 1);
        glm::vec2 base = sStart + dir * (springLen * t);
        float     side = (i % 2 == 0) ? amplitude : -amplitude;
        pts.push_back(base + perp * side);
    }

    pts.push_back(sEnd);
    pts.push_back(p2);

    return pts;
}

void Renderer::drawSpring(glm::vec2 p1, glm::vec2 p2, glm::vec4 color)
{
    auto pts = makeSpringPoints(p1, p2);
    if (pts.empty()) return;

    glLineWidth(2.0f);
    uploadAndDraw(pts, GL_LINE_STRIP, color);
    glLineWidth(1.0f);
}

// ─── Trail rendering (fading alpha) ──────────────────────────────────────────

void Renderer::drawTrail(const std::deque<glm::vec2>& trail, glm::vec4 baseColor)
{
    if (trail.size() < 2) return;

    // Draw in 8 chunks with increasing alpha (oldest → lowest alpha)
    constexpr int CHUNKS = 8;
    int total = (int)trail.size();
    int chunk = std::max(1, total / CHUNKS);

    for (int c = 0; c < CHUNKS; ++c)
    {
        int start = c * chunk;
        int end   = (c == CHUNKS - 1) ? total : (c + 1) * chunk + 1;
        if (start >= total) break;
        end = std::min(end, total);

        float alpha = ((float)(c + 1) / (float)(CHUNKS + 1)) * baseColor.a;
        glm::vec4 col(baseColor.r, baseColor.g, baseColor.b, alpha);

        std::vector<glm::vec2> seg(trail.begin() + start,
                                   trail.begin() + end);
        glLineWidth(1.5f);
        uploadAndDraw(seg, GL_LINE_STRIP, col);
    }
    glLineWidth(1.0f);
}

// ─── Grid ────────────────────────────────────────────────────────────────────

void Renderer::drawGrid(int worldW, int worldH, int spacing)
{
    std::vector<glm::vec2> lines;
    glm::vec4 gridColor(1.0f, 1.0f, 1.0f, 0.04f);

    for (int x = 0; x <= worldW; x += spacing)
    {
        lines.emplace_back((float)x, 0.0f);
        lines.emplace_back((float)x, (float)worldH);
    }
    for (int y = 0; y <= worldH; y += spacing)
    {
        lines.emplace_back(0.0f,        (float)y);
        lines.emplace_back((float)worldW, (float)y);
    }

    glLineWidth(1.0f);
    uploadAndDraw(lines, GL_LINES, gridColor, 1.0f);
}

// ─── Scene rendering ─────────────────────────────────────────────────────────

void Renderer::renderWorld(const PhysicsWorld& world)
{
    clear();
    drawGrid(world.worldW, world.worldH);

    // ── Rigid bodies ──────────────────────────────────────────────────────────
    for (const auto& b : world.bodies)
    {
        if (!b->isAlive) continue;
        glm::vec2 size = b->halfSize * 2.0f;
        drawRect(b->position, size, b->color);
        // Subtle outline
        glm::vec4 outline = b->color;
        outline.a = 0.5f;
        // (outline omitted for performance — bodies are clearly visible)
    }

    // ── Springs ───────────────────────────────────────────────────────────────
    for (const auto& s : world.springs)
    {
        // Draw small anchor dot if pinned to world
        if (!s->bodyB)
            drawCircle(s->anchor, 6.0f, {0.8f, 0.8f, 0.8f, 0.9f});

        drawSpring(s->getPointA(), s->getPointB(), s->color);

        // Draw body centres as filled circles
        drawCircle(s->getPointA(), s->bodyA->halfSize.x * 0.8f, s->bodyA->color);
        if (s->bodyB)
            drawCircle(s->getPointB(), s->bodyB->halfSize.x * 0.8f, s->bodyB->color);
    }

    // ── Simple pendulums ──────────────────────────────────────────────────────
    for (const auto& p : world.simplePendulums)
    {
        glm::vec2 bob = p->getBobPosition();

        // Rod
        drawLine(p->anchor, bob, p->color, 2.5f);

        // Anchor pin
        drawCircle(p->anchor, 5.0f, {0.85f, 0.85f, 0.85f, 1.0f});

        // Bob
        drawCircle(bob, p->bobRadius, p->color);

        // Bright highlight on bob
        glm::vec4 highlight = p->color + glm::vec4(0.3f, 0.3f, 0.3f, 0.0f);
        highlight = glm::clamp(highlight, glm::vec4(0.0f), glm::vec4(1.0f));
        drawCircle(bob, p->bobRadius * 0.4f, highlight);
    }

    // ── Double pendulum ───────────────────────────────────────────────────────
    for (const auto& dp : world.doublePendulums)
    {
        glm::vec2 bob1 = dp->getBob1Position();
        glm::vec2 bob2 = dp->getBob2Position();

        // Fading trail FIRST (behind everything else)
        glm::vec4 trailColor(dp->color.r, dp->color.g, dp->color.b, 0.85f);
        drawTrail(dp->trail, trailColor);

        // Rods
        drawLine(dp->anchor, bob1, dp->color,                     2.5f);
        drawLine(bob1,       bob2, dp->color * glm::vec4(0.8f, 0.8f, 1.0f, 1.0f), 2.5f);

        // Anchor pin
        drawCircle(dp->anchor, 5.0f, {0.85f, 0.85f, 0.85f, 1.0f});

        // Bobs
        drawCircle(bob1, dp->bobRadius,
                   dp->color * glm::vec4(0.9f, 0.9f, 0.9f, 1.0f));
        drawCircle(bob2, dp->bobRadius,
                   glm::vec4(dp->color.r, dp->color.g, dp->color.b * 1.2f,
                              dp->color.a));
    }
}

// ─── Private helper ───────────────────────────────────────────────────────────

void Renderer::uploadAndDraw(const std::vector<glm::vec2>& pts,
                              GLenum mode,
                              glm::vec4 color,
                              float /*lineWidth*/)
{
    if (pts.empty()) return;

    shader_.use();
    shader_.setMat4("uProjection", projection_);
    shader_.setMat4("uModel",      glm::mat4(1.0f));
    shader_.setVec4("uColor",      color);

    glBindVertexArray(lineVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO_);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(pts.size() * sizeof(glm::vec2)),
                 pts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(mode, 0, (GLsizei)pts.size());
    glBindVertexArray(0);
}
