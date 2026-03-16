#pragma once

#include "Shader.h"
#include "RigidBody.h"
#include "Pendulum.h"
#include "Spring.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <vector>
#include <deque>

// Forward declare to avoid circular includes
class PhysicsWorld;

/// Handles all OpenGL rendering for the simulation.
///
/// Mesh strategy:
///   circleVAO — precomputed unit-circle triangle fan (scaled per draw call)
///   quadVAO   — precomputed unit square             (scaled per draw call)
///   lineVAO   — dynamic buffer uploaded per draw call (springs, trails, rods)
///
/// All geometry is rendered with a single shader: one model matrix + one color.
class Renderer
{
public:
    Renderer() = default;
    ~Renderer();

    /// Create all GPU resources, compile shaders, build static meshes.
    /// Must be called after a valid OpenGL context is current.
    bool init(int viewportW, int viewportH);

    /// Recalculate the orthographic projection when the window resizes.
    void setViewport(int w, int h);

    // ── Primitive draw calls ─────────────────────────────────────────────────

    /// Draw a filled circle centred at `centre` with the given radius and color.
    void drawCircle(glm::vec2 centre, float radius, glm::vec4 color);

    /// Draw a filled axis-aligned rectangle centred at `centre`.
    void drawRect(glm::vec2 centre, glm::vec2 size, glm::vec4 color);

    /// Draw a single line segment.
    void drawLine(glm::vec2 p1, glm::vec2 p2, glm::vec4 color, float width = 2.0f);

    /// Draw a polyline from a list of points (GL_LINE_STRIP).
    void drawPolyline(const std::vector<glm::vec2>& pts, glm::vec4 color, float width = 1.5f);

    /// Draw a spring as a procedurally generated zigzag coil.
    void drawSpring(glm::vec2 p1, glm::vec2 p2, glm::vec4 color);

    /// Draw a fading trail from a deque (oldest = low alpha, newest = high alpha).
    void drawTrail(const std::deque<glm::vec2>& trail, glm::vec4 baseColor);

    // ── Scene rendering ──────────────────────────────────────────────────────

    /// Render a faint background grid.
    void drawGrid(int worldW, int worldH, int spacing = 60);

    /// Render the full physics world (called once per frame from main loop).
    void renderWorld(const PhysicsWorld& world);

    /// Clear the framebuffer with a near-black background.
    void clear() const;

private:
    Shader    shader_;

    // Static meshes
    GLuint circleVAO_ = 0, circleVBO_ = 0;
    int    circleVertCount_ = 0;

    GLuint quadVAO_ = 0, quadVBO_ = 0;

    // Dynamic line buffer (re-uploaded each draw)
    GLuint lineVAO_ = 0, lineVBO_ = 0;

    // Orthographic projection — bottom-left origin, Y-up, pixel units
    glm::mat4 projection_;

    // ── Private helpers ──────────────────────────────────────────────────────
    void buildCircleMesh(int segments = 40);
    void buildQuadMesh();
    void buildLineMesh();

    /// Upload pts to lineVBO and draw with the given GL primitive mode.
    void uploadAndDraw(const std::vector<glm::vec2>& pts,
                       GLenum mode,
                       glm::vec4 color,
                       float lineWidth = 1.5f);

    /// Generate zigzag points for a spring between p1 and p2.
    static std::vector<glm::vec2> makeSpringPoints(glm::vec2 p1,
                                                   glm::vec2 p2,
                                                   int       numCoils  = 9,
                                                   float     amplitude = 9.0f);
};
