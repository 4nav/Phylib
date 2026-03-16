#pragma once

#include <string>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

/// Loads, compiles, and links a GLSL vertex + fragment shader pair.
/// Provides convenience setters for the uniform types we use.
class Shader
{
public:
    GLuint ID = 0; ///< OpenGL program handle (0 = not loaded)

    Shader() = default;
    ~Shader();

    /// Read shader files from disk, compile, and link.
    /// @return true on success, false if any stage failed (error printed to stderr)
    bool load(const std::string& vertPath, const std::string& fragPath);

    /// Bind this shader program for subsequent draw calls.
    void use() const;

    // ── Uniform setters ───────────────────────────────────────────────────────
    void setMat4 (const std::string& name, const glm::mat4& m)  const;
    void setVec4 (const std::string& name, const glm::vec4& v)  const;
    void setVec3 (const std::string& name, const glm::vec3& v)  const;
    void setFloat(const std::string& name, float f)             const;
    void setInt  (const std::string& name, int   i)             const;

private:
    /// Compile a single shader stage. Returns GL handle, or 0 on error.
    static GLuint compileShader(GLenum type, const std::string& src);

    /// Read an entire text file into a std::string.
    static std::string readFile(const std::string& path);
};
