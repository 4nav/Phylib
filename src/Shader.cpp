#include "Shader.h"

#include <fstream>
#include <sstream>
#include <iostream>

// ─── Destructor ───────────────────────────────────────────────────────────────

Shader::~Shader()
{
    if (ID != 0)
        glDeleteProgram(ID);
}

// ─── Public ───────────────────────────────────────────────────────────────────

bool Shader::load(const std::string& vertPath, const std::string& fragPath)
{
    std::string vertSrc = readFile(vertPath);
    std::string fragSrc = readFile(fragPath);

    if (vertSrc.empty() || fragSrc.empty())
    {
        std::cerr << "[Shader] Failed to read shader files.\n"
                  << "  vert: " << vertPath << "\n"
                  << "  frag: " << fragPath << "\n";
        return false;
    }

    // Compile individual stages
    GLuint vert = compileShader(GL_VERTEX_SHADER,   vertSrc);
    GLuint frag = compileShader(GL_FRAGMENT_SHADER, fragSrc);

    if (vert == 0 || frag == 0)
    {
        if (vert) glDeleteShader(vert);
        if (frag) glDeleteShader(frag);
        return false;
    }

    // Link into a program
    ID = glCreateProgram();
    glAttachShader(ID, vert);
    glAttachShader(ID, frag);
    glLinkProgram(ID);

    // Check link status
    GLint success = 0;
    glGetProgramiv(ID, GL_LINK_STATUS, &success);
    if (!success)
    {
        char log[512];
        glGetProgramInfoLog(ID, 512, nullptr, log);
        std::cerr << "[Shader] Link error:\n" << log << "\n";
        glDeleteProgram(ID);
        ID = 0;
    }

    // Shaders are now baked into the program — release them
    glDeleteShader(vert);
    glDeleteShader(frag);

    return (success != 0);
}

void Shader::use() const
{
    glUseProgram(ID);
}

// ─── Uniform setters ─────────────────────────────────────────────────────────

void Shader::setMat4(const std::string& name, const glm::mat4& m) const
{
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()),
                       1, GL_FALSE, glm::value_ptr(m));
}

void Shader::setVec4(const std::string& name, const glm::vec4& v) const
{
    glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(v));
}

void Shader::setVec3(const std::string& name, const glm::vec3& v) const
{
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(v));
}

void Shader::setFloat(const std::string& name, float f) const
{
    glUniform1f(glGetUniformLocation(ID, name.c_str()), f);
}

void Shader::setInt(const std::string& name, int i) const
{
    glUniform1i(glGetUniformLocation(ID, name.c_str()), i);
}

// ─── Private helpers ─────────────────────────────────────────────────────────

GLuint Shader::compileShader(GLenum type, const std::string& src)
{
    GLuint shader = glCreateShader(type);
    const char* cstr = src.c_str();
    glShaderSource(shader, 1, &cstr, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        const char* typeName = (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";
        std::cerr << "[Shader] " << typeName << " compile error:\n" << log << "\n";
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

std::string Shader::readFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "[Shader] Cannot open file: " << path << "\n";
        return {};
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}
