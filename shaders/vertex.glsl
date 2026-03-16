#version 330 core

// 2D position of each vertex in local/object space
layout(location = 0) in vec2 aPos;

// Orthographic projection matrix (set once per frame)
uniform mat4 uProjection;

// Per-object model matrix (translate + scale for each body/primitive)
uniform mat4 uModel;

void main()
{
    gl_Position = uProjection * uModel * vec4(aPos, 0.0, 1.0);
}
