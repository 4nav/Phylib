#version 330 core

// Final pixel color output
out vec4 FragColor;

// RGBA color set per draw call (uniform = same for all fragments in one draw)
uniform vec4 uColor;

void main()
{
    FragColor = uColor;
}
