#version 430 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;
layout(location = 2) in vec3 normal;

out vec4 vertexColor;
out vec3 vertexNormal;

// Combined transformation matrix (projection * view for now)

uniform mat4 mvp;
uniform mat4 model;

void main()
{
    gl_Position = mvp * vec4(position, 1.0f);
    vertexColor = color;
    vertexNormal = normalize(mat3(model) * normal);
}
