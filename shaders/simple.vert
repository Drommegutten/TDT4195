#version 430 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 color;

out vec4 vertexColor;
uniform mat4 projection;
uniform mat4 translation;
uniform mat4 camera;

void main()
{   

    vec4 newMatrix =  projection  * translation *  camera * vec4(position, 1.0f);

    gl_Position = newMatrix;
    vertexColor = color;
}