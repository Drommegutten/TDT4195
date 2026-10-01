#version 430 core

in vec4 vertexColor;
in vec3 vertexNormal;

out vec4 FragColor;

void main()
{
    // Direction the light travels in (from the sun towards the scene)
    vec3 lightDirection = normalize(vec3(0.8, -0.5, 0.6));

    // Task 1d: Lambertian shading, only the RGB components are lit
    float diffuse = max(0.0, dot(normalize(vertexNormal), -lightDirection));
    FragColor = vec4(vertexColor.rgb * diffuse, vertexColor.a);

}
