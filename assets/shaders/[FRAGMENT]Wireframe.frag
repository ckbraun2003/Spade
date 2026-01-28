#version 430 core
#extension GL_OES_standard_derivatives : enable

in vec3 vBarycentric;
out vec4 FragColor;

// 1. Define Colors
vec3 wireColor = vec3(0.0, 0.0, 0.0); // Black
vec3 fillColor = vec3(0.5, 0.5, 0.5); // Gray

void main()
{
    // Wireframe Logic
    vec3 d = fwidth(vBarycentric);
    vec3 a3 = smoothstep(vec3(0.0), d * 1.5, vBarycentric);
    float edgeFactor = min(min(a3.x, a3.y), a3.z);

    FragColor = vec4(mix(wireColor, fillColor, edgeFactor), 1.0);
}