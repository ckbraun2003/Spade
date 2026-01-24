#version 430 core

struct Motion {
    vec3 velocity;
    float mass;
    vec3 acceleration;
    float density;
};

layout(std430, binding = 6) buffer InstanceMotionData {
    Motion instanceMotions[];
};

in vec3 v_Normal;
flat in uint v_GlobalInstanceIndex;

out vec4 FragColor;

// Customize your colors here (or make them uniforms to change in C++)
const vec3 cCold = vec3(0.0, 0.3, 1.0);  // Deep Blue (Stationary)
const vec3 cMid  = vec3(0.8, 0.5, 0.0);  // Orange (Moving)
const vec3 cHot  = vec3(1.0, 1.0, 1.0);  // White (Fast)
const float sensitivity = 0.05;

void main()
{
    Motion instanceMotion = instanceMotions[v_GlobalInstanceIndex];

    FragColor = vec4(vec3(instanceMotion.density), 1.0);
}