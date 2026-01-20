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

    float speed = length(instanceMotion.velocity);

    // --- THE MAGIC FORMULA ---
    // Instead of clamping, we use an exponential drop-off.
    // As speed -> infinity, t -> 1.0, but it never "clips."
    float t = 1.0 - exp(-speed * sensitivity);

    // --- COLOR RAMP ---
    vec3 finalColor;

    // We split the gradient into two segments based on 't'
    if (t < 0.5) {
        // Map 0.0 -> 0.5 range to 0.0 -> 1.0 for the Cold-to-Mid blend
        float blend = t * 2.0;
        finalColor = mix(cCold, cMid, blend);
    } else {
        // Map 0.5 -> 1.0 range to 0.0 -> 1.0 for the Mid-to-Hot blend
        float blend = (t - 0.5) * 2.0;
        finalColor = mix(cMid, cHot, blend);
    }

    FragColor = vec4(finalColor, 1.0);
}