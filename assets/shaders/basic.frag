#version 430 core
out vec4 FragColor;
in vec3 Normal;
in vec4 Color;

void main() {
    // Simple lighting
    vec3 lightDir = normalize(vec3(1, 1, 1));
    float diff = max(dot(normalize(Normal), lightDir), 0.1);
    FragColor = Color * vec4(vec3(diff), 1.0);
}
