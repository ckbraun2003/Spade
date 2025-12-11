#version 430 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexC;

layout(std140, binding = 0) uniform PerFrame {
    mat4 view;
    mat4 projection;
};

struct ObjectData {
    mat4 model;
    vec4 color;
};

layout(std430, binding = 1) buffer ObjectBuffer {
    ObjectData objects[];
};

uniform uint u_ObjectIndex;

out vec3 Normal;
out vec4 Color;

void main() {
    ObjectData data = objects[u_ObjectIndex];
    gl_Position = projection * view * data.model * vec4(aPos, 1.0);
    Normal = mat3(transpose(inverse(data.model))) * aNormal;
    Color = data.color;
}
