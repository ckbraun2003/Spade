#version 430 core

struct Material {
    vec4 color;
    float emission;
    float roughness;
    float metallic;
};

layout (std430, binding = 7) buffer InstanceMaterialData {
    Material instanceMaterials[];
};

flat in uint v_GlobalInstanceIndex;

out vec4 FragColor;

void main() {
    Material instanceMaterial = instanceMaterials[v_GlobalInstanceIndex];

    FragColor = instanceMaterial.color;
}
