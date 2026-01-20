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

in vec3 v_Normal;
flat in uint v_GlobalInstanceIndex;

out vec4 FragColor;

void main() {
    Material instanceMaterial = instanceMaterials[v_GlobalInstanceIndex];

    if(instanceMaterial.emission > 0.0) {
        FragColor = instanceMaterial.color * instanceMaterial.emission;
    } else {
        vec3 lightDir = normalize(vec3(1, 1, 1));
        float diff = max(dot(normalize(v_Normal), lightDir), 0.1);
        FragColor = instanceMaterial.color * vec4(vec3(diff), 1.0);
    }
}
