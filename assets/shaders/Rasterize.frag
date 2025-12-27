#version 430 core

struct Material {
    vec4 color;
    float emission;
    float roughness;
    float metallic;
};

layout(std430, binding = 4) buffer MaterialData {
    Material materials[];
};

in vec3 vNormal;
in flat uint vMaterialIndex;

out vec4 FragColor;
void main() {
    Material material = materials[vMaterialIndex];

    if(material.emission > 0.0) {

        FragColor = vec4(material.color.rgb * material.emission, 1.0);

    } else {

        vec3 lightDir = normalize(vec3(1, 1, 1));
        float diff = max(dot(normalize(vNormal), lightDir), 0.1);
        FragColor = vec4(material.color.rgb * diff, 1.0);

    }
}
