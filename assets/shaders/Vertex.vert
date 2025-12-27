#version 430 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexC;

struct Camera {
    mat4 view;
    mat4 projection;
    mat4 viewInverse;
    mat4 projInverse;
};

struct Transform {
    vec3 position;
    vec4 rotation;
    vec3 scale;
};

struct RenderTable {
    uint transformIndex;
    uint materialIndex;
};

layout(std140, binding = 0) uniform CameraData {
    Camera camera;
};

layout(location = 1) uniform uint u_RenderTableIndex;

layout(std430, binding = 2) buffer RenderTableData {
    RenderTable renderTables[];
};

layout(std430, binding = 3) buffer TransformData {
    Transform transforms[];
};

// Helper: Convert Quaternion (x,y,z,w) to Rotation Matrix
mat4 quatToMat4(vec4 q) {
    float qx = q.x; float qy = q.y; float qz = q.z; float qw = q.w;
    return mat4(
    1.0 - 2.0*qy*qy - 2.0*qz*qz, 2.0*qx*qy + 2.0*qz*qw,       2.0*qx*qz - 2.0*qy*qw,       0.0,
    2.0*qx*qy - 2.0*qz*qw,       1.0 - 2.0*qx*qx - 2.0*qz*qz, 2.0*qy*qz + 2.0*qx*qw,       0.0,
    2.0*qx*qz + 2.0*qy*qw,       2.0*qy*qz - 2.0*qx*qw,       1.0 - 2.0*qx*qx - 2.0*qy*qy, 0.0,
    0.0,                         0.0,                         0.0,                         1.0
    );
}

// Helper: Build Model Matrix
mat4 BuildModelMatrix(vec3 position, vec4 rotation, vec3 scale) {
    // 1. Scale Matrix
    mat4 S = mat4(
    scale.x, 0.0,     0.0,     0.0,
    0.0,     scale.y, 0.0,     0.0,
    0.0,     0.0,     scale.z, 0.0,
    0.0,     0.0,     0.0,     1.0
    );

    // 2. Rotation Matrix
    mat4 R = quatToMat4(rotation);

    // 3. Translation Matrix
    mat4 T = mat4(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    position.x, position.y, position.z, 1.0
    );

    // 4. Model Matrix
    return T * R * S;
}

out vec3 vNormal;
out vec3 vWorlPosition;
out flat uint vMaterialIndex;

void main() {
    vNormal = aNormal;

    RenderTable renderTable = renderTables[u_RenderTableIndex];
    Transform transform = transforms[renderTable.transformIndex];

    mat4 model = BuildModelMatrix(transform.position, transform.rotation, transform.scale);
    vWorlPosition = vec3(model * vec4(aPos, 1.0));

    gl_Position = camera.projection * camera.view * model * vec4(aPos, 1.0);

    vMaterialIndex = renderTable.materialIndex;
}