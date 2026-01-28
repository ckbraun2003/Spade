#version 430 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec3 normal;
    uint instanceIndex;
} gs_in[];

// Output to Fragment Shader
out vec3 vNormal;
out vec3 vBarycentric;
flat out uint vGlobalInstanceIndex;

void main() {
    uint currentInstance = gs_in[0].instanceIndex;

    // Vertex 0
    gl_Position = gl_in[0].gl_Position;
    vNormal = gs_in[0].normal;
    vGlobalInstanceIndex = currentInstance;// <--- Pass it here
    vBarycentric = vec3(1.0, 0.0, 0.0);
    EmitVertex();

    // Vertex 1
    gl_Position = gl_in[1].gl_Position;
    vNormal = gs_in[1].normal;
    vGlobalInstanceIndex = currentInstance;// <--- Pass it here
    vBarycentric = vec3(0.0, 1.0, 0.0);
    EmitVertex();

    // Vertex 2
    gl_Position = gl_in[2].gl_Position;
    vNormal = gs_in[2].normal;
    vGlobalInstanceIndex = currentInstance;// <--- Pass it here
    vBarycentric = vec3(0.0, 0.0, 1.0);
    EmitVertex();

    EndPrimitive();
}