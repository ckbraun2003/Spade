#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include "Spade/Core/Primitives.hpp"

namespace Spade {

    // --- ID Types ---
    using ResourceID = unsigned int;
    const ResourceID INVALID_RESOURCE_ID = 0;

    // --- Base Resource ---
    struct Resource {
        ResourceID id = INVALID_RESOURCE_ID;
        std::string name;
        virtual ~Resource() = default;
    };

    // --- Shape Types ---
    enum class ShapeType {
        Mesh,
        AnalyticSphere,
        AnalyticBox
    };

    // --- Mesh Resource ---
    struct MeshResource : public Resource {
        ShapeType type = ShapeType::Mesh; // Default to mesh
        
        unsigned int VAO = 0;
        unsigned int VBO = 0;
        unsigned int EBO = 0;
        unsigned int indexCount = 0;
        
        // CPU Side Data (Kept for RayTracing/Collision)
        std::vector<struct Vertex> vertices;
        std::vector<unsigned int> indices;

        ~MeshResource();
        void Upload(const std::vector<struct Vertex>& vertices, const std::vector<unsigned int>& indices);
    };

    // --- Shader Resource ---
    struct ShaderResource : public Resource {
        unsigned int programID = 0;

        ~ShaderResource();
        void Load(const std::string& vertexSrc, const std::string& fragmentSrc);
        void LoadFiles(const std::string& vertexPath, const std::string& fragmentPath);
        void Use() const;
        
        // Helper to get location
        int GetUniformLocation(const std::string& name) const;
    };

    // --- Material Resource ---
    struct MaterialResource : public Resource {
        ResourceID shaderID = INVALID_RESOURCE_ID;
        
        // Pipeline State
        bool depthTest = true;
        bool backfaceCulling = true;
        bool blending = false;

        // Default Properties (can be extended with a robust Uniform variant system later)
        glm::vec4 baseColor = {1.0f, 1.0f, 1.0f, 1.0f};
        float data1 = 0.0f; // placeholder for roughness/metallic etc
        
        void ApplyState() const;
    };

    // --- Resource Manager ---
    class ResourceManager {
    public:
        static ResourceManager& Get();

        // Explicitly delete copy constructor and assignment operator for singleton pattern
        ResourceManager(const ResourceManager&) = delete;
        ResourceManager& operator=(const ResourceManager&) = delete;
        
        // Explicit shutdown to clear GL resources before Context destruction
        void Shutdown(); 

        // Generic Get/System
        template<typename T>
        T* GetResource(ResourceID id);

        // Specific Creators
        ResourceID CreateMesh(const std::string& name);
        ResourceID CreateShader(const std::string& name);
        ResourceID CreateMaterial(const std::string& name);

        // Get by Name
        ResourceID GetMeshID(const std::string& name);
        ResourceID GetShaderID(const std::string& name);
        ResourceID GetMaterialID(const std::string& name);

    private:
        ResourceManager() = default;
        
        ResourceID m_NextID = 1;

        std::unordered_map<ResourceID, std::unique_ptr<Resource>> m_Resources;
        std::unordered_map<std::string, ResourceID> m_NameMap;
    };

    // Template Implementation
    template<typename T>
    T* ResourceManager::GetResource(ResourceID id) {
        auto it = m_Resources.find(id);
        if (it != m_Resources.end()) {
            return dynamic_cast<T*>(it->second.get());
        }
        return nullptr;
    }
}
