#include "Spade/Core/Resources.hpp"
#include "Spade/Core/Primitives.hpp" // For Vertex definition if there
#include <iostream>
#include <fstream>
#include <sstream>

namespace Spade {

    // --- ResourceManager ---
    ResourceManager& ResourceManager::Get() {
        static ResourceManager instance;
        return instance;
    }

    void ResourceManager::Shutdown() {
        m_Resources.clear();
        m_NameMap.clear();
    }

    ResourceID ResourceManager::CreateMesh(const std::string& name) {
        auto res = std::make_unique<MeshResource>();
        res->name = name;
        res->id = m_NextID++;
        
        ResourceID newID = res->id;
        m_Resources[newID] = std::move(res);
        m_NameMap[name] = newID;
        return newID;
    }

    ResourceID ResourceManager::CreateShader(const std::string& name) {
        auto res = std::make_unique<ShaderResource>();
        res->name = name;
        res->id = m_NextID++;
        
        ResourceID newID = res->id;
        m_Resources[newID] = std::move(res);
        m_NameMap[name] = newID;
        return newID;
    }

    ResourceID ResourceManager::CreateMaterial(const std::string& name) {
        auto res = std::make_unique<MaterialResource>();
        res->name = name;
        res->id = m_NextID++;
        
        ResourceID newID = res->id;
        m_Resources[newID] = std::move(res);
        m_NameMap[name] = newID;
        return newID;
    }
    
    ResourceID ResourceManager::GetMeshID(const std::string& name) {
        if(m_NameMap.find(name) != m_NameMap.end()) return m_NameMap[name];
        return INVALID_RESOURCE_ID;
    }

    ResourceID ResourceManager::GetShaderID(const std::string& name) {
        if(m_NameMap.find(name) != m_NameMap.end()) return m_NameMap[name];
        return INVALID_RESOURCE_ID;
    }

    ResourceID ResourceManager::GetMaterialID(const std::string& name) {
         if(m_NameMap.find(name) != m_NameMap.end()) return m_NameMap[name];
        return INVALID_RESOURCE_ID;
    }


    // --- MeshResource ---
    MeshResource::~MeshResource() {
        if (VAO) glDeleteVertexArrays(1, &VAO);
        if (VBO) glDeleteBuffers(1, &VBO);
        if (EBO) glDeleteBuffers(1, &EBO);
    }

    void MeshResource::Upload(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) {
        // Store CPU copy for Ray Tracing
        this->vertices = vertices;
        this->indices = indices;

        indexCount = static_cast<unsigned int>(indices.size());

        if (VAO == 0) glGenVertexArrays(1, &VAO);
        if (VBO == 0) glGenBuffers(1, &VBO);
        if (EBO == 0) glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        // VBO
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

        // EBO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

        // Attributes (Assuming Vertex struct layout: vec3 pos, vec3 normal, vec2 uv)
        // 0: Pos
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
        // 1: Normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        // 2: TexCoords
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));

        glBindVertexArray(0);
    }


    // --- ShaderResource ---
    ShaderResource::~ShaderResource() {
        if (programID) glDeleteProgram(programID);
    }

    void ShaderResource::LoadFiles(const std::string& vertexPath, const std::string& fragmentPath) {
        std::ifstream vShaderFile(vertexPath);
        std::ifstream fShaderFile(fragmentPath);
        
        if (!vShaderFile.is_open()) {
            std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << vertexPath << std::endl;
        }
        if (!fShaderFile.is_open()) {
            std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << fragmentPath << std::endl;
        }

        std::stringstream vShaderStream, fShaderStream;
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();

        Load(vShaderStream.str(), fShaderStream.str());
    }

    void ShaderResource::Load(const std::string& vertexSrc, const std::string& fragmentSrc) {
        const char* vShaderCode = vertexSrc.c_str();
        const char* fShaderCode = fragmentSrc.c_str();

        unsigned int vertex, fragment;
        int success;
        char infoLog[512];

        // Vertex Shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
        if(!success) {
             glGetShaderInfoLog(vertex, 512, NULL, infoLog);
             std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        };

        // Fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
        if(!success) {
            glGetShaderInfoLog(fragment, 512, NULL, infoLog);
            std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        }

        // Program
        programID = glCreateProgram();
        glAttachShader(programID, vertex);
        glAttachShader(programID, fragment);
        glLinkProgram(programID);
        glGetProgramiv(programID, GL_LINK_STATUS, &success);
        if(!success) {
            glGetProgramInfoLog(programID, 512, NULL, infoLog);
            std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }

        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    void ShaderResource::Use() const {
        glUseProgram(programID);
    }

    int ShaderResource::GetUniformLocation(const std::string& name) const {
        return glGetUniformLocation(programID, name.c_str());
    }

    // --- MaterialResource ---
    void MaterialResource::ApplyState() const {
        if (depthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
        if (backfaceCulling) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
        if (blending) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        } else {
            glDisable(GL_BLEND);
        }
    }
}
