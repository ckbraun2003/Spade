#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <format>
#include <iostream>
#include <fstream>
#include <sstream>

#include <glm/glm.hpp>
#include <glad/glad.h>

#include "Spade/Core/Primitives.hpp"

#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif

// --- Manual GL Compute Loader ---
#ifndef GL_COMPUTE_SHADER
#define GL_COMPUTE_SHADER 0x91B9
#endif
#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000
#endif
#ifndef GL_SHADER_IMAGE_ACCESS_BARRIER_BIT
#define GL_SHADER_IMAGE_ACCESS_BARRIER_BIT 0x00000020
#endif
#ifndef GL_RGBA32F
#define GL_RGBA32F 0x8814
#endif
#ifndef GL_WRITE_ONLY
#define GL_WRITE_ONLY 0x88B9
#endif
#ifndef GL_READ_WRITE
#define GL_READ_WRITE 0x88BA
#endif

namespace Spade {

  using BufferID = GLuint;
  using ProgramID = GLuint;

  class Resources
  {
  public:



    // Buffer Creation
    static BufferID CreateVertexArrayObject();
    static BufferID CreateBuffer();

    // Buffer Uploading
    static void UploadVertexBufferObject(const std::vector<Vertex>& vertices, const BufferID& VBO);
    static void UploadElementBufferObject(const std::vector<unsigned int>& indices, const BufferID& EBO);

    template <typename T>
    static void UploadShaderStorageBufferObject(const std::vector<T>& data, const BufferID& SSBO) {
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, SSBO);
      glBufferData(GL_SHADER_STORAGE_BUFFER, data.size() * sizeof(T), data.data(), GL_DYNAMIC_DRAW);
    };

    template <typename T>
    static void UploadUniformBufferObject(const T& object, const BufferID& UBO) {
      glBindBuffer(GL_UNIFORM_BUFFER, UBO);
      glBufferData(GL_UNIFORM_BUFFER, sizeof(T), &object, GL_STATIC_DRAW);
    };

    // Buffer Updating
    static void UpdateVertexBufferObject(const std::vector<Vertex>& vertices, const BufferID& VBO);
    static void UpdateElementBufferObject(const std::vector<unsigned int>& indices, const BufferID& EBO);

    template <typename T>
    static void UpdateShaderStorageBufferObject(const std::vector<T>& objects, const BufferID& SSBO) {
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, SSBO);
      glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, objects.size() * sizeof(T), &objects.data());
    };

    template <typename T>
    static void UpdateUniformBufferObject(const T& object, const BufferID& UBO) {
      glBindBuffer(GL_UNIFORM_BUFFER, UBO);
      glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(T), &object);
    };

    // Uniform Setting
    static void SetUniformUnsignedInt(const int location, const unsigned int value) { glUniform1ui(location, value);}


    // Buffer Binding
    static void BindVertexBufferObject(const BufferID& bufferID) { glBindBuffer(GL_ARRAY_BUFFER, bufferID); }
    static void BindElementBufferObject(const BufferID& bufferID) { glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferID); }
    static void BindUniformBufferObject(const BufferID& bufferID) { glBindBuffer(GL_UNIFORM_BUFFER, bufferID); }
    static void BindUniformToLocation(const int location, const BufferID& UBO) { glBindBufferBase(GL_UNIFORM_BUFFER, location, UBO); }
    static void BindShaderStorageToLocation(const int location, const BufferID& SSBO) { glBindBufferBase(GL_SHADER_STORAGE_BUFFER, location, SSBO); }
    static void BindShaderStorageBufferObject(const BufferID& bufferID) { glBindBuffer(GL_SHADER_STORAGE_BUFFER, bufferID); }
    static void BindVertexArrayObject(const BufferID& bufferID) { glBindVertexArray(bufferID); }

    // Buffer Unbinding
    static void UnbindVertexArrayObject() { glBindVertexArray(0); }

    // Shader Loading
    static std::string LoadShaderFile(const std::string& fileName);
    static unsigned int CreateVertexShader(const std::string& vertexShaderStream);
    static unsigned int CreateFragmentShader(const std::string& fragmentShaderStream);
    static ProgramID CreateShaderProgram(const std::string& vertexShaderFile, const std::string& fragmentShaderFile);
    static void UseShaderProgram(const ProgramID& programID) { glUseProgram(programID); }

  private:

    class ResourcesException : public std::runtime_error
    {
    public:
      explicit ResourcesException(const std::string& message);
    };

  };

}
