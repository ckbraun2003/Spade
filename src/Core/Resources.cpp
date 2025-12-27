#include "Spade/Core/Resources.hpp"

namespace Spade {

  BufferID Resources::CreateBuffer() {
    BufferID buffer;
    glGenBuffers(1, &buffer);

    return buffer;
  }

  BufferID Resources::CreateVertexArrayObject() {
    BufferID VAO;
    glGenVertexArrays(1, &VAO);

    return VAO;
  }

  void Resources::UploadVertexBufferObject(const std::vector<Vertex>& vertices, const BufferID& VBO) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
  }

  void Resources::UploadElementBufferObject(const std::vector<unsigned int>& indices, const BufferID& EBO) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
  }

  void Resources::UpdateVertexBufferObject(const std::vector<Vertex>& vertices, const BufferID& VBO) {
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), vertices.data());
  }

  void Resources::UpdateElementBufferObject(const std::vector<unsigned int>& indices, const BufferID& EBO) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, indices.size() * sizeof(unsigned int), indices.data());
  }

  std::string Resources::LoadShaderFile(const std::string& fileName) {
    std::ifstream shaderFile(fileName);

    if (!shaderFile.is_open()) {
      throw ResourcesException("ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: ");
    }

    std::stringstream shaderStream;
    shaderStream << shaderFile.rdbuf();

    return shaderStream.str();
  }

  unsigned int Resources::CreateVertexShader(const std::string& vertexShaderStream) {
    const char* vertexShaderCode = vertexShaderStream.c_str();
    int success;
    char infoLog[512];

    // Vertex Shader
    unsigned int vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vertexShaderCode, NULL);
    glCompileShader(vertex);
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if(!success) {
      glGetShaderInfoLog(vertex, 512, NULL, infoLog);
      throw ResourcesException(std::format("ERROR::SHADER::VERTEX::COMPILATION_FAILED\n{}", infoLog));
    };

    return vertex;
  }

  unsigned int Resources::CreateFragmentShader(const std::string& fragmentShaderStream) {
    const char* fragmentShaderCode = fragmentShaderStream.c_str();
    int success;
    char infoLog[512];

    // Vertex Shader
    unsigned int fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fragmentShaderCode, NULL);
    glCompileShader(fragment);
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if(!success) {
      glGetShaderInfoLog(fragment, 512, NULL, infoLog);
      throw ResourcesException(std::format("ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n{}", infoLog));
    };

    return fragment;
  }

  ProgramID Resources::CreateShaderProgram(const std::string &vertexShaderFile, const std::string &fragmentShaderFile) {
    int success;
    char infoLog[512];

    unsigned int vertex = CreateVertexShader(LoadShaderFile(vertexShaderFile));
    unsigned int fragment = CreateFragmentShader(LoadShaderFile(fragmentShaderFile));

    // Program ID
    ProgramID programID = glCreateProgram();
    glAttachShader(programID, vertex);
    glAttachShader(programID, fragment);
    glLinkProgram(programID);
    glGetProgramiv(programID, GL_LINK_STATUS, &success);
    if(!success) {
      glGetProgramInfoLog(programID, 512, NULL, infoLog);
      throw ResourcesException(std::format("ERROR::SHADER::PROGRAM::LINKING_FAILED\n{}", infoLog));
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);

    return programID;
  }

  Resources::ResourcesException::ResourcesException(const std::string &message) : runtime_error(message) {}

}
