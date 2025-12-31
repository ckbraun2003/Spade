#include "Spade/Core/Engine.hpp"

namespace Spade {

  Engine::~Engine() {
    glDeleteBuffers(1, &m_SSBO_InstanceTransforms);
    glDeleteBuffers(1, &m_SSBO_EntityTransforms);
    glDeleteBuffers(1, &m_SSBO_InstanceMotions);
    glDeleteBuffers(1, &m_SSBO_EntityMotions);

    glDeleteBuffers(1, &m_UBO_EntityTransformIndex);
    glDeleteBuffers(1, &m_UBO_InstanceTransformStartIndex);

    glDeleteBuffers(1, &m_UBO_Camera);

    glDeleteProgram(m_ShaderProgram);
    glDeleteProgram(m_ComputeProgram);

    glfwDestroyWindow(m_GLFWwindow);
    glfwTerminate();
  }

  void Engine::LoadCameraBuffer(Universe &universe) {
    auto& cameraPool = universe.GetPool<CameraComponent>();
    auto& transformPool = universe.GetPool<TransformComponent>();

    for (size_t c = 0; c < cameraPool.m_Data.size(); ++c) {
      CameraComponent& cameraComponent = cameraPool.m_Data[c];
      const EntityID entityID = transformPool.m_IndexToEntity[c];

      TransformComponent* transformComponent = transformPool.Get(entityID);

      if (cameraComponent.isActive) {

        m_ActiveCamera.camera.projection = glm::perspective(glm::radians(m_ActiveCamera.fov),
          ((float)m_WindowSize.x / (float)m_WindowSize.y), m_ActiveCamera.nearPlane, m_ActiveCamera.farPlane);
        m_ActiveCamera.camera.view = glm::inverse(transformComponent->GetModel());
        m_ActiveCamera.camera.viewInverse = m_ActiveCamera.camera.view;
        m_ActiveCamera.camera.projInverse = glm::inverse(m_ActiveCamera.camera.projection);
        break;
      }
    }

    if (m_UBO_Camera == 0) {
      m_UBO_Camera = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadUniformBufferObject<Camera>(m_ActiveCamera.camera, m_UBO_Camera);
      Resources::BindUniformToLocation(0, m_UBO_Camera);
    }

    // Update Camera Data
    Resources::UpdateUniformBufferObject<Camera>(m_ActiveCamera.camera, m_UBO_Camera);

  }

  void Engine::LoadMeshBuffers(Universe &universe) {
    // Establish and reserve storage vectors
    std::vector<Transform> entityTransforms;
    std::vector<Transform> instanceTransforms;

    std::vector<Motion> entityMotions;
    std::vector<Motion> instanceMotions;

    unsigned int entityTransformsSize = 0;
    unsigned int entityTransformIndex = 0;

    unsigned int instanceTransformsSize = 0;
    unsigned int instanceTransformStartIndex = 0;

    auto& meshPool = universe.GetPool<MeshComponent>();
    auto& transformPool = universe.GetPool<TransformComponent>();
    auto& motionPool = universe.GetPool<MotionComponent>();

    for(auto & meshComponent : meshPool.m_Data) {
      ++entityTransformsSize;
      instanceTransformsSize+= meshComponent.instanceTransforms.size();
    }

    entityTransforms.reserve(entityTransformsSize);
    instanceTransforms.reserve(instanceTransformsSize);

    entityMotions.reserve(entityTransformsSize);
    instanceMotions.reserve(instanceTransformsSize);

    // Loop through each mesh
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];
      const EntityID entityID = meshPool.m_IndexToEntity[i];

      TransformComponent* transformComponent = transformPool.Get(entityID);
      MotionComponent* motionComponent = motionPool.Get(entityID);

      // Load Vertex Buffer info
      if (meshComponent.VAO == 0) {
        meshComponent.VAO = Resources::CreateVertexArrayObject();
        meshComponent.VBO = Resources::CreateBuffer();
        meshComponent.EBO = Resources::CreateBuffer();

        Resources::BindVertexArrayObject(meshComponent.VAO);
        Resources::UploadVertexBufferObject(meshComponent.mesh.vertices, meshComponent.VBO);
        Resources::UploadElementBufferObject(meshComponent.mesh.indices, meshComponent.EBO);

        // Position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

        // Normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

        // TexCoords
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
      }

      // Set mesh indices
      meshComponent.entityTransformIndex = entityTransformIndex;
      meshComponent.instanceTransformStartIndex = instanceTransformStartIndex;

      entityTransforms.insert(entityTransforms.end(), transformComponent->transform);
      instanceTransforms.insert(instanceTransforms.end(), meshComponent.instanceTransforms.begin(), meshComponent.instanceTransforms.end());

      entityMotions.insert(entityMotions.end(), motionComponent->motion);
      instanceMotions.insert(instanceMotions.end(), meshComponent.instanceMotions.begin(), meshComponent.instanceMotions.end());

      ++entityTransformIndex;
      instanceTransformStartIndex+=meshComponent.instanceTransforms.size();
    }

    if (m_SSBO_EntityTransforms == 0) {
      m_SSBO_EntityTransforms = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Transform>(entityTransforms, m_SSBO_EntityTransforms);
      Resources::BindShaderStorageToLocation(1, m_SSBO_EntityTransforms);
    }


    if (m_SSBO_InstanceTransforms == 0) {
      m_SSBO_InstanceTransforms = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Transform>(instanceTransforms, m_SSBO_InstanceTransforms);
      Resources::BindShaderStorageToLocation(2, m_SSBO_InstanceTransforms);
    }

    if (m_SSBO_EntityMotions == 0) {
      m_SSBO_EntityMotions = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Motion>(entityMotions, m_SSBO_EntityMotions);
      Resources::BindShaderStorageToLocation(3, m_SSBO_EntityMotions);
    }


    if (m_SSBO_InstanceMotions == 0) {
      m_SSBO_InstanceMotions = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Motion>(instanceMotions, m_SSBO_InstanceMotions);
      Resources::BindShaderStorageToLocation(4, m_SSBO_InstanceMotions);
    }

    if (m_UBO_EntityTransformIndex == 0) {
      m_UBO_EntityTransformIndex = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadUniformBufferObject<unsigned int>(0, m_UBO_EntityTransformIndex);
      Resources::BindUniformToLocation(5, m_UBO_EntityTransformIndex);
    }

    if (m_UBO_InstanceTransformStartIndex == 0) {
      m_UBO_InstanceTransformStartIndex = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadUniformBufferObject<unsigned int>(0, m_UBO_InstanceTransformStartIndex);
      Resources::BindUniformToLocation(6, m_UBO_InstanceTransformStartIndex);
    }

    Resources::UpdateShaderStorageBufferObject<Transform>(entityTransforms, m_SSBO_EntityTransforms);
    Resources::UpdateShaderStorageBufferObject<Transform>(instanceTransforms, m_SSBO_InstanceTransforms);
    Resources::UpdateShaderStorageBufferObject<Motion>(entityMotions, m_SSBO_EntityMotions);
    Resources::UpdateShaderStorageBufferObject<Motion>(instanceMotions, m_SSBO_InstanceMotions);

  }

  void Engine::UpdatePhysics(Universe &universe) {
    if (m_ComputeProgram == 0) {
      m_ComputeProgram = Resources::CreateComputeProgram("assets/shaders/Compute.comp");
    }

    auto& meshPool = universe.GetPool<MeshComponent>();

    for(auto& meshComponent : meshPool.m_Data) {
      Resources::UseProgram(m_ComputeProgram);

      Resources::SetUniformUnsignedInt(5, meshComponent.entityTransformIndex);
      Resources::SetUniformUnsignedInt(6, meshComponent.instanceTransformStartIndex);
      Resources::SetUniformFloat(7, m_DeltaTime);

      GLuint groups = (meshComponent.instanceMotions.size() + 15) / 16;

      glDispatchCompute(groups, 1, 1);

    }

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }


  void Engine::DrawScene(Universe &universe) {
    UpdateStatistics();

    if (m_ShaderProgram == 0) {
      m_ShaderProgram = Resources::CreateRenderProgram("assets/shaders/Vertex.vert", "assets/shaders/Fragment.frag");
    }

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& meshPool = universe.GetPool<MeshComponent>();

    for(auto& meshComponent : meshPool.m_Data) {
      Resources::UseProgram(m_ShaderProgram);

      Resources::SetUniformUnsignedInt(5, meshComponent.entityTransformIndex);
      Resources::SetUniformUnsignedInt(6, meshComponent.instanceTransformStartIndex);

      Resources::BindVertexArrayObject(meshComponent.VAO);
      glDrawElementsInstanced(GL_TRIANGLES, meshComponent.mesh.indices.size(), GL_UNSIGNED_INT, 0, meshComponent.instanceTransforms.size());

      Resources::UnbindVertexArrayObject();

    }

    glfwSwapBuffers(m_GLFWwindow);
    glfwPollEvents();

  }

  void Engine::ProcessInput(Universe& universe) {
    if (IsKeyPressed(GLFW_KEY_ESCAPE)) {
      glfwSetWindowShouldClose(m_GLFWwindow, GLFW_TRUE);
      return;
    }

    auto& inputPool = universe.GetPool<InputComponent>();
    auto& transformPool = universe.GetPool<TransformComponent>();


    for (size_t i = 0; i < inputPool.m_Data.size(); ++i) {
      InputComponent& inputComponent = inputPool.m_Data[i];
      const EntityID entityID = inputPool.m_IndexToEntity[i];

      TransformComponent* transformComponent = transformPool.Get(entityID);

      for (auto& [key, input] : inputComponent.bindings) {
        if (IsKeyPressed(key)) {
          switch (input) {

            case MoveUp:
              transformComponent->transform.position += inputComponent.speed * m_DeltaTime * transformComponent->GetUp();
              break;

            case MoveDown:
              transformComponent->transform.position -= inputComponent.speed * m_DeltaTime * transformComponent->GetUp();
              break;

            case MoveBackward:
              transformComponent->transform.position -= inputComponent.speed * m_DeltaTime * transformComponent->GetForward();
              break;

            case MoveForward:
              transformComponent->transform.position += inputComponent.speed * m_DeltaTime * transformComponent->GetForward();
              break;

            case MoveLeft:
              transformComponent->transform.position -= inputComponent.speed * m_DeltaTime * glm::normalize(glm::cross(transformComponent->GetForward(), transformComponent->GetUp()));
              break;

            case MoveRight:
              transformComponent->transform.position += inputComponent.speed * m_DeltaTime * glm::normalize(glm::cross(transformComponent->GetForward(), transformComponent->GetUp()));
              break;

          }
          LoadCameraBuffer(universe);
        }
      }
    }
  }


  void Engine::SetupEngineWindow(int width, int height, const std::string& title) {
    // Creates window pointer, tell glad render size
    SetupGLFWandGLADWindow(width, height, title);
  }


  bool Engine::IsRunning() const {
      return !glfwWindowShouldClose(m_GLFWwindow);
  }

  float Engine::GetTime() const {
      return (float)glfwGetTime();
  }

  bool Engine::IsKeyPressed(int key) const {
      return glfwGetKey(m_GLFWwindow, key) == GLFW_PRESS;
  }

  bool Engine::IsMouseButtonPressed(int button) const {
      return glfwGetMouseButton(m_GLFWwindow, button) == GLFW_PRESS;
  }

  glm::vec2 Engine::GetMousePosition() const {
      double x, y;
      glfwGetCursorPos(m_GLFWwindow, &x, &y);
      return { (float)x, (float)y };
  }

  void Engine::SetMouseCursorMode(bool trapped) {
      if (trapped) {
          glfwSetInputMode(m_GLFWwindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
      } else {
          glfwSetInputMode(m_GLFWwindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      }
  }


  // --- Private ---
  void Engine::SetupGLFWandGLADWindow(const int& width, const int& height, const std::string& title) {

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Creates the GLFW Window with a Width, Height, Title, etc
    m_GLFWwindow = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);

    // Error handling
    if (m_GLFWwindow == nullptr)
    {
      glfwTerminate();
      throw EngineException("Failed to create GLFW window");
    }

    // Makes the window the current context for the thread
    glfwMakeContextCurrent(m_GLFWwindow);

    // Error handling
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
      throw EngineException("Failed to initialize GLAD");
    }

    // Tells OpenGL size of rendering window
    glViewport(0, 0, width, height);

    m_WindowSize = { (float)width, (float)height };

    glEnable(GL_DEPTH_TEST);
  }

  void Engine::UpdateStatistics() {
    // Update Time and FPS variables
    m_CurrentTime = GetTime();
    m_DeltaTime = m_CurrentTime - m_LastTime;
    m_LastTime = m_CurrentTime;

    m_FPSTimer += m_DeltaTime;
    m_FrameCounter++;
    m_TotalFrames++;

    if (m_FPSTimer >= 1.0f) {
      m_FPS = (float)m_FrameCounter / m_FPSTimer;

      m_FrameCounter = 0;
      m_FPSTimer = 0.0f;
    }

    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
      m_Memory = pmc.PrivateUsage / 1024.0f / 1024.0f;
    }
  }

  bool Engine::IsKeyPressed(int key) {
    return glfwGetKey(m_GLFWwindow, key) == GLFW_PRESS;
  }



  Engine::EngineException::EngineException(const std::string &message) : runtime_error(message) {}
}
