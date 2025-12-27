#include "Spade/Core/Engine.hpp"

namespace Spade {

  Engine::~Engine() {
    glDeleteBuffers(1, &m_SSBO_Triangles);
    glDeleteBuffers(1, &m_SSBO_Spheres);
    glDeleteBuffers(1, &m_SSBO_BoundingBoxes);
    glDeleteBuffers(1, &m_SSBO_Materials);
    glDeleteBuffers(1, &m_SSBO_Transforms);
    glDeleteBuffers(1, &m_SSBO_RenderTables);
    glDeleteProgram(m_ShaderProgram);
    glfwTerminate();
  }

  void Engine::InitializeUniverse(Universe &universe) {
    if (m_IsDirty) {
      SetupStorageCache(universe);
      SetupCameraCache(universe);
      SetupShaderStorageBuffers();
    }
  }


  void Engine::DrawScene(Universe &universe) {
    UpdateStatistics();

    if (m_ShaderProgram == 0) {
      m_ShaderProgram = Resources::CreateShaderProgram("assets/shaders/Vertex.vert", "assets/shaders/Fragment.frag");
    }

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_ActiveCamera) {
      throw EngineException("Active Camera needed for Rendering");
    }

    // Update Camera Data
    Resources::UpdateUniformBufferObject<Camera>(m_ActiveCamera->camera, m_ActiveCamera->m_UBO);

    auto& meshPool = universe.GetPool<MeshComponent>();

    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];

      Resources::SetUniformUnsignedInt(1, i);

      Resources::UseShaderProgram(m_ShaderProgram);
      Resources::BindVertexArrayObject(meshComponent.VAO);

      glDrawElements(GL_TRIANGLES, meshComponent.mesh.indices.size(), GL_UNSIGNED_INT, 0);

      Resources::UnbindVertexArrayObject();

    }

    glfwSwapBuffers(m_GLFWwindow);
    glfwPollEvents();

    m_IsDirty = false;

  }

  void Engine::ProcessInput(Universe& universe) {
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
          m_IsDirty = true;
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

  void Engine::SetupStorageCache(Universe &universe) {
    ClearBufferCache();

    auto& meshPool = universe.GetPool<MeshComponent>();
    auto& transformPool = universe.GetPool<TransformComponent>();
    auto& materialPool = universe.GetPool<MaterialComponent>();
    auto& boundingPool = universe.GetPool<BoundingComponent>();

    // For bounding box initializing
    unsigned int triangleStartIdx = 0;

    // Iterate all Meshes
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      RenderTable renderTable;

      MeshComponent& meshComponent = meshPool.m_Data[i];
      const EntityID entityID = meshPool.m_IndexToEntity[i];

      const TransformComponent* transformComponent = transformPool.Get(entityID);
      renderTable.transformIndex = (unsigned int)m_Transforms.size();
      m_Transforms.push_back(transformComponent->transform);

      // If Mesh has never been initialized, create buffers
      if (meshComponent.VAO == 0) {
        meshComponent.VAO = Resources::CreateVertexArrayObject();
        meshComponent.VBO = Resources::CreateBuffer();
        meshComponent.EBO = Resources::CreateBuffer();
      }

      // Upload the vertex and indices of mesh to buffers
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


      Resources::UnbindVertexArrayObject();

      // Skip if no material
      if (!materialPool.Has(entityID)) {
        m_RenderTables.push_back(renderTable);
        continue;
      }

      const MaterialComponent* materialComponent = materialPool.Get(entityID);
      renderTable.materialIndex = (unsigned int)m_Materials.size();;
      m_Materials.push_back(materialComponent->material);

      // Skip if no bounding
      if (!boundingPool.Has(entityID)) {
        m_RenderTables.push_back(renderTable);
        continue;
      }

      // Add spheres / triangles / bounding boxes for ray tracing later
      BoundingComponent* boundingComponent = boundingPool.Get(entityID);

      if (boundingComponent->isSphere) {
        boundingComponent->sphere.center = transformComponent->transform.position;
        boundingComponent->sphere.radius = transformComponent->transform.scale.x * 0.5f;
        boundingComponent->sphere.material = materialComponent->material;
        m_Spheres.push_back(boundingComponent->sphere);

      } else  {

        boundingComponent->boundingBox.center = transformComponent->transform.position;
        boundingComponent->boundingBox.startTriangleIndex = triangleStartIdx;
        boundingComponent->boundingBox.size = meshComponent.mesh.indices.size() / 3;

        // Creates triangles from mesh vertices and indices
        for(size_t j = 0; j < boundingComponent->boundingBox.size; ++j) {
          Triangle triangle;
          triangle.vertexA = meshComponent.mesh.vertices[meshComponent.mesh.indices[j]];
          triangle.vertexB = meshComponent.mesh.vertices[meshComponent.mesh.indices[j+1]];
          triangle.vertexC = meshComponent.mesh.vertices[meshComponent.mesh.indices[j+2]];
          triangle.material = materialComponent->material;
          m_Triangles.push_back(triangle);
          ++triangleStartIdx;
        }
        m_BoundingBoxes.push_back(boundingComponent->boundingBox);
      }
      m_RenderTables.push_back(renderTable);
    }

  }

  void Engine::SetupCameraCache(Universe &universe) {
    auto& cameraPool = universe.GetPool<CameraComponent>();
    auto& transformPool = universe.GetPool<TransformComponent>();

    for (size_t c = 0; c < cameraPool.m_Data.size(); ++c) {
      CameraComponent& cameraComponent = cameraPool.m_Data[c];
      const EntityID entityID = transformPool.m_IndexToEntity[c];

      TransformComponent* transformComponent = transformPool.Get(entityID);

      if (cameraComponent.isActive) {
        m_ActiveCamera = &cameraComponent;
        m_ActiveCamera->camera.projection = glm::perspective(glm::radians(m_ActiveCamera->fov),
          ((float)m_WindowSize.x / (float)m_WindowSize.y), m_ActiveCamera->nearPlane, m_ActiveCamera->farPlane);
        m_ActiveCamera->camera.view = glm::inverse(transformComponent->GetModel());
        m_ActiveCamera->camera.viewInverse = m_ActiveCamera->camera.view;
        m_ActiveCamera->camera.projInverse = glm::inverse(m_ActiveCamera->camera.projection);
        break;
      }
    }

    if (m_ActiveCamera->m_UBO == 0) {
      m_ActiveCamera->m_UBO = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadUniformBufferObject<Camera>(m_ActiveCamera->camera, m_ActiveCamera->m_UBO);
      Resources::BindUniformToLocation(0, m_ActiveCamera->m_UBO);
    }
  }


  void Engine::SetupShaderStorageBuffers() {
    if (m_SSBO_RenderTables == 0) m_SSBO_RenderTables = Resources::CreateBuffer();
    if (m_SSBO_Transforms   == 0) m_SSBO_Transforms   = Resources::CreateBuffer();

    Resources::UploadShaderStorageBufferObject(m_RenderTables, m_SSBO_RenderTables);
    Resources::BindShaderStorageToLocation(2, m_SSBO_RenderTables);

    Resources::UploadShaderStorageBufferObject(m_Transforms, m_SSBO_Transforms);
    Resources::BindShaderStorageToLocation(3, m_SSBO_Transforms);
  };

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
