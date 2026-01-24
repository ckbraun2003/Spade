#include "Spade/Core/Engine.hpp"

namespace Spade {

  Engine::~Engine() {
    glDeleteBuffers(1, &m_SSBO_InstanceTransforms);
    glDeleteBuffers(1, &m_SSBO_InstanceMaterials);
    glDeleteBuffers(1, &m_SSBO_InstanceMotions);
    glDeleteBuffers(1, &m_SSBO_InstanceToEntityIndex);
    
    glDeleteBuffers(1, &m_SSBO_EntityBounds);
    glDeleteBuffers(1, &m_SSBO_EntityFluidMaterials);

    glDeleteBuffers(1, &m_SSBO_GridHead);
    glDeleteBuffers(1, &m_SSBO_GridPairs);
    glDeleteBuffers(1, &m_SSBO_SortedTransforms);
    glDeleteBuffers(1, &m_SSBO_SortedMotions);

    glDeleteBuffers(1, &m_UBO_Camera);

    glDeleteProgram(m_ShaderProgram);
    glDeleteProgram(m_MotionProgram);
    glDeleteProgram(m_CollisionProgram);
    glDeleteProgram(m_GridCollisionProgram);
    glDeleteProgram(m_GravityProgram);
    glDeleteProgram(m_FluidDensityProgram);
    glDeleteProgram(m_FluidForceProgram);

    glDeleteProgram(m_GridClearProgram);
    glDeleteProgram(m_GridBuildProgram);
    glDeleteProgram(m_BitonicSortProgram);
    glDeleteProgram(m_GridOffsetsProgram);
    glDeleteProgram(m_GridReorderProgram);
    glDeleteProgram(m_GridScatterProgram);

    glfwDestroyWindow(m_GLFWwindow);
    glfwTerminate();
  }

  void Engine::LoadCameraBuffers(Universe &universe) {
    auto& cameraPool = universe.GetPool<CameraComponent>();
    auto& transformPool = universe.GetPool<TransformComponent>();

    for (size_t c = 0; c < cameraPool.m_Data.size(); ++c) {
      CameraComponent& cameraComponent = cameraPool.m_Data[c];
      const EntityID entityID = cameraPool.m_IndexToEntity[c];

      TransformComponent* transformComponent = transformPool.Get(entityID);

      if (cameraComponent.isActive) {

        m_ActiveCamera.camera.projection = glm::perspective(glm::radians(cameraComponent.fov),
          ((float)m_WindowSize.x / (float)m_WindowSize.y), cameraComponent.nearPlane, cameraComponent.farPlane);
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

  void Engine::LoadInstanceBuffers(Universe &universe) {

    m_InstanceTransforms.clear();
    m_InstanceMotions.clear();
    m_InstanceMaterials.clear();
    m_InstanceToEntityIndex.clear();
    
    auto& meshPool = universe.GetPool<MeshComponent>();

    unsigned int bufferSize = 0;

    for(auto & meshComponent : meshPool.m_Data) {
      bufferSize+= meshComponent.instanceTransforms.size();
    }

    m_InstanceTransforms.reserve(bufferSize);
    m_InstanceMotions.reserve(bufferSize);
    m_InstanceMaterials.reserve(bufferSize);
    m_InstanceToEntityIndex.reserve(bufferSize);

    // Loop through each mesh
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];

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

      meshComponent.instanceStartIndex = m_InstanceToEntityIndex.size();
      // Insert Values into buffers
      m_InstanceTransforms.insert(m_InstanceTransforms.end(), meshComponent.instanceTransforms.begin(), meshComponent.instanceTransforms.end());
      m_InstanceMotions.insert(m_InstanceMotions.end(), meshComponent.instanceMotions.begin(), meshComponent.instanceMotions.end());
      m_InstanceMaterials.insert(m_InstanceMaterials.end(), meshComponent.instanceMaterials.begin(), meshComponent.instanceMaterials.end());
      m_InstanceToEntityIndex.insert(m_InstanceToEntityIndex.end(), meshComponent.instanceTransforms.size(), i);
    }


    if (m_SSBO_InstanceTransforms == 0) {
      m_SSBO_InstanceTransforms = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_SSBO_InstanceTransforms);
      Resources::BindShaderStorageToLocation(5, m_SSBO_InstanceTransforms);
    }

    if (m_SSBO_InstanceMotions == 0) {
      m_SSBO_InstanceMotions = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Motion>(m_InstanceMotions, m_SSBO_InstanceMotions);
      Resources::BindShaderStorageToLocation(6, m_SSBO_InstanceMotions);
    }

    if (m_SSBO_InstanceMaterials == 0) {
      m_SSBO_InstanceMaterials = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Material>(m_InstanceMaterials, m_SSBO_InstanceMaterials);
      Resources::BindShaderStorageToLocation(7, m_SSBO_InstanceMaterials);
    }

    if (m_SSBO_InstanceToEntityIndex == 0) {
      m_SSBO_InstanceToEntityIndex = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<unsigned int>(m_InstanceToEntityIndex, m_SSBO_InstanceToEntityIndex);
      Resources::BindShaderStorageToLocation(8, m_SSBO_InstanceToEntityIndex);
    }

    Resources::UpdateShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_SSBO_InstanceTransforms);
    Resources::UpdateShaderStorageBufferObject<Motion>(m_InstanceMotions, m_SSBO_InstanceMotions);
    Resources::UpdateShaderStorageBufferObject<Material>(m_InstanceMaterials, m_SSBO_InstanceMaterials);
    Resources::UpdateShaderStorageBufferObject<unsigned int>(m_InstanceToEntityIndex, m_SSBO_InstanceToEntityIndex);
  }

  void Engine::LoadCollisionBuffers(Universe &universe) {
    
    auto& meshPool = universe.GetPool<MeshComponent>();
    auto& boundingPool = universe.GetPool<BoundingComponent>();

    std::vector<Bound> bounds;

    // Loop through each mesh
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];
      const EntityID entityID = meshPool.m_IndexToEntity[i];

      BoundingComponent* boundingComponent = boundingPool.Get(entityID);
      bounds.push_back(boundingComponent->bound);
    }

    if (m_SSBO_EntityBounds == 0) {
      m_SSBO_EntityBounds = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Bound>(bounds, m_SSBO_EntityBounds);
      Resources::BindShaderStorageToLocation(4, m_SSBO_EntityBounds);
    }

    Resources::UpdateShaderStorageBufferObject<Bound>(bounds, m_SSBO_EntityBounds);
    }

  void Engine::LoadFluidBuffers(Universe &universe) {

    auto& meshPool = universe.GetPool<MeshComponent>();
    auto& fluidPool = universe.GetPool<FluidComponent>();

    std::vector<FluidMaterial> fluids;

    // Loop through each mesh
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];
      const EntityID entityID = meshPool.m_IndexToEntity[i];

      FluidComponent* fluidComponent = fluidPool.Get(entityID);
      fluids.push_back(fluidComponent->fluidMaterial);
    }

    if (m_SSBO_EntityFluidMaterials == 0) {
      m_SSBO_EntityFluidMaterials = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<FluidMaterial>(fluids, m_SSBO_EntityFluidMaterials);
      Resources::BindShaderStorageToLocation(13, m_SSBO_EntityFluidMaterials);
    }

    Resources::UpdateShaderStorageBufferObject<FluidMaterial>(fluids, m_SSBO_EntityFluidMaterials);
  }

  void Engine::EnableMotion(float deltaTime) {
    if (m_InstanceMotions.empty()) return;

    if (m_MotionProgram == 0) {
      m_MotionProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]Motion.comp");
    }

    GLuint groups = (m_InstanceMotions.size() + 63) / 64;

    Resources::UseProgram(m_MotionProgram);
    Resources::SetLocationFloat(4, deltaTime);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableGravity(float globalGravity, float deltaTime) {
    if (m_InstanceMotions.empty()) return;

    if (m_GravityProgram == 0) {
      m_GravityProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GlobalGravity.comp");
    }

    GLuint groups = (m_InstanceMotions.size() + 63) / 64;

    Resources::UseProgram(m_GravityProgram);
    Resources::SetLocationFloat(4, deltaTime);
    Resources::SetLocationFloat(14, globalGravity);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableBruteForceCollision(float globalBounds, float deltaTime) {
    if (m_InstanceTransforms.empty()) return;

    if (m_CollisionProgram == 0) {
        // Only CollisionResolve is needed for Brute Force
        m_CollisionProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]BruteForceCollision.comp");
    }

    size_t numInstances = m_InstanceTransforms.size();
    GLuint groups = (numInstances + 63) / 64;

    // 4. Resolve Collisions (Direct Brute Force)
    Resources::UseProgram(m_CollisionProgram);
    Resources::SetLocationFloat(4, deltaTime);
    Resources::SetLocationFloat(15, globalBounds);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }
  
  void Engine::BuildGrid(float globalBounds, float cellSize) {
    if (m_InstanceTransforms.empty()) return;

    // 1. Initialize Shaders
    if (m_GridClearProgram == 0) {
      m_GridClearProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridClear.comp");
      m_GridBuildProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridBuild.comp");

      m_BitonicSortProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]BitonicSort.comp");
      m_GridOffsetsProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridOffsets.comp");
      m_GridReorderProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridReorder.comp");
      m_GridScatterProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridScatter.comp");
    }

    size_t numInstances = m_InstanceTransforms.size();

    // Calculate Next Power of Two for Bitonic Sort
    size_t sortedSize = 1;
    while(sortedSize < numInstances) sortedSize <<= 1;

    GLuint groups = (numInstances + 63) / 64;
    GLuint setSizeGroups = (sortedSize + 63) / 64;

    // Spatial Hash Table Size (Fixed)
    const unsigned int hashTableSize = 1 << 21;

    // 3. Initialize Buffers
    if (m_SSBO_GridHead == 0) {
      m_SSBO_GridHead = Resources::CreateBuffer();
      // Allocate fixed size hash table
      std::vector<int> emptyGrid(hashTableSize, -1);
      Resources::UploadShaderStorageBufferObject<int>(emptyGrid, m_SSBO_GridHead);
      Resources::BindShaderStorageToLocation(9, m_SSBO_GridHead);
    }

    if (m_SSBO_GridPairs == 0) {
      m_SSBO_GridPairs = Resources::CreateBuffer();
      // Initialize with MAX_UINT so padding sorts to the end
      std::vector<GridPair> pairs(sortedSize, { 0xFFFFFFFF, 0xFFFFFFFF });
      Resources::UploadShaderStorageBufferObject<GridPair>(pairs, m_SSBO_GridPairs);
      Resources::BindShaderStorageToLocation(10, m_SSBO_GridPairs);
    }

    if (m_SSBO_SortedTransforms == 0) {
      m_SSBO_SortedTransforms = Resources::CreateBuffer();
      Resources::UploadShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_SSBO_SortedTransforms);
      Resources::BindShaderStorageToLocation(11, m_SSBO_SortedTransforms);
    }

    if (m_SSBO_SortedMotions == 0) {
      m_SSBO_SortedMotions = Resources::CreateBuffer();
      Resources::UploadShaderStorageBufferObject<Motion>(m_InstanceMotions, m_SSBO_SortedMotions);
      Resources::BindShaderStorageToLocation(12, m_SSBO_SortedMotions);
    }

    // 1. Clear Grid (Head)
    Resources::UseProgram(m_GridClearProgram);
    Resources::SetLocationInt(11, hashTableSize);
    glDispatchCompute((hashTableSize + 63) / 64, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    // 2. Build Key-Value Pairs
    Resources::UseProgram(m_GridBuildProgram);
    Resources::SetLocationFloat(15, globalBounds);
    Resources::SetLocationFloat(16, cellSize);
    Resources::SetLocationUnsignedInt(17, hashTableSize); // Hash Table Size
    Resources::SetLocationUnsignedInt(18, numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    // 3. Bitonic Sort (Iterative Dispatch)
    Resources::UseProgram(m_BitonicSortProgram);
    // k = block width (2, 4, 8, ... N)
    // j = comparison distance (k/2, k/4, ... 1)
    for (unsigned int k = 2; k <= sortedSize; k <<= 1) {
      for (unsigned int j = k >> 1; j > 0; j >>= 1) {
        Resources::SetLocationUnsignedInt(0, j);
        Resources::SetLocationUnsignedInt(1, k);
        glDispatchCompute(setSizeGroups, 1, 1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
      }
    }


    // 4. Find Offsets (Populate GridHead)
    Resources::UseProgram(m_GridOffsetsProgram);
    Resources::SetLocationUnsignedInt(18, numInstances); // Only process valid particles
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    // 5. Reorder (Gather)
    Resources::UseProgram(m_GridReorderProgram);
    Resources::SetLocationUnsignedInt(18, numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  }

  void Engine::EnableSPHFluid(float globalBounds, float cellSize, float deltaTime) {
    if (m_InstanceTransforms.empty()) return;

    // 1. Initialize Shaders
    if (m_FluidDensityProgram == 0) {
      m_FluidDensityProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]FluidDensity.comp");
      m_FluidForceProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]FluidForce.comp");
    }

    BuildGrid(globalBounds, cellSize);
    const unsigned int hashTableSize = 1 << 21;

    size_t numInstances = m_InstanceTransforms.size();
    GLuint groups = (numInstances + 63) / 64;

    // Compute Density (SPH)
    Resources::UseProgram(m_FluidDensityProgram);
    Resources::SetLocationFloat(15, globalBounds);
    Resources::SetLocationFloat(16, cellSize);
    Resources::SetLocationUnsignedInt(17, hashTableSize);
    Resources::SetLocationUnsignedInt(18, numInstances);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Compute Forces (SPH)
    Resources::UseProgram(m_FluidForceProgram);
    Resources::SetLocationFloat(4, deltaTime);
    Resources::SetLocationFloat(15, globalBounds);
    Resources::SetLocationFloat(16, cellSize);
    Resources::SetLocationUnsignedInt(17, hashTableSize);
    Resources::SetLocationUnsignedInt(18, numInstances);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Scatter (Write Back)
    Resources::UseProgram(m_GridScatterProgram);
    Resources::SetLocationUnsignedInt(18, numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableGridCollision(float globalBounds, float cellSize, float deltaTime) {
    if (m_InstanceTransforms.empty()) return;

    if (m_GridCollisionProgram == 0) {
      m_GridCollisionProgram = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridCollision.comp");
    }

    BuildGrid(globalBounds, cellSize);
    const unsigned int hashTableSize = 1 << 21;

    size_t numInstances = m_InstanceTransforms.size();
    GLuint groups = (numInstances + 63) / 64;

    // Solve Collision (on Sorted Data)
    Resources::UseProgram(m_GridCollisionProgram);
    Resources::SetLocationFloat(4, deltaTime);
    Resources::SetLocationFloat(15, globalBounds);
    Resources::SetLocationFloat(16, cellSize);
    Resources::SetLocationUnsignedInt(17, hashTableSize);
    Resources::SetLocationUnsignedInt(18, numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    
    // Scatter (Write Back)
    Resources::UseProgram(m_GridScatterProgram);
    Resources::SetLocationUnsignedInt(18, numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::DrawScene(Universe &universe, const std::string& fragmentShaderFile = "assets/shaders/[FRAGMENT]Color.frag") {
    if (m_ShaderProgram == 0) {
      m_ShaderProgram = Resources::CreateRenderProgram("assets/shaders/Vertex.vert", fragmentShaderFile);
    }

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto& meshPool = universe.GetPool<MeshComponent>();

    for(auto& meshComponent : meshPool.m_Data) {
      Resources::UseProgram(m_ShaderProgram);

      Resources::SetLocationUnsignedInt(3, meshComponent.instanceStartIndex);

      Resources::BindVertexArrayObject(meshComponent.VAO);
      glDrawElementsInstanced(GL_TRIANGLES, meshComponent.mesh.indices.size(), GL_UNSIGNED_INT, 0, meshComponent.instanceTransforms.size());

      Resources::UnbindVertexArrayObject();
    }

    glfwSwapBuffers(m_GLFWwindow);
    glfwPollEvents();

    UpdateStatistics();

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
          LoadCameraBuffers(universe);
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

    if(glDispatchCompute) return;
    glTexStorage2D = (MY_PFNGLTEXSTORAGE2DPROC)glfwGetProcAddress("glTexStorage2D");
    glBindImageTexture = (MY_PFNGLBINDIMAGETEXTUREPROC)glfwGetProcAddress("glBindImageTexture");
    glDispatchCompute = (MY_PFNGLDISPATCHCOMPUTEPROC)glfwGetProcAddress("glDispatchCompute");
    glMemoryBarrier = (MY_PFNGLMEMORYBARRIERPROC)glfwGetProcAddress("glMemoryBarrier");
  }

  void Engine::UpdateStatistics() {
    // Update Time and FPS variables
    m_CurrentTime = GetTime();
    m_DeltaTime = m_CurrentTime - m_LastTime;
    m_LastTime = m_CurrentTime;

    // Cap DeltaTime to prevent physics explosions during lag spikes
    // 0.05f = 20 FPS. If framerate drops below this, simulation slows down instead of exploding.
    if (m_DeltaTime > 0.05f) m_DeltaTime = 0.05f;

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
  
  // Private non-const version
  bool Engine::IsKeyPressed(int key) {
    return glfwGetKey(m_GLFWwindow, key) == GLFW_PRESS;
  }

  Engine::EngineException::EngineException(const std::string &message) : runtime_error(message) {}
}
