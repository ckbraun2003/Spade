#include "Spade/Core/Engine.hpp"

#include <ranges>

namespace Spade {

  Engine::~Engine() {
    // Delete Programs
    for (const auto &id: m_ShaderPrograms | std::views::values) {
      glDeleteProgram(id);
    }

    // Delete Buffers
    for (const auto& id : m_BufferObjects | std::views::values) {
      glDeleteBuffers(1, &id);
    }

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

    if (!m_BufferObjects.contains("Camera")) {
      m_BufferObjects["Camera"] = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadUniformBufferObject<Camera>(m_ActiveCamera.camera, m_BufferObjects["Camera"]);
      Resources::BindUniformToLocation(0, m_BufferObjects["Camera"]);
    } else {
      // Update Camera Data
      Resources::UpdateUniformBufferObject<Camera>(m_ActiveCamera.camera, m_BufferObjects["Camera"]);
    }

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


    if (!m_BufferObjects.contains("InstanceTransform")) {
      m_BufferObjects["InstanceTransform"] = Resources::CreateBuffer();
      m_BufferObjects["InstanceMotion"] = Resources::CreateBuffer();
      m_BufferObjects["InstanceMaterial"] = Resources::CreateBuffer();
      m_BufferObjects["InstanceToEntityIndex"] = Resources::CreateBuffer();

      // Upload
      Resources::UploadShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_BufferObjects["InstanceTransform"]);
      Resources::UploadShaderStorageBufferObject<Motion>(m_InstanceMotions, m_BufferObjects["InstanceMotion"]);
      Resources::UploadShaderStorageBufferObject<Material>(m_InstanceMaterials, m_BufferObjects["InstanceMaterial"]);
      Resources::UploadShaderStorageBufferObject<unsigned int>(m_InstanceToEntityIndex, m_BufferObjects["InstanceToEntityIndex"]);

      // Bind
      Resources::BindShaderStorageToLocation(5, m_BufferObjects["InstanceTransform"]);
      Resources::BindShaderStorageToLocation(6, m_BufferObjects["InstanceMotion"]);
      Resources::BindShaderStorageToLocation(7, m_BufferObjects["InstanceMaterial"]);
      Resources::BindShaderStorageToLocation(8, m_BufferObjects["InstanceToEntityIndex"]);
    } else {
      // Update

      Resources::UpdateShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_BufferObjects["InstanceTransform"]);
      Resources::UpdateShaderStorageBufferObject<Motion>(m_InstanceMotions, m_BufferObjects["InstanceMotion"]);
      Resources::UpdateShaderStorageBufferObject<Material>(m_InstanceMaterials, m_BufferObjects["InstanceMaterial"]);
      Resources::UpdateShaderStorageBufferObject<unsigned int>(m_InstanceToEntityIndex, m_BufferObjects["InstanceToEntityIndex"]);
    }
  }

  void Engine::LoadCollisionBuffers(Universe &universe) {
    
    auto& meshPool = universe.GetPool<MeshComponent>();
    auto& boundingPool = universe.GetPool<BoundingComponent>();

    std::vector<Bound> bounds;

    // Loop through each mesh
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];
      const EntityID entityID = meshPool.m_IndexToEntity[i];

      const BoundingComponent* boundingComponent = boundingPool.Get(entityID);
      bounds.push_back(boundingComponent->bound);
    }

    if (!m_BufferObjects.contains("EntityBound")) {
      m_BufferObjects["EntityBound"] = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<Bound>(bounds, m_BufferObjects["EntityBound"]);
      Resources::BindShaderStorageToLocation(4, m_BufferObjects["EntityBound"]);
    } else {
      Resources::UpdateShaderStorageBufferObject<Bound>(bounds, m_BufferObjects["EntityBound"]);
    }
  }

  void Engine::LoadFluidBuffers(Universe &universe) {

    auto& meshPool = universe.GetPool<MeshComponent>();
    auto& fluidPool = universe.GetPool<FluidComponent>();

    std::vector<FluidMaterial> fluids;

    // Loop through each mesh
    for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
      MeshComponent& meshComponent = meshPool.m_Data[i];
      const EntityID entityID = meshPool.m_IndexToEntity[i];

      const FluidComponent* fluidComponent = fluidPool.Get(entityID);
      fluids.push_back(fluidComponent->fluidMaterial);
    }

    if (!m_BufferObjects.contains("EntityFluidMaterial")) {
      m_BufferObjects["EntityFluidMaterial"] = Resources::CreateBuffer();
      // Allocate once
      Resources::UploadShaderStorageBufferObject<FluidMaterial>(fluids, m_BufferObjects["EntityFluidMaterial"]);
      Resources::BindShaderStorageToLocation(13, m_BufferObjects["EntityFluidMaterial"]);
    } else {
      Resources::UpdateShaderStorageBufferObject<FluidMaterial>(fluids, m_BufferObjects["EntityFluidMaterial"]);
    }
  }

  void Engine::LoadGridBuffers() {

    // Calculate Next Power of Two for Bitonic Sort
    size_t sortedSize = 1;
    while(sortedSize < m_InstanceTransforms.size()) sortedSize <<= 1;

    // Spatial Hash Table Size (Fixed)
    const unsigned int hashTableSize = 1 << 21;

    // Allocate fixed size hash table
    std::vector<int> emptyGrid(hashTableSize, -1);
    std::vector<GridPair> pairs(sortedSize, { 0xFFFFFFFF, 0xFFFFFFFF });

    // 3. Initialize Buffers
    if (!m_BufferObjects.contains("GridHead")) {
      m_BufferObjects["GridHead"] = Resources::CreateBuffer();
      m_BufferObjects["GridPair"] = Resources::CreateBuffer();
      m_BufferObjects["SortedTransform"] = Resources::CreateBuffer();
      m_BufferObjects["SortedMotion"] = Resources::CreateBuffer();

      Resources::UploadShaderStorageBufferObject<int>(emptyGrid, m_BufferObjects["GridHead"]);
      Resources::UploadShaderStorageBufferObject<GridPair>(pairs, m_BufferObjects["GridPair"]);
      Resources::UploadShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_BufferObjects["SortedTransform"]);
      Resources::UploadShaderStorageBufferObject<Motion>(m_InstanceMotions, m_BufferObjects["SortedMotion"]);

      Resources::BindShaderStorageToLocation(9, m_BufferObjects["GridHead"]);
      Resources::BindShaderStorageToLocation(10, m_BufferObjects["GridPair"]);
      Resources::BindShaderStorageToLocation(11, m_BufferObjects["SortedTransform"]);
      Resources::BindShaderStorageToLocation(12, m_BufferObjects["SortedMotion"]);
    } else {
      Resources::UpdateShaderStorageBufferObject<int>(emptyGrid, m_BufferObjects["GridHead"]);
      Resources::UpdateShaderStorageBufferObject<GridPair>(pairs,m_BufferObjects["GridPair"]);
      Resources::UpdateShaderStorageBufferObject<Transform>(m_InstanceTransforms, m_BufferObjects["SortedTransform"]);
      Resources::UpdateShaderStorageBufferObject<Motion>(m_InstanceMotions, m_BufferObjects["SortedMotion"]);
    }
  }

  void Engine::EnableMotion(float deltaTime) {
    if (m_InstanceMotions.empty()) return;

    if (!m_ShaderPrograms.contains("Motion")) {
      m_ShaderPrograms["Motion"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]Motion.comp");
    }

    GLuint groups = (m_InstanceMotions.size() + 63) / 64;

    Resources::UseProgram(m_ShaderPrograms["Motion"]);
    Resources::SetUniformFloat(m_ShaderPrograms["Motion"], "deltaTime", deltaTime);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableGravity(float globalGravity) {
    if (m_InstanceMotions.empty()) return;

    if (!m_ShaderPrograms.contains("Gravity")) {
      m_ShaderPrograms["Gravity"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GlobalGravity.comp");
    }

    GLuint groups = (m_InstanceMotions.size() + 63) / 64;

    Resources::UseProgram( m_ShaderPrograms["Gravity"]);
    Resources::SetUniformFloat( m_ShaderPrograms["Gravity"], "globalGravity", globalGravity);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }
  
  void Engine::BuildGrid(float globalBounds, float cellSize) {
    if (m_InstanceTransforms.empty()) return;

    // 1. Initialize Shaders
    if (!m_ShaderPrograms.contains("GridBuild")) {
      m_ShaderPrograms["GridClear"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridClear.comp");
      m_ShaderPrograms["GridBuild"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridBuild.comp");

      m_ShaderPrograms["BitonicSort"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]BitonicSort.comp");
      m_ShaderPrograms["GridOffset"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridOffsets.comp");
      m_ShaderPrograms["GridReorder"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridReorder.comp");
      m_ShaderPrograms["GridScatter"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridScatter.comp");
    }

    size_t numInstances = m_InstanceTransforms.size();

    size_t sortedSize = 1;
    while(sortedSize < numInstances) sortedSize <<= 1;

    // Spatial Hash Table Size (Fixed)
    const unsigned int hashTableSize = 1 << 21;

    GLuint groups = (numInstances + 63) / 64;
    GLuint setSizeGroups = (sortedSize + 63) / 64;

    // 1. Clear Grid (Head)
    Resources::UseProgram(m_ShaderPrograms["GridClear"]);
    Resources::SetUniformInt(m_ShaderPrograms["GridClear"], "totalCells", hashTableSize);
    glDispatchCompute((hashTableSize + 63) / 64, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    // 2. Build Key-Value Pairs
    Resources::UseProgram(m_ShaderPrograms["GridBuild"]);
    Resources::SetUniformFloat(m_ShaderPrograms["GridBuild"], "globalBounds", globalBounds);
    Resources::SetUniformFloat(m_ShaderPrograms["GridBuild"], "cellSize", cellSize);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridBuild"], "hashTableSize", hashTableSize); // Hash Table Size
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridBuild"], "numInstances", numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    // 3. Bitonic Sort (Iterative Dispatch)
    Resources::UseProgram(m_ShaderPrograms["BitonicSort"]);
    // k = block width (2, 4, 8, ... N)
    // j = comparison distance (k/2, k/4, ... 1)
    for (unsigned int k = 2; k <= sortedSize; k <<= 1) {
      for (unsigned int j = k >> 1; j > 0; j >>= 1) {
        Resources::SetUniformUnsignedInt(m_ShaderPrograms["BitonicSort"], "j", j);
        Resources::SetUniformUnsignedInt(m_ShaderPrograms["BitonicSort"], "k", k);
        glDispatchCompute(setSizeGroups, 1, 1);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
      }
    }


    // 4. Find Offsets (Populate GridHead)
    Resources::UseProgram(m_ShaderPrograms["GridOffset"]);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridOffset"], "numInstances", numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);


    // 5. Reorder (Gather)
    Resources::UseProgram(m_ShaderPrograms["GridReorder"]);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridReorder"], "numInstances", numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

  }

  void Engine::EnableSPHFluid(float globalBounds, float cellSize) {
    if (m_InstanceTransforms.empty()) return;

    // 1. Initialize Shaders
    if (!m_ShaderPrograms.contains("SPHFluidDensity")) {
      m_ShaderPrograms["SPHFluidDensity"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]FluidDensity.comp");
      m_ShaderPrograms["SPHFluidForce"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]FluidForce.comp");
    }

    BuildGrid(globalBounds, cellSize);
    const unsigned int hashTableSize = 1 << 21;

    size_t numInstances = m_InstanceTransforms.size();
    GLuint groups = (numInstances + 63) / 64;

    // Compute Density (SPH)
    Resources::UseProgram(m_ShaderPrograms["SPHFluidDensity"]);
    Resources::SetUniformFloat(m_ShaderPrograms["SPHFluidDensity"], "globalBounds", globalBounds);
    Resources::SetUniformFloat(m_ShaderPrograms["SPHFluidDensity"], "cellSize", cellSize);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["SPHFluidDensity"], "hashTableSize", hashTableSize); // Hash Table Size
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["SPHFluidDensity"], "numInstances", numInstances);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Compute Forces (SPH)
    Resources::UseProgram(m_ShaderPrograms["SPHFluidForce"]);
    Resources::SetUniformFloat(m_ShaderPrograms["SPHFluidForce"], "globalBounds", globalBounds);
    Resources::SetUniformFloat(m_ShaderPrograms["SPHFluidForce"], "cellSize", cellSize);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["SPHFluidForce"], "hashTableSize", hashTableSize); // Hash Table Size
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["SPHFluidForce"], "numInstances", numInstances);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // Scatter (Write Back)
    Resources::UseProgram(m_ShaderPrograms["GridScatter"]);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridScatter"], "numInstances", numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableBruteForceCollision(float globalBounds) {
    if (m_InstanceTransforms.empty()) return;

    if (!m_ShaderPrograms.contains("BruteForceCollision")) {
      // Only CollisionResolve is needed for Brute Force
      m_ShaderPrograms["BruteForceCollision"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]BruteForceCollision.comp");
    }

    size_t numInstances = m_InstanceTransforms.size();
    GLuint groups = (numInstances + 63) / 64;

    // 4. Resolve Collisions (Direct Brute Force)
    Resources::UseProgram(m_ShaderPrograms["BruteForceCollision"]);
    Resources::SetUniformFloat(m_ShaderPrograms["BruteForceCollision"], "globalBounds", globalBounds);

    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableGridCollision(float globalBounds, float cellSize) {
    if (m_InstanceTransforms.empty()) return;

    if (!m_ShaderPrograms.contains("GridCollision")) {
      m_ShaderPrograms["GridCollision"] = Resources::CreateComputeProgram("assets/shaders/[SYSTEM]GridCollision.comp");
    }

    BuildGrid(globalBounds, cellSize);
    const unsigned int hashTableSize = 1 << 21;

    size_t numInstances = m_InstanceTransforms.size();
    GLuint groups = (numInstances + 63) / 64;

    // Solve Collision (on Sorted Data)
    Resources::UseProgram(m_ShaderPrograms["GridCollision"]);
    Resources::SetUniformFloat(m_ShaderPrograms["GridCollision"], "globalBounds", globalBounds);
    Resources::SetUniformFloat(m_ShaderPrograms["GridCollision"], "cellSize", cellSize);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridCollision"], "hashTableSize", hashTableSize); // Hash Table Size
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridCollision"], "numInstances", numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    
    // Scatter (Write Back)
    Resources::UseProgram(m_ShaderPrograms["GridScatter"]);
    Resources::SetUniformUnsignedInt(m_ShaderPrograms["GridScatter"], "numInstances", numInstances);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  }

  void Engine::EnableBruteForceNewtonianGravity(float gravityConstant) {
    return;
  }


  void Engine::RenderWireframe() {
    RenderShader("Color", "assets/shaders/[FRAGMENT]Wireframe.frag", "assets/shaders/[GEOMETRY]Barycentric.geom");
  }

  void Engine::RenderColor() {
    RenderShader("Color", "assets/shaders/[FRAGMENT]Color.frag");
  }

  void Engine::RenderVelocity() {
    RenderShader("Velocity", "assets/shaders/[FRAGMENT]Velocity.frag");
  }

  void Engine::RenderShader(const std::string& name, const std::string& fragmentShaderFile, const std::string& geometryShaderFile) {
    if (!m_ShaderPrograms.contains(name)) {
      m_ShaderPrograms[name] = Resources::CreateRenderProgram(
      "assets/shaders/Vertex.vert",
      fragmentShaderFile,
      geometryShaderFile);
    }

    m_ActiveProgram = m_ShaderPrograms[name];
  }


  void Engine::DrawScene(Universe &universe, glm::vec4 clearColor) {
    Resources::ClearRenderBuffer(clearColor);

    auto& meshPool = universe.GetPool<MeshComponent>();

    for(auto& meshComponent : meshPool.m_Data) {
      Resources::UseProgram(m_ActiveProgram);
      Resources::SetUniformUnsignedInt(m_ActiveProgram, "instanceStartIndex", meshComponent.instanceStartIndex);

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

    if (IsKeyPressed(GLFW_KEY_M)) {
      m_IsPlaying = !m_IsPlaying;
      return;
    }

    if (IsKeyPressed(GLFW_KEY_C)) {
      m_CursorTrapped = !m_CursorTrapped;
      SetMouseCursorMode();
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

  void Engine::SetMouseCursorMode() {
      if (m_CursorTrapped) {
          glfwSetInputMode(m_GLFWwindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
      } else {
          glfwSetInputMode(m_GLFWwindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      }
  }

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

  Engine::EngineException::EngineException(const std::string &message) : runtime_error(message) {}
}
