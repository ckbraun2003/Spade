#include "Spade/Core/Rendering.hpp"
#include "Spade/Core/Components.hpp"
#include "Spade/Core/Resources.hpp"
#include "Spade/Core/Primitives.hpp"

#include <iostream>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif

namespace Spade {

  Rendering::Rendering() {}

  void Rendering::InitializeResources() {
      SetupBuffers();

      // Create standard meshes
      ResourceManager& rm = ResourceManager::Get();
      
      // Sphere
      {
          MeshData data = GenerateSphere();
          ResourceID id = rm.CreateMesh("Sphere");
          auto* res = rm.GetResource<MeshResource>(id);
          if (res) res->Upload(data.vertices, data.indices);
      }
      // Cube
      {
          MeshData data = GenerateCube();
          ResourceID id = rm.CreateMesh("Cube");
          auto* res = rm.GetResource<MeshResource>(id);
          if (res) res->Upload(data.vertices, data.indices);
      }
      // Quad
      {
          MeshData data = GenerateQuad();
          ResourceID id = rm.CreateMesh("Quad");
          auto* res = rm.GetResource<MeshResource>(id);
          if (res) res->Upload(data.vertices, data.indices);
      }
      
      // Create a default shader
      {
          ResourceID id = rm.CreateShader("BasicShader");
          auto* shader = rm.GetResource<ShaderResource>(id);
          
          if(shader) {
             shader->LoadFiles("assets/shaders/basic.vert", "assets/shaders/basic.frag");
          }
      }
      
      // Create a default material
      {
          ResourceID id = rm.CreateMaterial("DefaultMaterial");
          auto* mat = rm.GetResource<MaterialResource>(id);
          mat->shaderID = rm.GetShaderID("BasicShader");
          mat->baseColor = {1.0f, 1.0f, 1.0f, 1.0f};
      }
  }

  void Rendering::SetupBuffers() {
    // UBO
    glGenBuffers(1, &m_UBO);
    glBindBuffer(GL_UNIFORM_BUFFER, m_UBO);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(PerFrameUniforms), nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_UBO);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    // SSBO (Initial size, will resize if needed)
    glGenBuffers(1, &m_SSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_SSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 1024 * sizeof(ObjectData), nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_SSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
  }

  void Rendering::Render(Universe &universe, Entity cameraEntity) {
      auto* camera = cameraEntity.GetComponent<Camera>();
      auto* camTransform = cameraEntity.GetComponent<Transform>();
      if (!camera || !camTransform) return;

      // 1. Update PerFrame UBO
      {
          PerFrameUniforms uniforms;
          float aspect = 1280.0f / 720.0f; // TODO: Get from Window
          uniforms.projection = glm::perspective(glm::radians(camera->fov), aspect, camera->nearPlane, camera->farPlane);

          uniforms.view = glm::inverse(camTransform->GetMatrix());

          glBindBuffer(GL_UNIFORM_BUFFER, m_UBO);
          glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(PerFrameUniforms), &uniforms);
          glBindBuffer(GL_UNIFORM_BUFFER, 0);
      }

      // 2. Collect Objects
      m_ObjectBufferCPU.clear();
      
      // Iterate MeshComponents
      auto& meshPool = universe.GetPool<MeshComponent>();
      auto& transformPool = universe.GetPool<Transform>();
      auto& materialPool = universe.GetPool<MaterialComponent>(); // Use if exists
      
      // Structure to hold draw calls: MaterialID -> MeshID -> List of Indices
      struct DrawCall {
          ResourceID matID;
          ResourceID meshID;
          std::vector<unsigned int> objectIndices;
      };

      struct RenderItem {
          ResourceID matID;
          ResourceID meshID;
          unsigned int objectIndex; // Index in SSBO
      };
      std::vector<RenderItem> renderQueue;

      unsigned int currentIndex = 0;
      
      // We iterate the map directly to get EntityIDs
      for(auto& pair : meshPool.m_EntityToIndex) {
          EntityID entityID = pair.first;
          size_t meshIndex = pair.second;
          MeshComponent& meshComp = meshPool.m_Data[meshIndex];
          
          if(meshComp.meshID == INVALID_RESOURCE_ID) continue;

          // Get Transform
          Transform* transform = transformPool.Get(entityID);
          if(!transform) continue;

          // Get Material
          ResourceID matID = INVALID_RESOURCE_ID;
          MaterialComponent* matComp = materialPool.Get(entityID);
          
          glm::vec4 color = {1,1,1,1};
          
          if(matComp) {
              matID = matComp->materialID;
              color = matComp->colorOverride; 
          }
          if(matID == INVALID_RESOURCE_ID) {
              // Fallback to default material
              matID = ResourceManager::Get().GetMaterialID("DefaultMaterial");
          }

          // Fill SSBO Data
          ObjectData objData;
          objData.model = transform->GetMatrix();
          objData.color = color;
          
          m_ObjectBufferCPU.push_back(objData);
          
          renderQueue.push_back({matID, meshComp.meshID, currentIndex});
          currentIndex++;
      }

      // 3. Upload SSBO
      if(!m_ObjectBufferCPU.empty()) {
          glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_SSBO);
          // Check size
          size_t requiredSize = m_ObjectBufferCPU.size() * sizeof(ObjectData);
          // Optimize: Only reallocate if larger. For now just BufferData.
          glBufferData(GL_SHADER_STORAGE_BUFFER, requiredSize, m_ObjectBufferCPU.data(), GL_DYNAMIC_DRAW);
          glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
      }

      // 4. Sort Render Queue (Minimize State Changes)
      // Sort by Material -> Mesh
      std::sort(renderQueue.begin(), renderQueue.end(), [](const RenderItem& a, const RenderItem& b){
          if(a.matID != b.matID) return a.matID < b.matID;
          return a.meshID < b.meshID;
      });

      // 5. Execute Draw Calls
      ResourceID currentMatID = INVALID_RESOURCE_ID;
      ResourceID currentMeshID = INVALID_RESOURCE_ID;
      ResourceManager& rm = ResourceManager::Get();

      for(const auto& item : renderQueue) {
          // Bind Material
          if(item.matID != currentMatID) {
              auto* mat = rm.GetResource<MaterialResource>(item.matID);
              if(mat) {
                  mat->ApplyState();
                  auto* shader = rm.GetResource<ShaderResource>(mat->shaderID);
                  if(shader) shader->Use();
              }
              currentMatID = item.matID;
              currentMeshID = INVALID_RESOURCE_ID; // Reset mesh binding when material changes
          }

          // Bind Mesh
          if(item.meshID != currentMeshID) {
              auto* mesh = rm.GetResource<MeshResource>(item.meshID);
              if(mesh) {
                  glBindVertexArray(mesh->VAO);
              }
              currentMeshID = item.meshID;
          }

          // Draw
          auto* mesh = rm.GetResource<MeshResource>(currentMeshID);
          if(mesh && mesh->indexCount > 0) {
              // Set Object Index
              // The shader uniform location might vary per shader if we didn't hardcode it. 
              // Assuming binding logic or layout(location) isn't used for uniform uint? 
              // Uniforms need explicit location query usually unless layout(location) is supported for uniforms (only recent GL).
              // Let's assume we need to get location for "u_ObjectIndex".
              // Optimization: Store location in ShaderResource.
              auto* mat = rm.GetResource<MaterialResource>(currentMatID);
              auto* shader = rm.GetResource<ShaderResource>(mat->shaderID);
              int loc = shader->GetUniformLocation("u_ObjectIndex");
              glUniform1ui(loc, item.objectIndex);
              
              glDrawElements(GL_TRIANGLES, mesh->indexCount, GL_UNSIGNED_INT, 0);
          }
      }
      
      glBindVertexArray(0);
  }

  Rendering::RenderException::RenderException(const std::string &message) : runtime_error(message) {}

}