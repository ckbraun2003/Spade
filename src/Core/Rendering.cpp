#include "Spade/Core/Rendering.hpp"
#include "Spade/Core/Components.hpp"
#include "Spade/Core/Resources.hpp"
#include "Spade/Core/Primitives.hpp"

#include <iostream>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <unordered_map>
#include <fstream>
#include <sstream>

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

// Typedefs (suffixed to avoid collision if glad has them but hides them)
typedef void (APIENTRY *MY_PFNGLTEXSTORAGE2DPROC) (GLenum target, GLsizei levels, GLenum internalformat, GLsizei width, GLsizei height);
typedef void (APIENTRY *MY_PFNGLBINDIMAGETEXTUREPROC) (GLuint unit, GLuint texture, GLint level, GLboolean layered, GLint layer, GLenum access, GLenum format);
typedef void (APIENTRY *MY_PFNGLDISPATCHCOMPUTEPROC) (GLuint num_groups_x, GLuint num_groups_y, GLuint num_groups_z);
typedef void (APIENTRY *MY_PFNGLMEMORYBARRIERPROC) (GLbitfield barriers);

static MY_PFNGLTEXSTORAGE2DPROC my_glTexStorage2D = nullptr;
static MY_PFNGLBINDIMAGETEXTUREPROC my_glBindImageTexture = nullptr;
static MY_PFNGLDISPATCHCOMPUTEPROC my_glDispatchCompute = nullptr;
static MY_PFNGLMEMORYBARRIERPROC my_glMemoryBarrier = nullptr;

static void EnsureComputeLoaded() {
    if(my_glDispatchCompute) return;
    my_glTexStorage2D = (MY_PFNGLTEXSTORAGE2DPROC)glfwGetProcAddress("glTexStorage2D");
    my_glBindImageTexture = (MY_PFNGLBINDIMAGETEXTUREPROC)glfwGetProcAddress("glBindImageTexture");
    my_glDispatchCompute = (MY_PFNGLDISPATCHCOMPUTEPROC)glfwGetProcAddress("glDispatchCompute");
    my_glMemoryBarrier = (MY_PFNGLMEMORYBARRIERPROC)glfwGetProcAddress("glMemoryBarrier");
}
// ------------------------------

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
          if (res) {
              res->type = ShapeType::AnalyticSphere; // Simulation optimization
              res->Upload(data.vertices, data.indices);
          }
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

  struct GPUSphere {
      glm::vec4 centerRadius; // xyz, radius
      unsigned int materialIndex;
      float padding[3];
  };

  struct GPUTriangle {
      glm::vec4 v0; // w unused
      glm::vec4 v1;
      glm::vec4 v2;
      unsigned int materialIndex;
      float padding[3];
  };

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
          uniforms.viewInverse = camTransform->GetMatrix();
          uniforms.projInverse = glm::inverse(uniforms.projection);
          
          glBindBuffer(GL_UNIFORM_BUFFER, m_UBO);
          glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(PerFrameUniforms), &uniforms);
          glBindBuffer(GL_UNIFORM_BUFFER, 0);
      }
      
      m_ObjectBufferCPU.clear();

      auto& meshPool = universe.GetPool<MeshComponent>();
      auto& transformPool = universe.GetPool<Transform>();
      auto& materialPool = universe.GetPool<MaterialComponent>();

      struct RenderItem {
          ResourceID matID;
          ResourceID meshID;
          unsigned int objectIndex; // Index in SSBO
      };
      std::vector<RenderItem> renderQueue;

      unsigned int currentIndex = 0;
      
      // We iterate the Dense vector (Fastest)
      for(size_t i = 0; i < meshPool.m_Data.size(); ++i) {
          // In Sparse Set:
          // m_Data[i] is the component.
          // m_IndexToEntity[i] is the ID.
          
          EntityID entityID = meshPool.m_IndexToEntity[i];
          MeshComponent& meshComp = meshPool.m_Data[i];
          
          if(meshComp.meshID == INVALID_RESOURCE_ID) continue;

          // Get Transform (O(1) lookup now with Sparse Set)
          Transform* transform = transformPool.Get(entityID);
          if(!transform) continue;

          // Get Material
          ResourceID matID = INVALID_RESOURCE_ID;
          MaterialComponent* matComp = materialPool.Get(entityID);
          
          glm::vec4 color = {1,1,1,1};
          
          float emission = 0.0f;
          if(matComp) {
              matID = matComp->materialID;
              color = matComp->colorOverride; 
              emission = matComp->emission;
          }
          if(matID == INVALID_RESOURCE_ID) {
              // Fallback to default material
              matID = ResourceManager::Get().GetMaterialID("DefaultMaterial");
          }

          // Fill SSBO Data
          ObjectData objData;
          objData.model = transform->GetMatrix();
          objData.color = color;
          objData.emission = emission;
          
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

  void Rendering::BuildPathTracingScene(Universe& universe) {
      if (!m_SceneDirty) return;
      m_SceneDirty = false;

      // init buffers if needed
      if(m_PTContext.materialBuffer == 0) {
           glGenBuffers(1, &m_PTContext.materialBuffer);
           glGenBuffers(1, &m_PTContext.sphereBuffer);
           glGenBuffers(1, &m_PTContext.triangleBuffer);
      }

      auto& meshPool = universe.GetPool<MeshComponent>();
      auto& transformPool = universe.GetPool<Transform>();
      auto& matPool = universe.GetPool<RayTracingMaterial>();
      
      ResourceManager& rm = ResourceManager::Get();
      
      std::vector<GPUSphere> spheres;
      std::vector<GPUTriangle> triangles;
      std::vector<RayTracingMaterial> materials;
      
      std::unordered_map<EntityID, unsigned int> entityToMatIndex;
      
      // 1. Collect Materials
      for(size_t i=0; i<matPool.m_Data.size(); ++i) {
          EntityID id = matPool.m_IndexToEntity[i];
          entityToMatIndex[id] = (unsigned int)materials.size();
          materials.push_back(matPool.m_Data[i]);
      }
      
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PTContext.materialBuffer);
      glBufferData(GL_SHADER_STORAGE_BUFFER, materials.size() * sizeof(RayTracingMaterial), materials.data(), GL_DYNAMIC_DRAW);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, m_PTContext.materialBuffer);

      // 2. Collect Geometry
      // Analytic Spheres and Triangles
      for(size_t i=0; i<meshPool.m_Data.size(); ++i) {
          EntityID id = meshPool.m_IndexToEntity[i];
          
          // STRICT RULE: Only include entities that have a RayTracingMaterial
          if(entityToMatIndex.find(id) == entityToMatIndex.end()) continue;

          MeshComponent& mesh = meshPool.m_Data[i];
          Transform* t = transformPool.Get(id);
          if(!t) continue;
          
          unsigned int matIdx = entityToMatIndex[id];
           
          auto* Res = rm.GetResource<MeshResource>(mesh.meshID);
          if(!Res) continue;

          if(Res->type == ShapeType::AnalyticSphere) {
              // Analytic Sphere Logic
              // Assume Transform Scale X is radius (uniform scale)
              GPUSphere s;
              float radius = t->scale.x * 0.5f; 
              s.centerRadius = glm::vec4(t->position, radius);
              s.materialIndex = matIdx;
              spheres.push_back(s);
          } 
          else if (Res->type == ShapeType::Mesh) {
              // Polygon Mesh Logic
              if(!Res->indices.empty()) {
                  glm::mat4 model = t->GetMatrix();
                  const auto& indices = Res->indices;
                  const auto& vertices = Res->vertices;
                  
                  for(size_t j=0; j<indices.size(); j+=3) {
                      GPUTriangle tri;
                      glm::vec3 v0 = vertices[indices[j]].position;
                      glm::vec3 v1 = vertices[indices[j+1]].position;
                      glm::vec3 v2 = vertices[indices[j+2]].position;
                      
                      tri.v0 = model * glm::vec4(v0, 1.0f);
                      tri.v1 = model * glm::vec4(v1, 1.0f);
                      tri.v2 = model * glm::vec4(v2, 1.0f);
                      tri.materialIndex = matIdx;
                      triangles.push_back(tri);
                  }
              }
          }
          // Box? Not implemented yet in Shader
      }
      
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PTContext.sphereBuffer);
      glBufferData(GL_SHADER_STORAGE_BUFFER, spheres.size() * sizeof(GPUSphere), spheres.data(), GL_DYNAMIC_DRAW);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, m_PTContext.sphereBuffer);

      glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PTContext.triangleBuffer);
      glBufferData(GL_SHADER_STORAGE_BUFFER, triangles.size() * sizeof(GPUTriangle), triangles.data(), GL_DYNAMIC_DRAW);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, m_PTContext.triangleBuffer);
  }

  // Helper for Compute Shader
  #ifndef GL_COMPUTE_SHADER
  #define GL_COMPUTE_SHADER 0x91B9
  #endif
  
  #ifndef GL_SHADER_STORAGE_BARRIER_BIT
  #define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000
  #endif

  #ifndef GL_SHADER_IMAGE_ACCESS_BARRIER_BIT 
  #define GL_SHADER_IMAGE_ACCESS_BARRIER_BIT 0x00000020
  #endif

  static unsigned int CreateComputeShader(const char* path) {
      std::string code;
      try {
          std::ifstream file(path);
          std::stringstream ss;
          ss << file.rdbuf();
          code = ss.str();
      } catch(...) { return 0; }
      
      const char* src = code.c_str();
      unsigned int shader = glCreateShader(GL_COMPUTE_SHADER);
      glShaderSource(shader, 1, &src, nullptr);
      glCompileShader(shader);
      
      int success;
      glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
      if(!success) {
          char info[512];
          glGetShaderInfoLog(shader, 512, nullptr, info);
          printf("Compute Error: %s\n", info);
      }
      
      unsigned int program = glCreateProgram();
      glAttachShader(program, shader);
      glLinkProgram(program);
      return program;
  }

  void Rendering::RenderPathTracing(Universe& universe, Entity cameraEntity) {
      BuildPathTracingScene(universe); 

      auto* camera = cameraEntity.GetComponent<Camera>();
      auto* camTransform = cameraEntity.GetComponent<Transform>();
      if (!camera || !camTransform) return;

      static unsigned int rayGenProgram = 0;
      static unsigned int intersectProgram = 0;
      static unsigned int shadeProgram = 0;
      
      static unsigned int screenTex = 0;
      static unsigned int blitVAO = 0;
      static unsigned int blitProgram = 0;
      
      static unsigned int rayBuffer = 0;
      static unsigned int hitBuffer = 0;

      static glm::ivec2 currentRes = {0,0};

      // Re-init resources if resolution changed
      bool resChanged = (currentRes != m_RenderResolution);
      if(m_RenderResolution.x <= 0 || m_RenderResolution.y <= 0) m_RenderResolution = {1280, 720}; // Safety

      static glm::mat4 lastView = glm::mat4(0.0f);
      glm::mat4 currentView = camTransform->GetMatrix();
      
      bool cameraMoved = (lastView != currentView);
      if(cameraMoved) {
          m_FrameCount = 0; // Reset
          lastView = currentView;
      }

      EnsureComputeLoaded();
      if(rayGenProgram == 0 || resChanged || cameraMoved) { // Re-init shader uniforms isn't needed, but Blit might be? No.
          // Just reset framecount updates u_Reset in loop later.
          // Wait, if camera moved, m_FrameCount=0.
          // resChanged logic logic triggers buffer rebuild.
      }
      
      if(rayGenProgram == 0 || resChanged) { // Only rebuild resources if Res changed
          currentRes = m_RenderResolution;
          
          if(rayGenProgram == 0) {
              rayGenProgram = CreateComputeShader("assets/shaders/raytracing/raygen.comp");
              intersectProgram = CreateComputeShader("assets/shaders/raytracing/intersect.comp");
              shadeProgram = CreateComputeShader("assets/shaders/raytracing/shade.comp");
              
              // Blit Quad Init
              glGenVertexArrays(1, &blitVAO);
              const char* vs = "#version 330 core\nlayout(location=0) in vec2 aPos; out vec2 TexCoords; void main(){TexCoords=(aPos+1.0)/2.0; gl_Position=vec4(aPos,0,1);}";
              const char* fs = "#version 330 core\nout vec4 C; in vec2 TexCoords; uniform sampler2D t; uniform float u_Samples; void main(){ C = texture(t,TexCoords) / u_Samples; C.a = 1.0; }";
              unsigned int v = glCreateShader(GL_VERTEX_SHADER); glShaderSource(v,1,&vs,0); glCompileShader(v);
              unsigned int f = glCreateShader(GL_FRAGMENT_SHADER); glShaderSource(f,1,&fs,0); glCompileShader(f);
              blitProgram = glCreateProgram(); glAttachShader(blitProgram,v); glAttachShader(blitProgram,f); glLinkProgram(blitProgram);
              float q[] = {-1,-1, 1,-1, -1,1, 1,1};
              unsigned int vbo; glGenBuffers(1,&vbo); glBindBuffer(GL_ARRAY_BUFFER,vbo); glBufferData(GL_ARRAY_BUFFER,sizeof(q),q,GL_STATIC_DRAW);
              glBindVertexArray(blitVAO); glVertexAttribPointer(0,2,GL_FLOAT,0,0,0); glEnableVertexAttribArray(0);
          }
          
          // Recreate Texture
          if(screenTex) glDeleteTextures(1, &screenTex);
          glGenTextures(1, &screenTex);
          glBindTexture(GL_TEXTURE_2D, screenTex);
          if(my_glTexStorage2D) my_glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA32F, currentRes.x, currentRes.y);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
          
          // Recreate Buffers
          size_t pixelCount = currentRes.x * currentRes.y;
          
          if(rayBuffer) glDeleteBuffers(1, &rayBuffer);
          glGenBuffers(1, &rayBuffer);
          glBindBuffer(GL_SHADER_STORAGE_BUFFER, rayBuffer);
          glBufferData(GL_SHADER_STORAGE_BUFFER, pixelCount * 64, nullptr, GL_DYNAMIC_DRAW); 
          
          if(hitBuffer) glDeleteBuffers(1, &hitBuffer);
          glGenBuffers(1, &hitBuffer);
          glBindBuffer(GL_SHADER_STORAGE_BUFFER, hitBuffer);
          glBufferData(GL_SHADER_STORAGE_BUFFER, pixelCount * 48, nullptr, GL_DYNAMIC_DRAW); 
      }

      // 1. Update Camera
      {
          PerFrameUniforms uniforms;
          // Use render resolution aspect or window aspect? 
          // Usually window aspect, but if we render stretched, use render resolution aspect?
          // Let's use Render Resolution Aspect to keep pixels square.
          float aspect = (float)currentRes.x / (float)currentRes.y; 
          uniforms.projection = glm::perspective(glm::radians(camera->fov), aspect, camera->nearPlane, camera->farPlane);
          uniforms.view = glm::inverse(camTransform->GetMatrix());
          uniforms.viewInverse = camTransform->GetMatrix();
          uniforms.projInverse = glm::inverse(uniforms.projection);
          
          glBindBuffer(GL_UNIFORM_BUFFER, m_UBO);
          glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(PerFrameUniforms), &uniforms);
          glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_UBO);
      }
      
      // Bind Shared Resources
      if(my_glBindImageTexture) my_glBindImageTexture(0, screenTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F); 
      
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, rayBuffer);       
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, hitBuffer);       
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, m_PTContext.sphereBuffer);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, m_PTContext.triangleBuffer);
      glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, m_PTContext.materialBuffer);
      
      int sphereSize = 0; glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PTContext.sphereBuffer); glGetBufferParameteriv(GL_SHADER_STORAGE_BUFFER, GL_BUFFER_SIZE, &sphereSize);
      int triSize = 0; glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_PTContext.triangleBuffer); glGetBufferParameteriv(GL_SHADER_STORAGE_BUFFER, GL_BUFFER_SIZE, &triSize);
      
      uint32_t numSpheres = sphereSize / 32; 
      uint32_t numTris = triSize / 64; 
      
      if(!my_glDispatchCompute || !my_glMemoryBarrier) return;

      // Calculate groups (16x16 local size)
      GLuint groupsX = (currentRes.x + 15) / 16;
      GLuint groupsY = (currentRes.y + 15) / 16;
      // Also update Shaders to use local_size 16x16 if not already. (I did that in previous steps).

      // 2. Dispatch Pipeline
      
      // Update Uniforms
      glUseProgram(rayGenProgram);
      // glProgramUniform1ui(rayGenProgram, ...); // If we had DSA. Use glUniform for now.
      
      for(int sample = 0; sample < m_RaysPerFrame; ++sample) {
          m_FrameCount++; // Increment GLOBAL frame count for RNG seed variation
          
          // A. Ray Generation
          glUseProgram(rayGenProgram);
          glUniform1ui(glGetUniformLocation(rayGenProgram, "u_FrameCount"), m_FrameCount);
          glUniform1ui(glGetUniformLocation(rayGenProgram, "u_Reset"), (m_FrameCount == 1) ? 1 : 0);
          my_glDispatchCompute(groupsX, groupsY, 1);
          my_glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
          
          // B. Bounce Loop
          for(int i=0; i<m_MaxBounces; ++i) {
               // Intersect
               glUseProgram(intersectProgram);
               glUniform1ui(glGetUniformLocation(intersectProgram, "u_NumSpheres"), numSpheres);
               glUniform1ui(glGetUniformLocation(intersectProgram, "u_NumTriangles"), numTris);
               my_glDispatchCompute(groupsX, groupsY, 1); 
               my_glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
               
               // Shade
               glUseProgram(shadeProgram);
               glUniform1ui(glGetUniformLocation(shadeProgram, "u_FrameCount"), m_FrameCount); // Pass Seed
               my_glDispatchCompute(groupsX, groupsY, 1); 
               my_glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
          }
      }
      
      // 3. Blit to Screen
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glViewport(0, 0, 1280, 720); // Assumption for now
      glUseProgram(blitProgram);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, screenTex);
      glUniform1i(glGetUniformLocation(blitProgram, "t"), 0);
      glUniform1f(glGetUniformLocation(blitProgram, "u_Samples"), (float)(m_FrameCount == 0 ? 1 : m_FrameCount)); // Prevent div by zero
      glBindVertexArray(blitVAO);
      glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  }
} // namespace Spade