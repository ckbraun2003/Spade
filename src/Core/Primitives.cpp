#include "Spade/Core/Primitives.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Spade {

  Object::Object(const std::string& name) {
    m_Name = name;
  }

  MeshData GenerateSphere(float radius, int sectors, int stacks) {
    MeshData data;
    float x, y, z, xy;                              // vertex position
    float nx, ny, nz, lengthInv = 1.0f / radius;    // vertex normal
    float s, t;                                     // vertex texCoord

    float sectorStep = 2 * M_PI / sectors;
    float stackStep = M_PI / stacks;
    float sectorAngle, stackAngle;

    for(int i = 0; i <= stacks; ++i) {
        stackAngle = M_PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);             // r * cos(u)
        z = radius * sinf(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        for(int j = 0; j <= sectors; ++j) {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            // vertex position (x, y, z)
            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)
            
            // normalized vertex normal (nx, ny, nz)
            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;

            // vertex tex coord (s, t) range between [0, 1]
            s = (float)j / sectors;
            t = (float)i / stacks;

            data.vertices.push_back({{x, y, z}, {nx, ny, nz}, {s, t}});
        }
    }

    // indices
    // k1--k1+1
    // |  / |
    // | /  |
    // k2--k2+1
    unsigned int k1, k2;
    for(int i = 0; i < stacks; ++i) {
        k1 = i * (sectors + 1);     // beginning of current stack
        k2 = k1 + sectors + 1;      // beginning of next stack

        for(int j = 0; j < sectors; ++j, ++k1, ++k2) {
            // 2 triangles per sector excluding first and last stacks
            // k1 => k2 => k1+1
            if(i != 0) {
                data.indices.push_back(k1);
                data.indices.push_back(k2);
                data.indices.push_back(k1 + 1);
            }

            // k1+1 => k2 => k2+1
            if(i != (stacks-1)) {
                data.indices.push_back(k1 + 1);
                data.indices.push_back(k2);
                data.indices.push_back(k2 + 1);
            }
        }
    }

    return data;
  }

  MeshData GenerateCube(float size) {
      MeshData data;
      float half = size * 0.5f;
      // 8 corners, but duplicate for normals/uvs -> 24 vertices
      // Face Normal +Z
      data.vertices.push_back({{-half, -half,  half}, {0,0,1}, {0,0}});
      data.vertices.push_back({{ half, -half,  half}, {0,0,1}, {1,0}});
      data.vertices.push_back({{ half,  half,  half}, {0,0,1}, {1,1}});
      data.vertices.push_back({{-half,  half,  half}, {0,0,1}, {0,1}});
      
      // Face Normal -Z
      data.vertices.push_back({{ half, -half, -half}, {0,0,-1}, {0,0}});
      data.vertices.push_back({{-half, -half, -half}, {0,0,-1}, {1,0}});
      data.vertices.push_back({{-half,  half, -half}, {0,0,-1}, {1,1}});
      data.vertices.push_back({{ half,  half, -half}, {0,0,-1}, {0,1}});

      // Face Normal -X
      data.vertices.push_back({{-half, -half, -half}, {-1,0,0}, {0,0}});
      data.vertices.push_back({{-half, -half,  half}, {-1,0,0}, {1,0}});
      data.vertices.push_back({{-half,  half,  half}, {-1,0,0}, {1,1}});
      data.vertices.push_back({{-half,  half, -half}, {-1,0,0}, {0,1}});

      // Face Normal +X
      data.vertices.push_back({{ half, -half,  half}, {1,0,0}, {0,0}});
      data.vertices.push_back({{ half, -half, -half}, {1,0,0}, {1,0}});
      data.vertices.push_back({{ half,  half, -half}, {1,0,0}, {1,1}});
      data.vertices.push_back({{ half,  half,  half}, {1,0,0}, {0,1}});

      // Face Normal -Y
      data.vertices.push_back({{-half, -half, -half}, {0,-1,0}, {0,0}});
      data.vertices.push_back({{ half, -half, -half}, {0,-1,0}, {1,0}});
      data.vertices.push_back({{ half, -half,  half}, {0,-1,0}, {1,1}});
      data.vertices.push_back({{-half, -half,  half}, {0,-1,0}, {0,1}});

      // Face Normal +Y
      data.vertices.push_back({{-half,  half,  half}, {0,1,0}, {0,0}});
      data.vertices.push_back({{ half,  half,  half}, {0,1,0}, {1,0}});
      data.vertices.push_back({{ half,  half, -half}, {0,1,0}, {1,1}});
      data.vertices.push_back({{-half,  half, -half}, {0,1,0}, {0,1}});

      // Indices for 6 faces * 2 tris = 12 tris = 36 indices
      unsigned int offset = 0;
      for(int i = 0; i < 6; ++i) {
          data.indices.push_back(offset + 0);
          data.indices.push_back(offset + 1);
          data.indices.push_back(offset + 2);
          
          data.indices.push_back(offset + 2);
          data.indices.push_back(offset + 3);
          data.indices.push_back(offset + 0);
          
          offset += 4;
      }
      return data;
  }
  
  MeshData GenerateQuad(float size) {
      MeshData data;
      float half = size * 0.5f;
      // Simple quad in XY plane (or XZ? usually XY for UI/2D, XZ for ground. Let's do XY facing +Z)
      data.vertices.push_back({{-half, -half, 0}, {0,0,1}, {0,0}});
      data.vertices.push_back({{ half, -half, 0}, {0,0,1}, {1,0}});
      data.vertices.push_back({{ half,  half, 0}, {0,0,1}, {1,1}});
      data.vertices.push_back({{-half,  half, 0}, {0,0,1}, {0,1}});
      
      data.indices = {0, 1, 2, 2, 3, 0};
      
      return data;
  }

}
