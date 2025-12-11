#include "Spade/Systems/ScriptSystem.hpp"
#include "Spade/Core/Components.hpp"

namespace Spade {

    void ScriptSystem::Update(Universe& universe, float deltaTime) {
        auto& scriptPool = universe.GetPool<NativeScriptComponent>();
        
        // Iterating Dense Data directly (Fastest)
        // Need EntityID though? Yes, script callback takes Entity.
        // Assuming SparseSet Refactor in Objects.hpp will provide index->entity mapping.
        // Current map-based: iterate m_Data is fine, but need EntityID.
        // map-based: m_IndexToEntity[i] works.
        
        for(size_t i = 0; i < scriptPool.m_Data.size(); ++i) {
            NativeScriptComponent& script = scriptPool.m_Data[i];
            if (script.OnUpdate) {
               // We need the EntityID for this component index
               // Objects.hpp (Map version) has m_IndexToEntity
               // Objects.hpp (Sparse version) will also have m_Dense (Index->Entity)
               // So this code is forward-compatible!
               
                EntityID id = scriptPool.m_IndexToEntity[i];
                script.OnUpdate(Entity(id, &universe), deltaTime);
            }
        }
    }

}
