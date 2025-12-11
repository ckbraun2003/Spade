#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <vector>
#include <unordered_map>

#include "Spade/Core/Primitives.hpp"

namespace Spade {

  // --- Type IDs ---
  using ComponentTypeID = std::size_t;
  using EntityID = unsigned int;
  static const EntityID INVALID_ENTITY_ID = 0xFFFFFFFF;

  namespace Internal {
      inline ComponentTypeID GetUniqueComponentID() {
          static std::atomic<ComponentTypeID> lastID{0};
          return lastID++;
      }
  }

  template<typename T>
  inline ComponentTypeID GetComponentTypeID() {
      static ComponentTypeID typeID = Internal::GetUniqueComponentID();
      return typeID;
  }

  class IComponentPool {
  public:
      virtual ~IComponentPool() = default;
      virtual void Remove(EntityID entity) = 0;
  };

  template<typename T>
  class ComponentPool : public IComponentPool {
  public:
      // Dense Data
      std::vector<T> m_Data;
      
      // Dense Index -> EntityID (for reverse lookup during iteration)
      std::vector<EntityID> m_IndexToEntity;

      // Sparse EntityID -> Index (Direct Lookup)
      // If m_EntityToIndex[entityID] == INVALID_INDEX, it has no component.
      // We reserve enough space for max entities.
      std::vector<size_t> m_EntityToIndex;
      
      static constexpr size_t INVALID_INDEX = 0xFFFFFFFF;

      ComponentPool() {
          // Reserve space for a reasonable amount of entities to avoid reallocs
          // In a production engine, this would be a Page allocator or huge reserve.
          m_EntityToIndex.resize(10000, INVALID_INDEX); 
      }

      T& Add(EntityID entity, T component) {
          if (Has(entity)) {
              // Replace existing
              m_Data[m_EntityToIndex[entity]] = std::move(component);
              return m_Data[m_EntityToIndex[entity]];
          }

          // Ensure Sparse Array is big enough
          if (entity >= m_EntityToIndex.size()) {
              m_EntityToIndex.resize(entity + 1000, INVALID_INDEX);
          }

          size_t index = m_Data.size();
          m_Data.push_back(std::move(component));
          m_IndexToEntity.push_back(entity);
          m_EntityToIndex[entity] = index;
          
          return m_Data.back();
      }

      void Remove(EntityID entity) override {
          if (!Has(entity)) return;

          size_t indexToRemove = m_EntityToIndex[entity];
          size_t lastIndex = m_Data.size() - 1;
          EntityID lastEntity = m_IndexToEntity[lastIndex];

          // Swap-and-pop Dense Data
          m_Data[indexToRemove] = std::move(m_Data[lastIndex]);
          m_IndexToEntity[indexToRemove] = lastEntity;

          // Update Sparse Map
          m_EntityToIndex[lastEntity] = indexToRemove;
          m_EntityToIndex[entity] = INVALID_INDEX;

          m_Data.pop_back();
          m_IndexToEntity.pop_back();
      }

      T* Get(EntityID entity) {
          if (entity >= m_EntityToIndex.size()) return nullptr;
          size_t index = m_EntityToIndex[entity];
          if (index == INVALID_INDEX) return nullptr;
          return &m_Data[index];
      }
      
      bool Has(EntityID entity) const {
          if (entity >= m_EntityToIndex.size()) return false;
          return m_EntityToIndex[entity] != INVALID_INDEX;
      }
  };

  class Universe : public Object {
  public:
      std::unordered_map<ComponentTypeID, std::unique_ptr<IComponentPool>> m_Pools;

      template<typename T>
      ComponentPool<T>& GetPool() {
          ComponentTypeID id = GetComponentTypeID<T>();
          if (m_Pools.find(id) == m_Pools.end()) {
              m_Pools[id] = std::make_unique<ComponentPool<T>>();
          }
          return static_cast<ComponentPool<T>&>(*m_Pools[id]);
      }
      
      EntityID m_NextEntityID = 0;
      EntityID CreateEntityID() { return m_NextEntityID++; }
      
      // Clear function?
  };

  class Entity : public Object {
  public:
      EntityID m_Id;
      Universe* m_Universe;

      Entity() : m_Id(INVALID_ENTITY_ID), m_Universe(nullptr) {}
      Entity(EntityID id, Universe* universe) : m_Id(id), m_Universe(universe) {}

      EntityID GetID() const { return m_Id; }
      bool IsValid() const { return m_Id != INVALID_ENTITY_ID && m_Universe != nullptr; }

      bool operator==(const Entity& other) const { return m_Id == other.m_Id && m_Universe == other.m_Universe; }
      bool operator!=(const Entity& other) const { return !(*this == other); }

      template<typename T>
      T* AddComponent(T component = T()) {
          if (!IsValid()) return nullptr;
          return &m_Universe->GetPool<T>().Add(m_Id, std::move(component));
      }

      template<typename T>
      T* GetComponent() {
          if (!IsValid()) return nullptr;
          return m_Universe->GetPool<T>().Get(m_Id);
      }
      
      template<typename T>
      bool HasComponent() {
           if (!IsValid()) return false;
           return m_Universe->GetPool<T>().Has(m_Id);
      }
  };

}