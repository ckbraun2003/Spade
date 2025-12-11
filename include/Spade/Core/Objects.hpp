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
      std::vector<T> m_Data;
      std::unordered_map<EntityID, size_t> m_EntityToIndex;
      std::unordered_map<size_t, EntityID> m_IndexToEntity;

      T& Add(EntityID entity, T component) {
          size_t index = m_Data.size();
          m_Data.push_back(std::move(component));
          m_EntityToIndex[entity] = index;
          m_IndexToEntity[index] = entity;
          return m_Data.back();
      }

      void Remove(EntityID entity) override {
          if (m_EntityToIndex.find(entity) == m_EntityToIndex.end()) return;

          size_t indexToRemove = m_EntityToIndex[entity];
          size_t lastIndex = m_Data.size() - 1;
          EntityID lastEntity = m_IndexToEntity[lastIndex];

          // Swap-and-pop
          std::swap(m_Data[indexToRemove], m_Data[lastIndex]);

          // Update lookup
          m_EntityToIndex[lastEntity] = indexToRemove;
          m_IndexToEntity[indexToRemove] = lastEntity;

          m_EntityToIndex.erase(entity);
          m_IndexToEntity.erase(lastIndex);

          m_Data.pop_back();
      }

      T* Get(EntityID entity) {
          auto it = m_EntityToIndex.find(entity);
          if (it == m_EntityToIndex.end()) return nullptr;
          return &m_Data[it->second];
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
      
      // Central registry for "Alive" entities if needed, or just ID counter
      EntityID m_NextEntityID = 0;
      EntityID CreateEntityID() { return m_NextEntityID++; }
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
          // Note: component argument is by value/move
          return &m_Universe->GetPool<T>().Add(m_Id, std::move(component));
      }

      template<typename T>
      T* GetComponent() {
          if (!IsValid()) return nullptr;
          return m_Universe->GetPool<T>().Get(m_Id);
      }
  };

}