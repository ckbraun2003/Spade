#include "Spade/Core/Objects.hpp"

namespace Spade::Internal {
    ComponentTypeID GetUniqueComponentID() {
        static std::atomic<ComponentTypeID> lastID{0};
        return lastID++;
    }
}
