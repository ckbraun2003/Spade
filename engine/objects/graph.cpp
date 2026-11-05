// graph.cpp -- spade_objects implementation.
//
// Task 1 created this as a placeholder so the STATIC target had a translation
// unit. Task 2 gave it the component-name table; Task 3 adds ObjectGraph's
// lifecycle. Stage 5 Task 7 moved the names into component.hpp's kComponentTypes, beside each
// type's availability.

#include "objects/graph.hpp"

#include "objects/component.hpp"
#include "objects/object.hpp"

#include <array>

namespace spade::objects {
std::span<const ComponentTypeInfo> component_types() noexcept {
    return std::span<const ComponentTypeInfo>(kComponentTypes);
}

// The names live in kComponentTypes (component.hpp), indexed BY ComponentTypeId and spelt beside each
// type's availability: one table, one site. A table that drifted out of step with the enum would hand
// back the wrong name for every type after the drift point, silently and only visibly in a saved file;
// the three cases of test_objects_component.cpp that tie a concrete TYPE to its id catch an enumerator
// swap, because a round-trip through this table proves only that the table is self-consistent.
std::string_view component_type_name(ComponentTypeId id) noexcept {
    const auto i = static_cast<uint32_t>(id);
    return i < kComponentTypeCount ? kComponentTypes[i].name : std::string_view{};
}

std::optional<ComponentTypeId> component_type_id_from_name(std::string_view name) noexcept {
    for (const ComponentTypeInfo& info : kComponentTypes) {
        if (info.name == name) return info.id;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// ObjectGraph (Task 3). Generation parity is BodyRef's: odd == live, even ==
// dead, 0 == never issued. See graph.hpp for why these counters -- unlike
// BodyRef's -- are deliberately NOT registered state.
// ---------------------------------------------------------------------------

ObjectId ObjectGraph::create(std::string_view name) {
    uint32_t index = 0;
    if (!free_list_.empty()) {
        index = free_list_.back();
        free_list_.pop_back();
    } else {
        index = static_cast<uint32_t>(slots_.size());
        slots_.emplace_back();
        generations_.push_back(0);
        masks_.emplace_back();  // sizing only; the reset below is the invariant
    }
    generations_[index] += 1;  // even -> odd: the slot is now live

    // Assigning a whole fresh Object, rather than only overwriting `name`, is
    // what keeps a recycled slot from handing the new occupant the dead one's
    // placement. Naming the one field that differs makes "everything else
    // resets" a single visible fact instead of an omission to notice.
    //
    // THIS IS THE ONLY PLACE A SLOT IS RESET, deliberately. destroy() could
    // scrub too, and the obvious instinct is to do both -- but then neither
    // could be removed without the other covering for it, and the test that
    // exists to catch a ghost-inheriting recycle (RecycledSlotStartsFromA-
    // DefaultObject) would pass under the deletion of either one. A guard
    // that cannot fail is not a guard (SL18). Resetting here also states the
    // guarantee where it is made and covers both paths uniformly: a fresh
    // emplace_back slot and a recycled one leave this line in the same state,
    // whereas scrubbing in destroy() would make create()'s promise depend on
    // an invariant maintained somewhere else. The cost is that a destroyed
    // slot keeps its dead name's allocation until reuse -- bounded by the
    // pool's peak, which slots_ never returns anyway.
    slots_[index] = Object{.name = std::string(name)};
    masks_[index] = 0;  // and with it every component the dead occupant had

    ++live_count_;
    return ObjectId{.index = index, .generation = generations_[index]};
}

bool ObjectGraph::destroy(ObjectId id) {
    if (!alive(id)) return false;
    generations_[id.index] += 1;  // odd -> even: the slot is now dead
    // The slot's contents are deliberately left alone; create() is the single
    // place that resets one. See the note there.
    free_list_.push_back(id.index);
    --live_count_;
    return true;
}

bool ObjectGraph::alive(ObjectId id) const noexcept {
    if (id.index >= generations_.size()) return false;
    // No separate null-handle case: a null ObjectId is generation 0, 0 is
    // even, and the parity test below rejects it for the same reason it
    // rejects every destroyed slot. One rule rather than two that could drift
    // apart. (The counter's uint32 wrap preserves parity in both directions,
    // as BodyRef's does; reaching it would take 2^32 create/destroy cycles on
    // a single slot.)
    return generations_[id.index] == id.generation && (id.generation % 2u) == 1u;
}

Object* ObjectGraph::get(ObjectId id) noexcept {
    return alive(id) ? &slots_[id.index] : nullptr;
}

const Object* ObjectGraph::get(ObjectId id) const noexcept {
    return alive(id) ? &slots_[id.index] : nullptr;
}

}  // namespace spade::objects
