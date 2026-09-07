// behavior.cpp -- see behavior.hpp for SL6's two rules that matter here:
// registration order is execution order, and a missing record_gpu is a refusal
// rather than a fallback.
//
// SubstepContext stays INCOMPLETE in this translation unit, deliberately.
// run_slot() forwards the reference and never reads a member, so nothing here
// needs physics/schedule.hpp -- which is what keeps objects/ and physics/ from
// including each other.

#include "objects/behavior.hpp"

#include <algorithm>

namespace spade::objects {

Result<uint32_t> BehaviorRegistry::register_behavior(const BehaviorDesc& desc) {
    if (desc.execute_cpu == nullptr) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "behavior '" + std::string(desc.name) +
                                         "' has no execute_cpu; SL6 requires a CPU implementation "
                                         "of every behavior"});
    }
    const bool duplicate = std::any_of(entries_.begin(), entries_.end(), [&](const Entry& e) {
        return e.name == desc.name;
    });
    if (duplicate) {
        return std::unexpected(Error{Code::invalid_argument,
                                     "behavior '" + std::string(desc.name) +
                                         "' is already registered; names are the key a saved "
                                         "BehaviorComponent resolves through and must be unique"});
    }

    const auto index = static_cast<uint32_t>(entries_.size());
    entries_.push_back(Entry{.name = std::string(desc.name),
                             .slot = desc.slot,
                             .reads = desc.reads,
                             .writes = desc.writes,
                             .execute_cpu = desc.execute_cpu,
                             .record_gpu = desc.record_gpu,
                             .user_data = desc.user_data});

    // Latches false and never recovers, which is the intended shape: once ANY
    // behavior is CPU-only the registry as a whole cannot be trusted on the
    // GPU-authoritative path, and unregistering is not offered.
    if (desc.record_gpu == nullptr) gpu_eligible_ = false;

    return index;
}

void BehaviorRegistry::run_slot(BehaviorSlot slot,
                                const physics::SubstepContext& ctx) const noexcept {
    // Registration order, filtered by slot. Not sorted, not grouped: the order
    // a caller registered in is the order it gets, so two behaviors touching
    // the same accumulator compose reproducibly.
    for (const Entry& entry : entries_) {
        if (entry.slot != slot) continue;
        entry.execute_cpu(ctx, entry.user_data);
    }
}

std::string_view BehaviorRegistry::name_at(uint32_t index) const noexcept {
    if (index >= entries_.size()) return {};
    return entries_[index].name;
}

}  // namespace spade::objects
