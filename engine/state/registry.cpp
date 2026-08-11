#include "state/registry.hpp"

#include <algorithm>
#include <iterator>

namespace spade {

Result<void> StateRegistry::validate(const RegisteredArray& desc) const {
    if (desc.name.empty()) {
        return std::unexpected(Error{Code::invalid_argument, "StateRegistry: empty array name"});
    }
    if (desc.elem_size == 0) {
        return std::unexpected(
            Error{Code::invalid_argument, "StateRegistry: zero elem_size for array '" + desc.name + "'"});
    }
    if (find(desc.name) != nullptr) {
        return std::unexpected(
            Error{Code::invalid_argument, "StateRegistry: duplicate array name '" + desc.name + "'"});
    }
    // A zero-element array (world_count or capacity_per_world of 0) is
    // allowed to carry a null pointer -- there is nothing to copy. A
    // non-empty one must not: a snapshot walk would dereference it.
    if (desc.data == nullptr && desc.element_count() != 0) {
        return std::unexpected(
            Error{Code::invalid_argument, "StateRegistry: null data for non-empty array '" + desc.name + "'"});
    }
    return {};
}

Result<void> StateRegistry::register_array(RegisteredArray desc) {
    if (Result<void> ok = validate(desc); !ok) return ok;
    arrays_.push_back(std::move(desc));
    return {};
}

Result<void> StateRegistry::register_arrays(std::vector<RegisteredArray> descs) {
    for (std::size_t i = 0; i < descs.size(); ++i) {
        if (Result<void> ok = validate(descs[i]); !ok) return ok;
        // validate() only sees what is already committed, so intra-batch
        // name collisions have to be caught here.
        for (std::size_t j = 0; j < i; ++j) {
            if (descs[j].name == descs[i].name) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "StateRegistry: duplicate array name '" + descs[i].name + "' within one batch"});
            }
        }
    }
    // Reserve before inserting so the commit itself cannot throw partway
    // through (moving a RegisteredArray is noexcept).
    arrays_.reserve(arrays_.size() + descs.size());
    arrays_.insert(arrays_.end(), std::make_move_iterator(descs.begin()), std::make_move_iterator(descs.end()));
    return {};
}

const RegisteredArray* StateRegistry::find(std::string_view name) const noexcept {
    auto it = std::find_if(arrays_.begin(), arrays_.end(),
                           [name](const RegisteredArray& array) { return array.name == name; });
    return it == arrays_.end() ? nullptr : &*it;
}

std::size_t StateRegistry::total_bytes() const noexcept {
    std::size_t total = 0;
    for (const RegisteredArray& array : arrays_) {
        total += array.byte_size();
    }
    return total;
}

}  // namespace spade
