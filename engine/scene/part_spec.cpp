#include "scene/part_spec.hpp"

// RED stub: tests/test_part_spec.cpp fails until the implementation replaces this.
namespace spade::scene {

Result<std::optional<PartFile>> part_spec_from_text(std::string_view) {
    return std::unexpected(Error{Code::invalid_argument, "spec: not implemented"});
}

}  // namespace spade::scene
