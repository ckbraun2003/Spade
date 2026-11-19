#include "scene/library_compile.hpp"

// RED stub: tests/test_library_compile.cpp fails until the implementation replaces this.
namespace spade::scene {

LibraryResult<CompiledDrone> compile_drone(const LibrarySet&, std::string_view, const CompileOptions&) {
    return std::unexpected(LibraryRefusal{"not_implemented", "compile_drone is not written yet", {}, {}, {}, {}, {}});
}

}  // namespace spade::scene
