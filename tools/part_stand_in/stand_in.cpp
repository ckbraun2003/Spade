#include "stand_in.hpp"

// RED stub: the tests in tests/test_part_stand_in.cpp fail until the implementation replaces this.
namespace spade::stand_in {

std::expected<std::vector<Box>, std::string> boxes_of_file(const std::filesystem::path& file, const Options&) {
    return std::unexpected("not implemented: " + file.string());
}

std::expected<StandInResult, std::string> add_stand_in(std::string_view, const std::filesystem::path&, const Options&) {
    return std::unexpected(std::string("not implemented"));
}

}  // namespace spade::stand_in
