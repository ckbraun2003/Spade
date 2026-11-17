#include "store.hpp"

#include "world/sha256.hpp"

namespace spade::partimport {

std::string cache_key_text(const KeyInputs& in) {
    return "spade-part-import-key-1;converter=" + in.converter + ";tolerance=" + json_number(in.tolerance_m) + ";angle=" +
           json_number(in.angle_rad) + ";unit=" + in.unit + ";axes=" + in.axes.text() + ";scale=" + json_number(in.scale) +
           ";object=" + in.object;
}

std::string cache_key(const KeyInputs& in) { return sha256_hex(cache_key_text(in)).substr(0, 16); }

}  // namespace spade::partimport
