#include "fetch.hpp"

namespace spade::geofetch {

// RED: stubs.
int exit_code_of(FetchResultKind) { return 99; }
const char* name_of(FetchResultKind) { return ""; }
FetchResult fetch_map(const FetchRequest&, const FetchEnv&) { return {}; }

}  // namespace spade::geofetch
