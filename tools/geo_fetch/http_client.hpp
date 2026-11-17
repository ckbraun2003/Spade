#pragma once

// ---------------------------------------------------------------------------
// geo_fetch/http_client.hpp -- the one seam between the fetch and the network (L85, SP-5; the design note, section 6).
//
// Everything above this interface is the same on every platform and is what the tests drive through a fake. Two things sit
// below it: WinHTTP on Windows (http_winhttp.cpp: in the OS, with its TLS, certificate store and proxy settings), and, on
// every other platform for now, a client that says "no HTTP layer on this platform yet" (http_none.cpp; L85d: libcurl waits
// for a real Linux need). Certificates are always verified; there is no insecure option. The User-Agent names the tool and
// its version and nothing else.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace spade::geofetch {

enum class HttpError {
    none,
    offline,         // no network: DNS, connect or TLS setup failed before any server answered
    timeout,         // connected, then no answer in time
    tls,             // the server's certificate or protocol was refused
    too_large,       // the answer is bigger than the cap
    no_http_layer,   // this platform has no HTTP layer yet
};

struct HttpResponse {
    HttpError error = HttpError::none;
    int status = 200;      // the HTTP status when error is none
    std::string body;      // the whole answer
    std::string detail;    // for an error: what the system said, as plain text
};

// Called as the body arrives: bytes so far, and the total when the server said it (0 when it did not).
using HttpProgress = std::function<void(std::uint64_t received, std::uint64_t total)>;

class HttpClient {
  public:
    virtual ~HttpClient() = default;
    // GETs `url` (always https), following redirects. `timeout_s` bounds the wait for the answer after connecting.
    virtual HttpResponse get(const std::string& url, int timeout_s, const HttpProgress& progress) = 0;
};

inline constexpr std::uint64_t kMaxResponseBytes = 64ull * 1024ull * 1024ull;  // an answer larger than this is too_large

// This platform's client: WinHTTP on Windows; elsewhere one whose get() returns no_http_layer.
[[nodiscard]] std::unique_ptr<HttpClient> make_platform_http_client();

}  // namespace spade::geofetch
