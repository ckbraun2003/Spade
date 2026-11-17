// The HttpClient of a platform with no HTTP layer yet (L85d: on Linux the tool returns its named offline result and libcurl waits
// for a real Linux need; Windows has WinHTTP, http_winhttp.cpp).

#if !defined(_WIN32)

#include "http_client.hpp"

namespace spade::geofetch {

namespace {

class NoHttp final : public HttpClient {
  public:
    HttpResponse get(const std::string&, int, const HttpProgress&) override {
        HttpResponse r;
        r.error = HttpError::no_http_layer;
        r.detail = "no HTTP layer on this platform yet";
        return r;
    }
};

}  // namespace

std::unique_ptr<HttpClient> make_platform_http_client() { return std::make_unique<NoHttp>(); }

}  // namespace spade::geofetch

#endif  // !_WIN32
