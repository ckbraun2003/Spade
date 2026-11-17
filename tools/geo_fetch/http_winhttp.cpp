// WinHTTP behind geo_fetch's HttpClient (L85, SP-5; the design note, section 6): in every Windows 10 and 11, with the OS's TLS
// (Schannel), certificate store and proxy settings, so a tester's PC needs nothing installed. Certificates are verified; nothing
// here turns that off. Only https URLs are fetched. It is exercised by the one live check, never by a test.

#if defined(_WIN32)

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <winhttp.h>

#include <algorithm>
#include <string>

#include "http_client.hpp"
#include "spade_version.hpp"

namespace spade::geofetch {

namespace {

[[nodiscard]] std::wstring widen(const std::string& s) {
    if (s.empty()) return {};
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
    return w;
}

[[nodiscard]] std::string narrow(const std::wstring& w) {
    if (w.empty()) return {};
    const int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0, nullptr, nullptr);
    std::string s(static_cast<std::size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), s.data(), n, nullptr, nullptr);
    return s;
}

class Handle {
  public:
    explicit Handle(HINTERNET h = nullptr) : h_(h) {}
    ~Handle() {
        if (h_ != nullptr) WinHttpCloseHandle(h_);
    }
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    [[nodiscard]] HINTERNET get() const { return h_; }
    [[nodiscard]] explicit operator bool() const { return h_ != nullptr; }

  private:
    HINTERNET h_;
};

[[nodiscard]] std::string message_of(DWORD code) {
    wchar_t text[512] = {};
    const DWORD n = FormatMessageW(FORMAT_MESSAGE_FROM_HMODULE | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                                   GetModuleHandleW(L"winhttp.dll"), code, 0, text, 512, nullptr);
    std::string s = n != 0 ? narrow(std::wstring(text, n)) : "WinHTTP error " + std::to_string(code);
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ')) s.pop_back();
    return s + " (" + std::to_string(code) + ")";
}

[[nodiscard]] HttpResponse failed(DWORD code) {
    HttpResponse r;
    r.detail = message_of(code);
    switch (code) {
        case ERROR_WINHTTP_TIMEOUT: r.error = HttpError::timeout; break;
        case ERROR_WINHTTP_SECURE_FAILURE:
        case ERROR_WINHTTP_SECURE_CERT_CN_INVALID:
        case ERROR_WINHTTP_SECURE_CERT_DATE_INVALID:
        case ERROR_WINHTTP_SECURE_CERT_REV_FAILED:
        case ERROR_WINHTTP_SECURE_CERT_REVOKED:
        case ERROR_WINHTTP_SECURE_INVALID_CA:
        case ERROR_WINHTTP_SECURE_INVALID_CERT:
        case ERROR_WINHTTP_CLIENT_AUTH_CERT_NEEDED: r.error = HttpError::tls; break;
        default: r.error = HttpError::offline; break;  // name not resolved, cannot connect, connection error, ...
    }
    return r;
}

class WinHttpClient final : public HttpClient {
  public:
    HttpResponse get(const std::string& url, int timeout_s, const HttpProgress& progress) override {
        const std::wstring wurl = widen(url);
        URL_COMPONENTS parts{};
        parts.dwStructSize = sizeof parts;
        parts.dwSchemeLength = parts.dwHostNameLength = parts.dwUrlPathLength = parts.dwExtraInfoLength = static_cast<DWORD>(-1);
        if (!WinHttpCrackUrl(wurl.c_str(), 0, 0, &parts)) return failed(GetLastError());
        if (parts.nScheme != INTERNET_SCHEME_HTTPS) {
            HttpResponse r;
            r.error = HttpError::tls;
            r.detail = "only https URLs are fetched";
            return r;
        }
        const std::wstring host(parts.lpszHostName, parts.dwHostNameLength);
        const std::wstring object(parts.lpszUrlPath, parts.dwUrlPathLength + parts.dwExtraInfoLength);

        const std::wstring agent = widen(std::string("spade-geo-fetch/") + spade::version::kString);
        Handle session(WinHttpOpen(agent.c_str(), WINHTTP_ACCESS_TYPE_AUTOMATIC_PROXY, WINHTTP_NO_PROXY_NAME,
                                   WINHTTP_NO_PROXY_BYPASS, 0));
        if (!session) {  // before Windows 8.1 there is no automatic proxy mode
            session = Handle(WinHttpOpen(agent.c_str(), WINHTTP_ACCESS_TYPE_DEFAULT_PROXY, WINHTTP_NO_PROXY_NAME,
                                         WINHTTP_NO_PROXY_BYPASS, 0));
        }
        if (!session) return failed(GetLastError());
        DWORD protocols = WINHTTP_FLAG_SECURE_PROTOCOL_TLS1_2;
        WinHttpSetOption(session.get(), WINHTTP_OPTION_SECURE_PROTOCOLS, &protocols, sizeof protocols);
        const int wait_ms = std::max(1, timeout_s) * 1000;
        WinHttpSetTimeouts(session.get(), 10000, 10000, wait_ms, wait_ms);

        Handle connection(WinHttpConnect(session.get(), host.c_str(), parts.nPort, 0));
        if (!connection) return failed(GetLastError());
        Handle request(WinHttpOpenRequest(connection.get(), L"GET", object.c_str(), nullptr, WINHTTP_NO_REFERER,
                                          WINHTTP_DEFAULT_ACCEPT_TYPES, WINHTTP_FLAG_SECURE));
        if (!request) return failed(GetLastError());
        if (!WinHttpSendRequest(request.get(), WINHTTP_NO_ADDITIONAL_HEADERS, 0, WINHTTP_NO_REQUEST_DATA, 0, 0, 0) ||
            !WinHttpReceiveResponse(request.get(), nullptr)) {
            return failed(GetLastError());
        }

        HttpResponse out;
        DWORD status = 0;
        DWORD size = sizeof status;
        if (WinHttpQueryHeaders(request.get(), WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER, WINHTTP_HEADER_NAME_BY_INDEX,
                                &status, &size, WINHTTP_NO_HEADER_INDEX)) {
            out.status = static_cast<int>(status);
        }
        DWORD length = 0;
        size = sizeof length;
        if (!WinHttpQueryHeaders(request.get(), WINHTTP_QUERY_CONTENT_LENGTH | WINHTTP_QUERY_FLAG_NUMBER,
                                 WINHTTP_HEADER_NAME_BY_INDEX, &length, &size, WINHTTP_NO_HEADER_INDEX)) {
            length = 0;  // the server did not say
        }
        std::string chunk(64 * 1024, '\0');
        for (;;) {
            DWORD got = 0;
            if (!WinHttpReadData(request.get(), chunk.data(), static_cast<DWORD>(chunk.size()), &got)) {
                HttpResponse r = failed(GetLastError());
                if (r.error == HttpError::offline) r.error = HttpError::timeout;  // the answer broke off after it began
                return r;
            }
            if (got == 0) break;
            if (out.body.size() + got > kMaxResponseBytes) {
                HttpResponse r;
                r.error = HttpError::too_large;
                r.detail = "the answer is larger than the cap";
                return r;
            }
            out.body.append(chunk.data(), got);
            if (progress) progress(out.body.size(), length);
        }
        return out;
    }
};

}  // namespace

std::unique_ptr<HttpClient> make_platform_http_client() { return std::make_unique<WinHttpClient>(); }

}  // namespace spade::geofetch

#endif  // _WIN32
