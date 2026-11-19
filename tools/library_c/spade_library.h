/* ---------------------------------------------------------------------------
 * spade_library.h -- the C ABI of the library (spade_library.dll / libspade_library.so; L88l, S4).
 * Contract: docs/design/core/plans/2026-11-17-spade-library-json-contract.md, sections 0 to 2.
 *
 * C linkage, no struct crosses: a request is UTF-8 JSON text and so is every response. The host loads the library by the full path
 * it was given and looks nowhere else; there is no environment variable inside it.
 *
 * Return value of spade_lib_open and spade_lib_call:
 *   0  the call was answered with "ok": true
 *   1  the call was answered with "ok": false (a refusal object is in the response)
 *   2  ABI misuse (a null argument, a null handle): when response_json is not null it is set to an invalid_request refusal
 *      ("no library is open", "ok": false); with a null response_json nothing is produced
 * Every response is a NUL-terminated string the caller frees with spade_lib_free. A handle after spade_lib_close is dangling, and any use
 * of it is undefined: the ABI cannot detect it.
 *
 * Threads: on one handle the read ops may run concurrently from any threads; spade_lib_close must not race any other call on the handle.
 * --------------------------------------------------------------------------- */
#ifndef SPADE_LIBRARY_H
#define SPADE_LIBRARY_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && defined(SPADE_LIBRARY_BUILD_DLL)
#define SPADE_LIBRARY_API __declspec(dllexport)
#elif defined(_WIN32) && defined(SPADE_LIBRARY_USE_DLL)
#define SPADE_LIBRARY_API __declspec(dllimport)
#elif defined(SPADE_LIBRARY_BUILD_DLL) && defined(__GNUC__)
#define SPADE_LIBRARY_API __attribute__((visibility("default")))
#else
#define SPADE_LIBRARY_API
#endif

typedef struct spade_lib spade_lib;

/* A static string, "spade.library/1.0": the format's major and the minor this build speaks. */
SPADE_LIBRARY_API const char* spade_lib_version(void);

/* Opens the roots the request names; on 0, *out is the handle. On 1 or 2, *out is null. */
SPADE_LIBRARY_API int spade_lib_open(const char* request_json, spade_lib** out, char** response_json);

/* One call: `op` is "list", "find", "resolve", "hash_text", "validate", "compose_placement", "mesh_file_info", and so on. */
SPADE_LIBRARY_API int spade_lib_call(spade_lib* h, const char* op, const char* request_json, char** response_json);

SPADE_LIBRARY_API void spade_lib_close(spade_lib* h);

/* Frees any response_json, from open or call. A null pointer is fine. */
SPADE_LIBRARY_API void spade_lib_free(char* response_json);

#ifdef __cplusplus
}
#endif

#endif /* SPADE_LIBRARY_H */
