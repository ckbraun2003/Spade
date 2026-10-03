// The installed spade::render_gl, beside a consumer's own GL loader.
//
// Built only when the prefix exports spade::render_gl (see CMakeLists.txt).
// It proves two things about the installed package, with no GL context:
//   1. The archive links and its API is callable: GlRenderer::create() with a
//      loader that finds nothing refuses with Code::unavailable before any GL
//      call (Rendering's GlRendererCreate.ANullLoaderIsRefusedBeforeAnyGlCall).
//   2. A consumer can bring its OWN GL loader. This program links glad itself
//      and calls it. render_gl loads GL through the caller's GlProcLoader into
//      a private table and holds no glad symbol, so the two cannot clash at
//      link time (the lead's condition, 2026-10-03).

#include <cstdio>

#include <glad/glad.h>

#include "render_gl/gl_renderer.hpp"

namespace {

// A loader that finds no GL entry point: no context exists here.
void* no_gl(const char*) {
    return nullptr;
}

}  // namespace

int check_render_gl() {
    // The consumer's own loader, on the same empty loader: it loads nothing.
    if (gladLoadGLLoader(&no_gl) != 0) {
        std::fprintf(stderr, "render_gl check: the consumer's own glad loaded GL from an empty loader\n");
        return 1;
    }

    const auto made = spade::render_gl::GlRenderer::create(&no_gl);
    if (made.has_value()) {
        std::fprintf(stderr, "render_gl check: GlRenderer::create() accepted a loader with no GL\n");
        return 1;
    }
    if (made.error().code != spade::Code::unavailable) {
        std::fprintf(stderr, "render_gl check: GlRenderer::create() refused with the wrong code: %s\n",
                     made.error().context.c_str());
        return 1;
    }
    std::printf("render_gl OK: an empty loader is refused (%s), and the consumer's own glad links beside it\n",
                made.error().context.c_str());
    return 0;
}
