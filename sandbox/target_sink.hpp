// TargetSink -- Plan C task C1, the seam SL11 names (SL11's own word was
// `Presenter`; see below for why that name is refused).
//
// ⛔ THE NAME `Presenter` IS REFUSED, RULED 2026-09-18. editor/ui/viewport/
// presenter.h:84 already declares `class Presenter`, which receives frame
// packets from the C5 host and puts them in a widget -- very nearly this
// seam's job. That would be a FOURTH homograph in an estate that has already
// paid for three: FramePool (three distinct objects), `component` (two live
// senses), and this very directory's spade_sandbox beside v1's Sandbox target.
//
// The criterion, which outlives the name: NAME THE SEAM FOR THE TYPE IT
// CONSUMES. This one takes a RenderTarget; the editor's takes a frame packet.
// The distinguishing word is the thing that actually differs, which is the
// name a future author cannot re-collide by accident.
//
// ⚠⚠ AND THE CRITERION ALSO TELLS YOU WHAT TO LEAVE OFF. SL11's draft gave
// this interface a second method, `should_close()`. It is NOT here, and that
// is a correctness argument rather than a tidiness one: a headless sink has no
// honest answer to it. Always-false means "run forever"; always-true means
// "one frame". Both are ARBITRARY VALUES INVENTED TO SATISFY AN INTERFACE, and
// an arbitrary answer on an interface is how a loop ends up asking the wrong
// object when to stop. It is the same shape as sim_tick being non-optional and
// zero-defaulted in both C5 frame mirrors: the fabricated value is LEGAL, so
// the fabrication is silent.
//
// Loop termination is the APPLICATION's concern. C0/C1 have no loop at all.
// C2's windowed sink exposes closing on its own concrete type.
//
// SL2b: this header includes only what the engine INSTALLS.

#pragma once

#include <cstdint>
#include <fstream>
#include <ios>
#include <string>
#include <vector>

#include "render/target.hpp"

namespace spade::sandbox {

// ---------------------------------------------------------------------------
// THE ONE STATEMENT OF THE CHANNEL ORDER, AND THE ONLY ONE IN THE SANDBOX.
//
// C2's windowed sink needs BGRX -> RGBA (GL_BGRA is ABSENT from ImGui's
// filtered loader, and absent from GL 1.1's header on the system path, so the
// swap is ours to do); the headless sink needs BGRX -> RGB for its PPM. Those
// are two destinations for ONE fact about the source layout, and writing them
// as two loops would be "two expressions that happen to agree" -- this realm's
// own named defect, the one that let competition-700's arithmetic hold to the
// last digit in two files FOR OPPOSITE REASONS.
//
// So both sinks call this, and the reorder exists once.
//
// ⭐ AND IT CHANGES WHAT L301'S FIX IS WORTH. That row records that the C1
// suite's oracle restates the conversion, so channel order is asserted by
// nothing. With one statement here, L301's known-answer case pins the WINDOWED
// path too when it lands, instead of leaving it beside the fix.
//
// Templated on the destination width so the alpha branch compiles out rather
// than running per pixel; static_assert keeps the instantiation set honest.
// ---------------------------------------------------------------------------
template <unsigned DstChannels>
void bgrx_to_channels(const spade::render::RenderTarget& target, std::vector<uint8_t>& out) {
    static_assert(DstChannels == 3u || DstChannels == 4u,
                  "the sandbox converts BGRX to RGB (PPM) or RGBA (GL texture) and nothing else");
    const size_t n = static_cast<size_t>(target.width) * target.height;
    out.assign(n * DstChannels, 0u);
    for (size_t i = 0; i < n; ++i) {
        out[i * DstChannels + 0] = target.pixels[i * 4 + 2];  // R <- the B slot
        out[i * DstChannels + 1] = target.pixels[i * 4 + 1];  // G
        out[i * DstChannels + 2] = target.pixels[i * 4 + 0];  // B <- the R slot
        if constexpr (DstChannels == 4u) {
            // X is undefined in BGRX by PA-1's own definition -- it is padding,
            // not alpha. Forcing 255 rather than copying it is deliberate: a
            // texture that inherited garbage in alpha would blend against the
            // clear colour and look like a renderer bug.
            out[i * 4 + 3] = 255u;
        }
    }
}

// The whole seam. One method and a destructor.
struct TargetSink {
    // PA-1: the RenderTarget NEVER owns its pixel memory -- a sink is HANDED a
    // buffer and must not allocate one. Get this wrong and the sandbox grows a
    // frame pool the engine deliberately does not have.
    virtual void accept(const spade::render::RenderTarget& target) = 0;
    virtual ~TargetSink() = default;
};

// ---------------------------------------------------------------------------
// HeadlessTargetSink -- the CI path, and the only sink C1 ships.
//
// BGRX8 -> binary PPM (P6), chosen because it needs no dependency at all: the
// sandbox must not pull an image library in to prove a seam links.
//
// ⚠ THIS IS REAL LOGIC, NOT A PASS-THROUGH, and that is what makes SL10's
// same-path test worth writing. The channel reorder below (BGRX -> RGB) is a
// transformation that CAN be wrong, so "what the sandbox emitted" and "what
// spade::render produced" are genuinely two things that must be shown to
// agree -- rather than one buffer compared with itself, which is the shape
// that cannot fail.
// ---------------------------------------------------------------------------
class HeadlessTargetSink final : public TargetSink {
  public:
    // `path` empty means render-and-discard: the frame is still accepted and
    // still converted, so the conversion is exercised even when nothing is
    // written. A sink that skipped its work when no file was wanted would make
    // --out change more than where the bytes go.
    explicit HeadlessTargetSink(std::string path) : path_(std::move(path)) {}

    void accept(const spade::render::RenderTarget& target) override {
        ++accepted_;
        // C2: the loop that used to live here is now bgrx_to_channels<3>, which
        // the windowed sink also calls with <4>. Behaviour-identical by
        // construction -- and NOT called verified until SandboxTargetSink's four
        // arms say so, because "by construction" is a claim about my reasoning
        // and ctest is a claim about the bytes.
        bgrx_to_channels<3u>(target, rgb_);
        width_ = target.width;
        height_ = target.height;
        if (path_.empty()) {
            return;
        }
        // ⚠ std::ofstream RATHER THAN std::fopen, and it is not a style choice.
        // This header compiles into TWO targets with DIFFERENT warning levels:
        // spade_sandbox, and spade_tests at /W4 /WX. MSVC raises C4996 on
        // fopen, which is a warning in one target and a hard error in the
        // other -- so the header was valid where it was written and invalid
        // where it is consumed.
        //
        // ⭐ THE HEADER-FANOUT CONTROL, ARRIVING AS A COMPILER ERROR: naming
        // the target you are changing is not the same as naming the whole
        // subject. The alternative fixes were both suppressions
        // (_CRT_SECURE_NO_WARNINGS, or relaxing the test target's flags), and
        // a suppression would have hidden the next instance too. Streams are
        // simply not deprecated on either toolchain.
        std::ofstream f(path_, std::ios::binary);
        if (!f) {
            failed_ = true;
            return;
        }
        const std::string header =
            "P6\n" + std::to_string(width_) + " " + std::to_string(height_) + "\n255\n";
        f.write(header.data(), static_cast<std::streamsize>(header.size()));
        f.write(reinterpret_cast<const char*>(rgb_.data()), static_cast<std::streamsize>(rgb_.size()));
        if (!f) {
            failed_ = true;
        }
    }

    // What this sink EMITTED, for SL10's same-path assertion to hash. Held
    // rather than re-read from disk so the assertion works with no --out path,
    // which is how the test runs it.
    [[nodiscard]] const std::vector<uint8_t>& emitted_rgb() const noexcept { return rgb_; }
    [[nodiscard]] uint32_t width() const noexcept { return width_; }
    [[nodiscard]] uint32_t height() const noexcept { return height_; }
    [[nodiscard]] uint32_t accepted() const noexcept { return accepted_; }
    // A write that could not complete. REPORTED, never absorbed: a sandbox that
    // silently produces no file is how "it ran and did nothing" becomes a bug
    // report.
    [[nodiscard]] bool failed() const noexcept { return failed_; }

  private:
    std::string path_;
    std::vector<uint8_t> rgb_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t accepted_ = 0;
    bool failed_ = false;
};

}  // namespace spade::sandbox
