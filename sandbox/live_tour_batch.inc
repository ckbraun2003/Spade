// THE BATCH CHAPTER (L64; the lead, 2026-11-11): chapter 4 of the live smoke, "new in this batch". NOT A HEADER and not
// a translation unit of its own: live_tour.cpp includes it, inside its anonymous namespace, after the editor chapter,
// because the tour's classes (Tour, the ledger) are that file's. Its steps are THIS batch's visible changes, named in
// the batch's READY by the merging realm and written here; the next batch replaces them (a step worth keeping is
// promoted into a standing chapter by a light edit). A batch with nothing visible says so on the card: "Engine only:
// <list>. Nothing new to see."
//
// The ledger is kSteps' "batch" entries; the poster is 04_batch.png. BUDGET: at most 3 steps and 25 s of video, 2 s of
// it the card (60 rendered frames make a second of video: the tour renders at 60 and records every second frame), and
// the whole tour at most 170 s; the checks "length-budget" and "tour-length" fail the run past either.
//
// THIS RUN IS THE CATCH-UP "since batch 23" (batches 24-29): soft shadows (RND-10), the Subgrid turbulence row on the
// GPU (L52/L54, L64 (b)), the Run-mode wind row (L46, L64 (c)). The card says "New in this batch" from batch 30 on.

constexpr uint64_t kBatchChapterMaxFrames = 23u * 60u;  // 25 s less the card's 2
constexpr uint64_t kTourMaxVideoFrames = 170u * kVideoFps;

// The card: the label is "<sha> <date>" (live-smoke.ps1's), shown "<sha> . <date>". What is new, in a line each, and
// what is engine only (the catch-up lists L50, the library's install: no view of it exists).
[[nodiscard]] std::string batch_card(std::string_view label) {
    std::string tagged(label);
    if (const std::size_t space = tagged.find(' '); space != std::string::npos) {
        tagged.replace(space, 1, " \xC2\xB7 ");
    }
    return "4  New since batch 23 \xC2\xB7 " + tagged +
           "\nsoft shadows \xC2\xB7 subgrid turbulence (GPU) \xC2\xB7 a live wind in Run"
           "\nthe airflow keeps its start wind"
           "\nEngine only: the installed drone library (L50). Nothing new to see.";
}

// A frame of `scene` from `cam` on the CPU raster, BGRX8, `w` x `h`: what the soft-shadow step compares.
[[nodiscard]] bool batch_shot(const render::RenderScene& scene, const render::Camera& cam,
                              const render::RenderOptions& options, uint32_t w, uint32_t h,
                              std::vector<uint8_t>& out) {
    out.assign(static_cast<std::size_t>(w) * h * 4u, uint8_t{0});
    render::RenderTarget target{.pixels = std::span<uint8_t>(out), .width = w, .height = h, .stride = w * 4u};
    return render::render(scene, cam, options, target).has_value();
}

// Whether pixels `a` and `b` (BGRX8) differ in any colour channel by more than `threshold` of 255.
[[nodiscard]] bool batch_differs(const uint8_t* a, const uint8_t* b, int threshold) {
    for (int c = 0; c < 3; ++c) {
        if (std::abs(static_cast<int>(a[c]) - static_cast<int>(b[c])) > threshold) return true;
    }
    return false;
}

// The world point where the ray through pixel (px, py) of a `w` x `h` frame from `cam` meets the plane y = `ground`.
[[nodiscard]] std::optional<glm::vec3> batch_ground_point(const render::Camera& cam, float w, float h, float px,
                                                          float py, float ground) {
    const float tan_half = std::tan(0.5f * cam.fov_y_radians);
    const glm::vec3 local((2.0f * (px + 0.5f) / w - 1.0f) * tan_half * (w / h),
                          (1.0f - 2.0f * (py + 0.5f) / h) * tan_half, -1.0f);
    const glm::vec3 dir = glm::normalize(cam.orientation * local);
    if (!(dir.y < -1.0e-4f)) return std::nullopt;
    const float s = (ground - cam.position.y) / dir.y;
    if (!(s > 0.0f)) return std::nullopt;
    return cam.position + s * dir;
}

void batch_tour(Tour& t, const fs::path& assets) {
    Smoke& s = t.smoke();
    std::unique_ptr<EditorSession> session;
    const DrawFn draw = [&session, &t](const FrameInput& in) { return session->frame(in, t.sink().delta_seconds()); };
    const auto idle = InputFn{};
    const auto open = [&]() -> bool { return s.check(session != nullptr, "open", "the chapter's editor is open"); };
    const auto w = [&]() -> editor::EditorWorkspace& { return session->workspace(); };
    const auto scene_now = [&]() -> const scene::SceneDesc& { return session->workspace().documents().scene.desc(); };
    const auto then = [&](int frames) { t.pump(frames, idle, draw); };
    const auto command = [&](editor::Command c) {
        w().panels().commands.push_back(c);
        then(5);
    };
    const auto click_towards = [&](glm::vec3 at) {
        const glm::vec3 eye = session->camera().to_render_camera().position;
        w().click(eye, at - eye);
    };
    const auto arm = [&](std::string_view item) {
        for (std::size_t i = 0; i < w().panels().library.size(); ++i) {
            if (w().panels().library[i].name == item) w().panels().armed = i;
        }
    };
    const auto model_of = [&](std::string_view vehicle) -> const vehicles::ModelType* {
        const scene::SceneVehicle* v = editor::EditorWorkspace::find_vehicle(scene_now(), vehicle);
        if (v == nullptr) return nullptr;
        for (const vehicles::ModelType& m : scene_now().models) {
            if (m.name == v->model) return &m;
        }
        return nullptr;
    };
    const fs::path work = t.out_dir() / "batch_work";
    const uint64_t frames_at_start = t.frames();

    // ---- STEP 1: soft shadows (RND-10, batch 27). The drone's shadow on the ground, close, in the window (the video);
    // beside it, on the CPU raster and the same scene and camera, the shadow frame at today's defaults against the
    // frame at the old ones (filter 0, no map of their own for the dynamics) and against none.
    t.step("batch", "soft_shadows", [&] {
        std::error_code ec;
        fs::remove_all(work, ec);
        for (const char* dir : {"scenes", "worlds"}) {
            fs::create_directories(work / dir, ec);
            for (fs::directory_iterator it(assets / dir, ec), end; !ec && it != end; it.increment(ec)) {
                if (it->is_regular_file()) {
                    fs::copy_file(it->path(), work / dir / it->path().filename(), fs::copy_options::overwrite_existing,
                                  ec);
                }
            }
        }
        if (!ec) {
            fs::copy(assets / "library", work / "library",
                     fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
        }
        if (!s.check(!ec, "work-copy", ec ? ec.message() : "assets/scenes, assets/worlds and assets/library copied")) {
            return;
        }
        const fs::path file = work / "scenes" / "hover.scene.yaml";
        Result<editor::OpenedScene> opened = editor::open_scene(file);
        if (!s.check(opened.has_value(), "opens", opened ? file.filename().string() : opened.error().context)) return;
        Result<std::unique_ptr<EditorSession>> made = EditorSession::create(std::move(*opened));
        if (!s.check(made.has_value(), "composes", made ? "hover" : made.error().context)) return;
        session = std::move(*made);
        session->attach(t.sink());
        then(30);

        const render::RenderOptions& defaults = session->render_options();
        s.check(defaults.shadows && defaults.shadow_filter_texels == 1.0f && defaults.dynamic_shadow_size == 1024u,
                "defaults-on",
                std::format("shadows {}, filter {} texel, the dynamics' own map {}", defaults.shadows ? "on" : "off",
                            defaults.shadow_filter_texels, defaults.dynamic_shadow_size));

        // Find the drone's shadow: the wide frame's pixels the shadows darken, their centroid put on the ground.
        constexpr uint32_t kW = 1280;
        constexpr uint32_t kH = 720;
        const glm::vec3 drone_at = scene_now().vehicles.at(0).start.pos;
        OrbitCamera& orbit = session->camera();
        orbit.target = glm::vec3(drone_at.x, 0.0f, drone_at.z);
        orbit.yaw = 0.6f;
        orbit.pitch = 0.9f;
        orbit.distance = 2.5f;
        then(10);
        render::RenderOptions hard = defaults;  // today's frame before RND-10: one texel, unfiltered; dynamics folded
        hard.shadow_filter_texels = 0.0f;
        hard.dynamic_shadow_size = 0u;
        render::RenderOptions none = defaults;
        none.shadows = false;
        const render::RenderScene scene = session->render_scene();
        std::vector<uint8_t> wide_hard;
        std::vector<uint8_t> wide_none;
        const render::Camera wide = orbit.to_render_camera();
        if (!s.check(batch_shot(scene, wide, hard, kW, kH, wide_hard) && batch_shot(scene, wide, none, kW, kH, wide_none),
                     "renders", "the CPU raster drew the frames")) {
            return;
        }
        // The shadow's pixels are those a shadow darkens by more than 8 of 255 in any channel. At 2.5 m a pixel is
        // 2 x 2.5 x tan 30 / 720 = 4 mm and the quad's shadow is about 0.3 m: thousands of pixels; 100 is a floor.
        constexpr std::size_t kShadowSeenPixels = 100;
        double sum_x = 0.0;
        double sum_y = 0.0;
        std::size_t dark = 0;
        for (uint32_t y = 0; y < kH; ++y) {
            for (uint32_t x = 0; x < kW; ++x) {
                const std::size_t i = (static_cast<std::size_t>(y) * kW + x) * 4u;
                if (batch_differs(&wide_hard[i], &wide_none[i], 8)) {
                    sum_x += x;
                    sum_y += y;
                    ++dark;
                }
            }
        }
        if (!s.check(dark >= kShadowSeenPixels, "shadow-seen",
                     std::format("{} pixels of the wide frame are in the drone's shadow (at least {})", dark,
                                 kShadowSeenPixels))) {
            return;
        }
        const std::optional<glm::vec3> ground =
            batch_ground_point(wide, static_cast<float>(kW), static_cast<float>(kH),
                               static_cast<float>(sum_x / static_cast<double>(dark)),
                               static_cast<float>(sum_y / static_cast<double>(dark)), 0.0f);
        const float off = ground ? glm::length(glm::vec2(ground->x - drone_at.x, ground->z - drone_at.z)) : -1.0f;
        if (!s.check(ground.has_value() && off < 2.0f, "shadow-on-the-ground",
                     ground ? std::format("the shadow's centre is {:.2f} m from under the drone", off)
                            : std::string("the shadow's centre is above the horizon"))) {
            return;
        }

        // Close on it: 0.6 m, so a pixel is 2 x 0.6 x tan 30 / 720 = 1 mm.
        orbit.target = *ground;
        orbit.pitch = 0.8f;
        orbit.distance = 0.6f;
        then(10);
        const render::Camera close = orbit.to_render_camera();
        const render::RenderScene close_scene = session->render_scene();
        std::vector<uint8_t> soft_a;
        std::vector<uint8_t> soft_b;
        std::vector<uint8_t> old_frame;
        std::vector<uint8_t> no_shadows;
        if (!s.check(batch_shot(close_scene, close, defaults, kW, kH, soft_a) &&
                         batch_shot(close_scene, close, defaults, kW, kH, soft_b) &&
                         batch_shot(close_scene, close, hard, kW, kH, old_frame) &&
                         batch_shot(close_scene, close, none, kW, kH, no_shadows),
                     "close-renders", "the CPU raster drew the close frames")) {
            return;
        }
        // THE NOISE FLOOR: two renders at the same options are byte-identical (ShadowDefaults, RND-10), so any
        // difference between the soft and the old frame is the options' doing.
        s.check(soft_a == soft_b, "deterministic", "two renders at the defaults are byte for byte the same");

        // THE PENUMBRA THRESHOLD, fixed before it was measured. The outline E of the shadow is the count of its
        // pixels (those shadows darken by more than 8 of 255) that have a 4-neighbour outside it. The defaults'
        // filter is one texel wide, so a receiver pixel on the outline changes with the old frame's when the
        // filter's ramp covers part of it: with probability about min(1, texel / pixel). The dynamics' map is the
        // drone's light-space footprint over 1024 texels, 0.3 m / 1024 = 0.3 mm for this quad at the least, against
        // a 1 mm pixel at 0.6 m: a ramp fraction of about 0.3. The threshold is a sixth of that, 1/20 of E: the
        // outline's other pixels (a tap a texel away reads a neighbouring face, the shadow's inner and the
        // drone's own) may add, never subtract; and at least 8 pixels, so a tiny outline still has to show some.
        std::size_t outline = 0;
        std::size_t changed = 0;
        const auto in_shadow = [&](uint32_t x, uint32_t y) {
            const std::size_t i = (static_cast<std::size_t>(y) * kW + x) * 4u;
            return batch_differs(&old_frame[i], &no_shadows[i], 8);
        };
        for (uint32_t y = 0; y < kH; ++y) {
            for (uint32_t x = 0; x < kW; ++x) {
                const std::size_t i = (static_cast<std::size_t>(y) * kW + x) * 4u;
                if (batch_differs(&soft_a[i], &old_frame[i], 0)) ++changed;
                if (!in_shadow(x, y)) continue;
                const bool edge = x == 0 || y == 0 || x + 1 == kW || y + 1 == kH || !in_shadow(x - 1, y) ||
                                  !in_shadow(x + 1, y) || !in_shadow(x, y - 1) || !in_shadow(x, y + 1);
                if (edge) ++outline;
            }
        }
        const std::size_t threshold = std::max<std::size_t>(8, outline / 20);
        s.check(outline >= 200 && changed >= threshold, "penumbra",
                std::format("the shadow's outline is {} pixels; {} pixels differ from the old frame, at least {}",
                            outline, changed, threshold));
        then(60);
    });

    // ---- STEP 2: Run air with the Subgrid turbulence row on (L52/L54; L64 (b), batch 29). On the GPU: nu_t is met
    // and the line names Cs; with no device the row is off and the card says so.
    t.step("batch", "turbulence_air", [&] {
        if (!open()) return;
        constexpr uint64_t kAirFrames = 6;
        constexpr int kMaxIterations = 150;  // 5 s of video at most
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        arm("Phantom 3-class quad");
        click_towards(glm::vec3(at.x, 0.0f, at.z + 1.5f));
        const std::string quad = w().panels().selection.name;
        if (!s.check(!quad.empty() && model_of(quad) != nullptr, "placed", w().panels().status)) return;
        w().panels().pending.push_back(editor::AddAirflowRegion{quad});
        w().apply_panels();
        if (!s.check(!scene_now().airflow.empty(), "region", w().panels().status)) return;
        w().panels().air_turbulence = true;  // the Advanced row
        command(editor::Command::run_air);
        if (!s.check(w().air() != nullptr, "runs", w().panels().status)) return;
        w().panels().pilot.controls = analysis::Controls{true, 0.6f, 0.0f, 0.0f, 0.0f};
        const OrbitCamera before = session->camera();
        for (const scene::SceneVehicle& v : scene_now().vehicles) {
            if (v.name == quad) session->camera().target = v.start.pos - glm::vec3(0.0f, 0.1f, 0.0f);
        }
        session->camera().yaw = 0.52f;
        session->camera().pitch = 0.35f;
        const scene::SceneAirflowRegion& region = scene_now().airflow.back();
        session->camera().distance = 2.0f * static_cast<float>(region.cells.nx) * region.cells.dx;
        std::shared_ptr<const editor::AirFrame> frame;
        for (int i = 0; i < kMaxIterations; ++i) {
            then(1);
            frame = w().air() != nullptr ? w().air()->latest() : nullptr;
            if (frame && (!frame->error.empty() || frame->frames >= kAirFrames)) break;
        }
        s.check(frame && frame->error.empty() && frame->frames >= kAirFrames, "publishes",
                !frame                  ? "no frame handed over"
                : !frame->error.empty() ? frame->error
                                        : std::format("{} frames, {:.3f} s of air", frame->frames, frame->t_s));
        if (frame && w().air() != nullptr) {
            const bool on_gpu = w().air()->backend() == "GPU";
            const std::string expected_off = editor::turbulence_line(editor::kAirSgsCs, false, analysis::LiveSgs{});
            const bool ok = on_gpu ? frame->sgs.nu_t_max > 0.0f && frame->turbulence_line.rfind("Cs 0.17", 0) == 0
                                   : frame->turbulence_line == expected_off &&
                                         w().air()->backend().find(editor::kAirTurbulenceNeedsTheGpu) !=
                                             std::string::npos;
            s.check(ok, "turbulence", (on_gpu ? "on the GPU: " : "no GPU, so off: ") + frame->turbulence_line);
            s.check(!w().panels().turbulence_line.empty(), "row-line",
                    "the panel shows: " + w().panels().turbulence_line);
            t.sink().set_card(frame->turbulence_line);
        }
        t.poster("batch");
        then(5);
        then(25);
        t.sink().set_card({});
        command(editor::Command::stop);
        session->camera() = before;
        w().panels().air_turbulence = false;
        command(editor::Command::undo);  // the region
        command(editor::Command::undo);  // the drone
    });

    // ---- STEP 3: the Run-mode wind (L46; L64 (c), batch 29). A Phantom climbing in Run; the panel's wind row set
    // to 6 m/s along +x; the wind is the run's (recorded as an input), not the document's, and it moves the drone.
    t.step("batch", "wind_change", [&] {
        if (!open()) return;
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        const std::size_t before = scene_now().vehicles.size();
        arm("Phantom 3-class quad");
        click_towards(glm::vec3(at.x, 0.0f, at.z + 1.5f));
        if (!s.check(scene_now().vehicles.size() == before + 1, "placed", w().panels().status)) return;
        const std::string quad = scene_now().vehicles.back().name;
        w().panels().selection = editor::Selection{editor::Selection::Kind::object, quad};
        command(editor::Command::run);
        const editor::PilotPanel& pilot = w().panels().pilot;
        if (!s.check(w().run() != nullptr && pilot.vehicle == quad, "piloted",
                     pilot.vehicle.empty() ? pilot.why_not : "flying " + pilot.vehicle)) {
            command(editor::Command::stop);
            command(editor::Command::undo);
            return;
        }
        const auto body_vel = [&]() -> glm::vec3 {
            const editor::EditorRun* run = w().run();
            const std::optional<VehicleRef> ref = run != nullptr ? run->vehicle_ref(quad) : std::nullopt;
            if (!ref) return glm::vec3(0.0f);
            const Result<const BodyState*> body = run->sim().body(ref->body);
            return body ? (*body)->vel : glm::vec3(0.0f);
        };
        const auto fly = [&](int n, const FrameInput& keys) {
            return t.pump(n, [keys](int) { return keys; }, draw);
        };
        FrameInput none;
        FrameInput space;
        space.arm_pressed = true;
        FrameInput page_up;
        page_up.throttle = 1.0f;
        const glm::vec3 wind(6.0f, 0.0f, 0.0f);
        const glm::vec3 document = w().documents().world.desc().environment.wind;
        if (!fly(1, space) || !fly(30, none) || !fly(45, page_up)) return;
        // Two windows of a second: the first without the wind is the drift's own (the climb's), the second with it.
        const glm::vec3 v0 = body_vel();
        if (!fly(60, none)) return;
        const glm::vec3 v1 = body_vel();
        w().panels().wind_request = wind;
        w().apply_panels();
        if (!fly(60, none)) return;
        const glm::vec3 v2 = body_vel();
        const editor::EditorRun* run = w().run();
        s.check(run != nullptr && run->wind() == wind && run->wind_sets().size() == 1u &&
                    run->wind_sets()[0].wind == wind && run->wind_sets()[0].tick <= run->tick(),
                "wind-set",
                run != nullptr ? std::format("wind {} {} {}, {} set recorded at tick {} of {}", run->wind().x, run->wind().y,
                                             run->wind().z, run->wind_sets().size(),
                                             run->wind_sets().empty() ? 0u : run->wind_sets()[0].tick, run->tick())
                               : std::string("no run"));
        s.check(w().panels().live_wind == wind, "row-shows-it",
                std::format("the row holds {} {} {}", w().panels().live_wind.x, w().panels().live_wind.y,
                            w().panels().live_wind.z));
        s.check(w().documents().world.desc().environment.wind == document && !w().documents().world.dirty(),
                "not-an-edit", "the world's own wind is as it was and nothing is unsaved");
        // THE DRIFT THRESHOLD, fixed before it was measured: the second's change of velocity along the wind must be
        // at least 10 times the previous second's (the drift the climb has on its own) and at least 0.05 m/s. At
        // 6 m/s a body of the Phantom's size takes a drag of about 0.5 x 1.2 x 36 x 0.05 x 1 = 1 N on a mass of
        // 1.2 kg: 0.9 m/s^2 for a second, 18 times the floor, which stands for a drag area 18 times smaller.
        const float own = std::abs(v1.x - v0.x);
        const float moved = v2.x - v1.x;
        s.check(moved >= std::max(10.0f * own, 0.05f), "wind-moves-the-drone",
                std::format("{:.3f} m/s along the wind in the second after the set, against {:.3f} m/s of its own "
                            "drift the second before (at least {:.3f})",
                            moved, own, std::max(10.0f * own, 0.05f)));
        t.sink().set_card("a live wind in Run: set on the row, recorded, not saved.\nThe airflow keeps its start wind.");
        then(30);
        t.sink().set_card({});
        command(editor::Command::stop);
        command(editor::Command::undo);
        s.check(t.frames() - frames_at_start <= kBatchChapterMaxFrames, "length-budget",
                std::format("{:.1f} s of video in the chapter's steps (at most {} s, the card besides)",
                            static_cast<double>(t.frames() - frames_at_start) / 60.0, kBatchChapterMaxFrames / 60));
    });

    session.reset();  // detaches its panels while the window is still open
    std::error_code ec;
    fs::remove_all(work, ec);
    if (ec) s.note("the chapter's work copy was not removed: " + ec.message());
}
