// THE BATCH CHAPTER (L64; the lead, 2026-11-11): chapter 5 of the live smoke, "new in this batch". NOT A HEADER and not
// a translation unit of its own: live_tour.cpp includes it, inside its anonymous namespace, after the bodies chapter,
// because the tour's classes (Tour, the ledger) are that file's. Its steps are THIS batch's visible changes, named in
// the batch's READY by the merging realm and written here; the next batch replaces them (a step worth keeping is
// promoted into a standing chapter by a light edit). A batch with nothing visible says so on the card: "Engine only:
// <list>. Nothing new to see."
//
// The ledger is kSteps' "batch" entries; the poster is 05_batch.png. BUDGET: at most 3 steps and 25 s of video, 2 s of
// it the card (60 rendered frames make a second of video: the tour renders at 60 and records every second frame), and
// the whole tour at most 170 s; the checks "length-budget" and "tour-length" fail the run past either.
//
// THIS RUN (batch 32; the lead, 2026-11-16): L82, contact shapes in the editor. ONE step: the hover quad is tilted and
// lifted, a Box is picked as the Collision section's combo does (the shape sized from the quad's parts), the cyan
// wireframe is shown, and Run drops it: it lands on a face. The earlier batch's steps (About, User Guide, the "Save
// changes?" prompt) are retired; their unit tests judge them.

constexpr uint64_t kBatchChapterMaxFrames = 23u * 60u;  // 25 s less the card's 2
constexpr uint64_t kTourMaxVideoFrames = 170u * kVideoFps;

// The card: the label is "<sha> <date>" (live-smoke.ps1's), shown "<sha> . <date>". What is new, in a line each, and
// what is engine only.
[[nodiscard]] std::string batch_card(std::string_view label) {
    std::string tagged(label);
    if (const std::size_t space = tagged.find(' '); space != std::string::npos) {
        tagged.replace(space, 1, " \xC2\xB7 ");
    }
    return "5  New in this batch \xC2\xB7 " + tagged +
           "\nSpade Editor " + std::string(spade::version::kString) +
           "\nA contact shape on a vehicle: pick it, see it, drop it";
}

void batch_tour(Tour& t, const fs::path& assets) {
    Smoke& s = t.smoke();
    std::unique_ptr<EditorSession> session;
    const DrawFn draw = [&session, &t](const FrameInput& in) { return session->frame(in, t.sink().delta_seconds()); };
    const auto idle = InputFn{};
    const auto w = [&]() -> editor::EditorWorkspace& { return session->workspace(); };
    const auto scene_now = [&]() -> const scene::SceneDesc& { return session->workspace().documents().scene.desc(); };
    const auto then = [&](int frames) { t.pump(frames, idle, draw); };
    const auto command = [&](editor::Command c) {
        w().panels().commands.push_back(c);
        then(5);
    };
    const uint64_t frames_at_start = t.frames();

    // ---- STEP 1: a Box on the hover quad, seen and dropped. Nothing is saved: the scene is opened from the assets as
    // they are and never written.
    t.step("batch", "shapes", [&] {
        Result<editor::OpenedScene> opened = editor::open_scene(assets / "scenes" / "hover.scene.yaml");
        if (!s.check(opened.has_value(), "opens", opened ? "hover.scene.yaml" : opened.error().context)) return;
        Result<std::unique_ptr<EditorSession>> made = EditorSession::create(std::move(*opened));
        if (!s.check(made.has_value(), "composes", made ? "hover" : made.error().context)) return;
        session = std::move(*made);
        session->attach(t.sink());
        const std::string quad = scene_now().vehicles.at(0).name;

        // Selected, tilted 0.6 rad and lifted to 0.6 m with its rotors still: it is dropped, not flown.
        w().panels().selection = editor::Selection{editor::Selection::Kind::object, quad};
        spade::VehicleSpawn start = scene_now().vehicles.at(0).start;
        start.pos.y = 0.6f;
        start.orient = glm::angleAxis(0.6f, glm::normalize(glm::vec3(1.0f, 0.0f, 0.4f)));
        start.vel = glm::vec3(0.0f);
        start.omega_body = glm::vec3(0.0f);
        start.rotor_omega = 0.0f;
        w().panels().pending.push_back(editor::SetVehicleStart{quad, start});
        w().apply_panels();
        then(30);
        s.check(!w().selected_shape().has_value(), "no-shape", "the quad starts with no contact shape");

        // The combo's pick: a box sized from the model's parts.
        const vehicles::ModelType* model = nullptr;
        for (const auto& v : scene_now().vehicles) {
            if (v.name != quad) continue;
            for (const auto& m : scene_now().models) {
                if (m.name == v.model) model = &m;
            }
        }
        if (!s.check(model != nullptr, "model", "the quad's model")) return;
        Result<physics::ShapeDesc> box = editor::shape_for_kind(*model, physics::ShapeKind::box);
        if (!s.check(box.has_value(), "box-sized", box ? "half extents from the parts" : box.error().context)) return;
        w().panels().pending.push_back(editor::SetModelShape{quad, *box});
        w().apply_panels();
        t.sink().set_card("Collision shape: Box\nthe cyan wireframe is what the contact pass tests");
        // The camera on the quad: its start pose at 0.6 m and the ground it will land on; the poster is the next frame.
        session->camera().target = glm::vec3(start.pos.x, 0.3f, start.pos.z);
        session->camera().yaw = 0.52f;
        session->camera().pitch = 0.3f;
        session->camera().distance = 1.4f;
        t.poster("batch");
        then(120);
        const bool shown = w().selected_shape().has_value();
        s.check(shown && w().panels().status.find("refus") == std::string::npos, "shape-set",
                shown ? "the box is on the quad's model, drawn at its start pose" : "no shape: " + w().panels().status);

        // Run: it falls and lands on a face.
        t.sink().set_card({});
        command(editor::Command::run);
        then(150);
        const editor::EditorRun* run = w().run();
        if (!s.check(run != nullptr && run->tick() > 0u, "runs",
                     run != nullptr ? "tick " + std::to_string(run->tick()) : w().panels().status)) {
            return;
        }
        const std::optional<FrameState> now = run->state_of(quad);
        if (s.check(now.has_value(), "state", "the quad's state in the Run")) {
            const glm::vec3 up = glm::conjugate(now->orient) * glm::vec3(0.0f, 1.0f, 0.0f);
            const float aligned = std::max({std::fabs(up.x), std::fabs(up.y), std::fabs(up.z)});
            s.check(aligned >= 0.99863f && now->pos.y < 0.3f, "lands-on-a-face",
                    "a body axis is " + fmt(std::acos(std::min(aligned, 1.0f)) * 57.29578f) + " degrees from up, height " +
                        fmt(now->pos.y) + " m");
        }
        then(30);
        command(editor::Command::stop);
        s.check(t.frames() - frames_at_start <= kBatchChapterMaxFrames, "length-budget",
                std::format("{:.1f} s of video in the chapter's steps (at most {} s, the card besides)",
                            static_cast<double>(t.frames() - frames_at_start) / 60.0, kBatchChapterMaxFrames / 60));
    });

    session.reset();  // detaches its panels while the window is still open
}
