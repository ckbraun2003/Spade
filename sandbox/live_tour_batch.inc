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
// THIS RUN IS THE REFURBISHED TOUR (the audit, 2026-11-13): the catch-up chapter's soft shadows are retired (their
// unit tests judge them), the Subgrid turbulence row and the Run-mode wind row moved into the editor chapter, and this
// chapter shows what the L73a app work added for a tester: the About dialog and its version, Help > User Guide, and the
// "Save changes?" prompt on closing with an unsaved scene.

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
           "\nHelp > About \xC2\xB7 Help > User Guide \xC2\xB7 \"Save changes?\" when closing an unsaved scene";
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
    const fs::path work = t.out_dir() / "batch_work";
    const uint64_t frames_at_start = t.frames();

    // ---- STEP 1: Help > About Spade Editor. The text the host hands the panels names the version and the licence;
    // the dialog is opened as the Help button's entry would, shown, and closed by its Close button.
    t.step("batch", "app", [&] {
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
        w().panels().about = app::about_text(spade::version::kString);
        then(30);
        const std::string& about = w().panels().about;
        s.check(about.find(spade::version::kString) != std::string::npos &&
                    about.find("Apache License") != std::string::npos &&
                    about.find("THIRD_PARTY_NOTICES.md") != std::string::npos,
                "about-text", "the About text names " + std::string(spade::version::kString) + ", the licence and the notices");
        s.check(w().panels().title == "hover.scene.yaml", "title", "the toolbar names the scene: " + w().panels().title);
        w().panels().about_request = true;  // Help > About Spade Editor
        then(150);
        w().panels().about_dismiss = true;  // its Close button
        then(10);
    });

    // ---- STEP 2: Help > User Guide. The guide's file is opened by the host's opener (a stand-in here: the tour opens
    // no browser); a guide that is missing is a line in the panel, never a failure.
    t.step("batch", "guide", [&] {
        if (!open()) return;
        const fs::path guide = work / "guide.html";
        {
            std::ofstream out(guide, std::ios::binary);
            out << "<!doctype html><title>Spade Editor guide</title>";
        }
        fs::path asked;
        w().set_user_guide(guide, [&asked](const fs::path& f) {
            asked = f;
            return true;
        });
        w().panels().user_guide_request = true;  // Help > User Guide
        then(5);
        s.check(asked == guide && w().panels().status.rfind("User Guide:", 0) != 0, "guide-opened",
                asked.empty() ? "the opener was not asked" : "the opener was given " + path_text(asked.filename()));
        t.sink().set_card("Help > User Guide opens the guide in the browser;\nwithout it the panel says so");
        w().set_user_guide(work / "missing.html", [](const fs::path&) { return true; });
        w().panels().user_guide_request = true;
        then(5);
        s.check(w().panels().status.find("the guide is missing") != std::string::npos, "guide-missing",
                "the panel says: " + w().panels().status);
        then(90);
        t.sink().set_card({});
    });

    // ---- STEP 3: closing with an unsaved scene. A drone placed makes the scene unsaved (its title ends " *"); Esc asks
    // "Save changes to <scene>?", and Cancel goes back to the editor with nothing lost or quit.
    t.step("batch", "save_prompt", [&] {
        if (!open()) return;
        const glm::vec3 at = scene_now().vehicles.at(0).start.pos;
        const std::size_t before = scene_now().vehicles.size();
        arm("Phantom 3-class quad");
        click_towards(glm::vec3(at.x, 0.0f, at.z + 1.5f));
        if (!s.check(scene_now().vehicles.size() == before + 1, "placed", w().panels().status)) return;
        then(5);
        s.check(w().unsaved() && w().panels().title == "hover.scene.yaml *", "title-unsaved",
                "the toolbar says: " + w().panels().title);
        w().request_close();  // Esc
        then(5);
        s.check(w().panels().close_prompt == "hover.scene.yaml" && !w().close_confirmed(), "prompt",
                "asks about " + w().panels().close_prompt + ", not quit");
        t.poster("batch");
        then(120);
        w().panels().close_choice = editor::CloseChoice::cancel;
        then(5);
        s.check(w().panels().close_prompt.empty() && !w().close_confirmed() && w().unsaved(), "cancel",
                "back in the editor, the scene still unsaved and the editor still open");
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
