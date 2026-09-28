# Plan C — `spade_sandbox`, the engine's reference application

**Status: WRITTEN 2026-09-17.** Executes `SL10`-`SL13` and `SL15a`/`SL15b`; discharges `SL17` phase
**P5** and its **CHECKPOINT ②**. Companion to Plan A (`SL3`-`SL6`, merged `9ed536f6`) and Plan B
(`SL8`, still unwritten).

**Owner:** Spade realm. **Wave:** W10 of the Phase C plan.

---

## 0. Why this document exists, and what it is not

For eighteen days the spec set has said *"Plan C is unwritten"* while `SL10`-`SL13` described the
sandbox in detail. **The rulings were never the gap.** What was missing is the thing that turns four
rulings into an ordered set of tasks with verification attached — and, more importantly, **a stated
position on what this tool is for**, because `SL12` is a mature-product description and a phase with
twelve other items in it cannot build all of it.

**This plan is narrower than `SL12` on purpose, and section 4 says exactly what it defers.** The
user's framing governs the cut:

> *"Spade is intended to act as a full solo render/physics engine, **like a micro unity**."*
> *"improving user experience and designs to ensure things operate and work smoothly, **not just
> correctly**."*

⭐ **So the cut is by BREADTH, never by FLUENCY.** Half the object types handled smoothly beats all
of them handled awkwardly. A panel that technically permits an edit but makes it laborious has not
landed, and "it is possible in the GUI" is not this plan's bar.

---

## 1. Entry conditions — all three, before task C0

| # | Condition | State |
|---|---|---|
| 1 | **`SL1`-`SL18` signed.** The user's own ruling: *sign before Phase C code starts*. `SL10`-`SL13` is what this plan executes, so it cannot precede its own authority. | ✅ prerequisites cleared — `RS14` fixed (`6776bfde`), `SL16`-`SL18` harvested (`20ea8d45`, `fbfbbe8b`) |
| 2 | **`PC21`'s gate flipped to SIGNED.** `tools/tests/test_spade_sandbox_gate.py` refuses a `spade_sandbox` target while `06-sandbox-and-v1.md`'s token says UNSIGNED. **It is designed to block C0 and it should.** | ⏸ awaiting (1) |
| 3 | **The `spade/` editing boundary lifted**, scoped to the GUI and the sandbox flag. | ✅ ruled by the user, Phase C dispatch |

⚠ **Condition 2 is not a formality and must not be worked around.** The gate flips by changing one
word; if that word has not changed, the signature has not happened, whatever anyone remembers being
said. *A ruling with a gate and no watcher is indistinguishable from a ruling nobody made* — and the
watcher exists now.

---

## 2. The spine: headless first, window second

**This is the single most important sequencing decision in the plan, and it is `SL15b` taken
literally.**

> `SL15b` — *"`spade_sandbox --headless` loads a scene, applies options, steps, renders and dumps
> frames with no window… **Every panel's logic is reachable from it.** The window is a deliberately
> dumb shell… **any logic that migrates into it is a defect, because it becomes untestable by
> construction.**"*

A GUI built window-first satisfies that sentence only by discipline, and discipline is what fails.
**Built headless-first it is satisfied by construction:** every panel's behaviour must already exist
as a callable operation on the model before there is any widget to put it behind, because for the
first three tasks there is no widget at all.

**The concrete rule this plan runs on:** a panel is a *view* over an operation that exists and is
tested without it. If a task cannot state the headless command that exercises its logic, the task is
not ready to start.

⚠ **And this is what makes a GUI testable in a repo whose GPU tests never run in CI.** The sandbox's
gate is `spade.yml` (`spade/tests/` is unreachable from every root build tree — `07-status.md` rule
14 and the header of `spade/tests/CMakeLists.txt`). **That runner has no GPU and no display.**
Headless is not a convenience mode here; **it is the only mode CI will ever execute.**

---

## 3. Tasks

Each task states its **headless command** — the thing CI runs — before its UI.

### C0 — the flag and a target that builds

`SPADE_BUILD_SANDBOX`, **default ON, CI configures OFF**, per `SL15a`. A `spade_sandbox` target that
links `spade::` targets, runs `--headless` over a built-in scene, writes one frame, exits 0.

⚠⚠ **INTRODUCE the flag; do NOT move GLFW/ImGui off `SPADE_BUILD_V1`.** `SL14a` specifies that move
— and `SL14a` is the **quarantine**, correctly blocked on `SL8`'s open SPH row. **Moving them
executes a quarantine that must not execute.** Phase C adds a second, independent gate alongside the
existing one. Getting this wrong fails **all six** workflow configurations at configure time, on a
runner with no X11; getting it right costs **zero** added CI time.

* **Headless:** `spade_sandbox --headless --scene <builtin> --out frame.ppm`
* **Blast radius:** none — a new gated target. `SL2b`'s purity guard acquires its first subject.
* **Verify:** Linux CI green with `SPADE_BUILD_SANDBOX=OFF`; local build green with it ON; the
  out-of-tree consumer still builds with the flag OFF (`SL18` obligation 7).
* **Size: S.**

### C1 — the `TargetSink` seam, headless implementation first

> ✅ **RULED 2026-09-18, after a paper design pass, before any code.** Two things in the draft below
> were changed and the draft's own versions are kept beside them, because each one reads perfectly
> reasonable and that is exactly why it survived review until someone measured.

⛔ **THE NAME `Presenter` IS REFUSED.** `editor/ui/viewport/presenter.h:84` already declares
`class Presenter` — it receives frame packets from the C5 host and puts them in a widget, which is
very nearly this seam's job. **A fourth homograph in an estate that has already paid for three:**
`FramePool` (three distinct objects), `component` (two live senses, `fbfbbe8b`), and `spade_sandbox`
beside v1's `Sandbox` target.

⚠⚠ **AND THE DRAFT OF THIS VERY SECTION CITED THE `FramePool` WARNING BY NAME WHILE PROPOSING THE
COLLISION IT WARNS ABOUT** (see the `PA-1` note below, which still does). ⭐ **A warning quoted as
CONTEXT rather than applied to the sentence it sits in** — the citation made the paragraph read
*more* careful, not less, which is why nobody caught it for three weeks.

**`TargetSink` is the ruled name, and the criterion matters more than the word: NAME THE SEAM FOR
THE TYPE IT CONSUMES.** It takes a `RenderTarget`; the editor's takes a frame packet. ⭐ The
distinguishing word is the thing that actually differs, which is the name a future author **cannot
re-collide by accident.**

```
struct TargetSink {
    virtual void accept(const RenderTarget&) = 0;   // BGRX8, caller-owned (PA-1)
    virtual ~TargetSink() = default;
};
```

⛔ **`should_close()` IS OFF THE SEAM, and this is a correctness argument rather than a tidiness
one.** The draft had `virtual bool should_close() = 0;`. **`HeadlessTargetSink` has no honest answer
to it:** always-`false` means *run forever*, always-`true` means *one frame*, and **both are
arbitrary values invented to satisfy an interface.**

⭐⭐⭐ **AN ARBITRARY ANSWER ON AN INTERFACE IS HOW A LOOP ENDS UP ASKING THE WRONG OBJECT WHEN TO
STOP.** It is the same shape as `sim_tick` being non-optional and zero-defaulted in both frame
mirrors: **the fabricated value is LEGAL, so the fabrication is silent.** ⭐ And it sharpens the
naming criterion into something that does more work — *name the seam for what it consumes* also
tells you **what to leave off**: loop termination is not a property of a thing that receives pixels.

**Loop termination is the APPLICATION's concern.** `C0` has no loop at all. `C2`'s windowed sink
exposes closing on **its own concrete type**, or behind a separate small interface — either way the
headless one is never asked a question it has no opinion about.

* `HeadlessTargetSink` — writes frames to disk. **This is the CI path.**
* The windowed sink — **lands in C2, not here, and it is `SL11`'s signed OpenGL3.** It was ruled
  to Vulkan on 2026-09-18 and **UNRULED the same day** once this realm withdrew the grounds it had
  supplied; see `C2`. ⚠ **This bullet asserted "it is VULKAN" for the whole interval** — a second
  site stating a decision, updated a step behind the section that owns it. *A decision recorded in
  two places is two things to change, and the summary is the one that gets missed.*

⚠ **`RenderTarget` never owns its memory (`PA-1`).** The presenter is *handed* a buffer; it must not
allocate one. This is an interface constraint, not a detail — get it wrong and the sandbox grows a
frame pool the engine deliberately does not have (see `RS14`'s engine-§9 row: three different things
are called `FramePool` in this estate).

⚠ **`SL11` requires re-verification, not assumption:** that ImGui's OpenGL3 backend bundles its own
loader since v1.87, against the **pinned** tag. Measured: the pin is `v1.91.5-docking` and the GLFW
+ OpenGL3 backends are already the ones vendored, so this is expected to hold — **check it anyway,
at implementation.** If it fails, the fallback is the **Vulkan presenter** over the already
unconditional Vulkan-Headers and volk, **never a return to GLAD**.

* **Verify:** `SL10`'s same-path invariant — the frame the sandbox presents is byte-identical to the
  frame `spade::render` produces for the same (scene, camera, options). Assert it on the hash, in
  headless. *A tool that can disagree with the tests is worse than no tool, because it will be
  believed.*

  🔴 **AND THE ASSERTION ABOVE CANNOT FAIL AS DRAWN — A MUTATION CONTROL SHIPS WITH IT OR IT IS
  WORTHLESS.** ⚠ **The invariant is TRUE BY CONSTRUCTION today:** `spade_sandbox` renders into a
  buffer and writes its PPM from **that same buffer**, so there is no seam for anything to diverge
  across. **`C1` creates the first real opportunity for divergence, and this test exists to notice
  it.** But a sink that holds a *reference* to the caller's buffer makes the two hashes **trivially
  equal**, and then the assertion **cannot fail for ANY sink, correct or not.**

  ⭐⭐⭐ **That is *two copies that move together, reported as agreement*** — this realm's own
  competition-700 finding, where `peak × mass / n == thrust_per_motor` held to the last digit in two
  files **for opposite reasons** and could not fail in either.

  **So a deliberately-wrong sink — one that flips a single byte — ships beside the test, and it MUST
  turn it red.** ⭐ Same shape as the `SR-17a` golden pair (`3911d092`): **a test whose subject is a
  seam must be falsifiable by that seam misbehaving, and the only way to know that it is, is to
  BUILD the misbehaviour and watch it fire.** Asserting the property is not the same as owning an
  instrument that can detect its absence.
* **Size: M.**
* ✅ **FREE OF CI RISK, which is why it is separated from `C2`:** no windowing, no GLFW, no ImGui,
  no change to `spade/vendor/CMakeLists.txt`'s gate. It can land in Wave 4 with nothing to decide.

### C2 — the window, and nothing else in it

> 🔴🔴 **THE BASIS FOR THIS RULING IS WITHDRAWN BY THE REALM THAT SUPPLIED IT, 2026-09-18. THE
> RULING ITSELF STANDS — it is the user's word — BUT IT WAS OBTAINED ON AN ARGUMENT THAT IS WRONG
> IN BOTH OF ITS HALVES, AND A RE-RULING IS INVITED.**
>
> **① "A bare Linux CI runner has no X11/Wayland dev packages" — THAT RUNNER DOES NOT RUN.**
> `docker/ci-linux.Dockerfile:4-7` states it plainly: *"Every GitHub-hosted job has failed in ~2 s
> since 2026-08-27 on a payments error, so `.github/workflows/` is retained UNRUN … Until then the
> runners in this directory are what 'CI passed' means in this repo."*
> ⚠ **THE QUOTED DATE IS WRONG AND THE QUOTED CONCLUSION IS RIGHT** (measured 2026-09-18): hosted
> CI had runners assigned on both platforms **2026-09-01 → 09-07** (7 calendar days; boundaries
> measured by runner assignment, not by run conclusion), so *"since 2026-08-27"* is not continuous.
> The sentence this quotation is used for — **our own image is the gate** — is unaffected. ⭐ *I
> quoted another file's unmeasured claim as authority; a second witness that quotes the first is one
> witness.* **The actual Linux gate is
> `kat-ci-linux:local`, built from a Dockerfile THIS ESTATE OWNS**, whose install line is
> `build-essential gcc-13 g++-13 cmake ninja-build git curl ca-certificates python3 …` — it has no
> X11 **by choice, not by constraint.** ⭐ **I described a one-line change to a file we control as
> an immovable property of someone else's infrastructure.**
>
> **② "Vulkan-Headers and volk are already unconditional, so the gate stays intact" — TRUE ABOUT
> VULKAN AND FALSE ABOUT THE WINDOW.** ⭐⭐⭐ **VULKAN DOES NOT CREATE WINDOWS.** A swapchain needs a
> `VkSurfaceKHR`, which is created from a **native window handle** (`vkCreateWin32SurfaceKHR`,
> `vkCreateXlibSurfaceKHR`), which needs a window and therefore X11/Wayland on Linux exactly as
> GLFW does. **Switching OpenGL3 → Vulkan does not remove the windowing dependency; it changes only
> which API draws into the window.** The thing the ruling was chosen to avoid is not avoided.
>
> ⚠⚠ **SO THE DECISION WAS PUT TO THE USER AS "DIVERGE FROM YOUR SIGNED TEXT TO PROTECT THE LINUX
> LEG", AND IT PROTECTS NOTHING.** The real question is far smaller and lives entirely in files we
> own: **add X11/Wayland dev headers to `docker/ci-linux.Dockerfile`, and move the GLFW/ImGui fetch
> out of `if(SPADE_BUILD_V1)`.** With that done, OpenGL3 vs Vulkan is an ordinary graphics-backend
> choice on its merits — and on merits `SL11`'s **signed** OpenGL3 is the cheaper one: ImGui's
> OpenGL3 backend is already vendored, already verified, and needs no swapchain, device selection
> or present loop.
>
> **RECOMMENDED: revert to `SL11` as signed (OpenGL3), and make the two one-line changes above.**
> ⛔ **Not enacted.** The user ruled Vulkan and only they can unrule it; this note records that the
> grounds they were given were unsound, which is the one thing that must not stay buried.
>
> ⭐⭐⭐ **AND THE SHAPE IS THIS PHASE'S OWN:** I escalated *because* the divergence was substantive,
> insisted it needed the user's own words, and got them. **The rigour of the escalation is what made
> the false premise credible** — *the quality of the finding was the alibi for its not having been
> checked.* A claim does not become true by being escalated carefully.
>
> 🔴 **UNRULED BY THE USER, 2026-09-18, AFTER THIS REALM WITHDREW THE GROUNDS IT HAD SUPPLIED.
> THE BACKEND IS `SL11` AS SIGNED: OpenGL3.** Their words: *"Revert to SL11 as signed
> (recommended)."* The Vulkan ruling below is **struck** and is kept only as the record of what was
> decided and undecided — see the four-state table further down, and the withdrawal above it.
>
> ⭐ **A SIGNED CLAUSE IS THE DEFAULT AGAIN, WHICH IS WHERE IT SHOULD HAVE STAYED.** The divergence
> was sought to protect the Linux leg and protected nothing; with the basis gone there is no reason
> to be anywhere other than the text the user already signed.
>
> ~~✅ **RULED BY THE USER, 2026-09-18: the windowed sink is VULKAN.**~~ ⛔ STRUCK. This supersedes `SL11`'s
> signed *"ImGui runs on its OpenGL3 backend"*, on the user's own authority about this specific
> thing. Relayed through the coordinator, which the user authorised for exactly this.
>
> 🔴 **AND THIS RIDER TRAVELS WITH THE CHANGE OR THE NEXT READER DRAWS A FALSE CONCLUSION:
> `SL11` CONTAINS A VULKAN FALLBACK, AND IT IS *NOT* WHY WE ARE TAKING VULKAN.** That fallback's
> stated trigger is *the ImGui-loader claim failing.* **The claim was re-verified on 2026-09-18 and
> it HOLDS** — pin `v1.91.5-docking`, `imgui_impl_glfw.cpp` + `imgui_impl_opengl3.cpp` vendored.
> **The trigger did not fire.** We take Vulkan for the Linux-leg reason below, which `SL11` does not
> name. ⭐⭐ **Without this paragraph a future reader sees "the Vulkan presenter", reads `SL11`'s own
> sentence, and correctly infers the loader claim failed — which is false, and was measured false.
> A correct-looking inference drawn from a true sentence is harder to catch than a wrong sentence,
> because there is nothing to disagree with.**
>
> ⚠ **The ruling was put to the user WITH the divergence named** — *"diverges from your signed text,
> but for a reason the text does not name"* — so the ratification is **knowing**, not a general
> approval read as a specific one.
>
> #### ⚠⚠ THIS ROW SAID THREE DIFFERENT THINGS IN ONE HOUR. THE HISTORY IS KEPT ON PURPOSE.
>
> | at | said | correct? |
> |---|---|---|
> | `e7afaac5` | **RULED** | ❌ **No — nobody had ruled.** A coordinator recommendation transcribed as a ruling |
> | `4dd82c0c` | **RECOMMENDED, NOT RULED** + *`SL11`'s OpenGL3 governs meanwhile* | ✅ **Yes, at the moment it was written** |
> | `f0586ba3` | **RULED (user, 2026-09-18): VULKAN** | ⚠ **Correctly recorded, unsoundly obtained** — the ruling was real, its grounds were false |
> | `18662f82` | grounds **WITHDRAWN** by the realm that supplied them | ✅ Yes |
> | here | 🔴 **UNRULED (user). `SL11` as signed: OpenGL3.** | ✅ Yes |
>
> ⭐⭐⭐ **THE SIGNED TEXT AND THE LAST ROW BOTH SAY OpenGL3, AND THE JOURNEY BETWEEN THEM IS NOT
> WASTE — IT IS THE ONLY REASON ANYONE KNOWS THAT CLAUSE SURVIVED EXAMINATION RATHER THAN MERELY
> NEVER HAVING BEEN EXAMINED.** A reader arriving at the last row alone would conclude nothing
> happened here.
>
> ⚠ (An earlier draft of this sentence said *"rows 1 and 6"*. **The table has five rows, and row 1
> is already the error state, not the signed one.** Written without counting — the fourth fabricated
> figure this realm has put into a tracked file in one session. **The failure is not arithmetic: it
> is reaching for a number to make a sentence land, when the sentence did not need one.**)
>
> ⚠⚠ **AND THE LAST ROW EXISTS ONLY BECAUSE THE REALM THAT WON ROW 4 WENT BACK AND CHECKED ITS OWN
> ARGUMENT.** The escalation that produced it was careful about **who may decide** and careless
> about **whether the thing being decided was true.** Two separable disciplines — and only the
> first had a rule attached. *A signature is recorded only from the user's own words* protects
> against the **wrong person** deciding; **nothing protected against the right person deciding on a
> false premise, and the premise was mine.** ⭐ **A claim does not become true by being escalated
> carefully — and a careful escalation is HARDER to challenge than a careless one, which is the
> whole danger.**
>
> ⭐⭐⭐ **`e7afaac5` AND `f0586ba3` BOTH SAY "RULED", AND THE AUTHORITY BEHIND THEM IS NOT THE SAME
> — WHICH IS THE ENTIRE DISTINCTION `4dd82c0c` EXISTED TO PROTECT.** (Cited by SHA rather than by
> row number, because row numbers move when a row is added and a SHA does not — the same
> immutable-reference lesson this realm learned from a positive control pinned to `HEAD`.) A reader skimming `git log` sees RULED → NOT
> RULED → RULED and reads a flip-flop. It was not one: **the first was a transcription error and the
> third is a decision.** Row 2 is not an embarrassment to be tidied away; **it is the only reason
> anyone can tell rows 1 and 3 apart.**
>
> **How row 1 happened, recorded because it is the reusable part:** I was told `C2` was ruled, and I
> **wrote down what I was told at the same moment I was disputing whether it COULD be ruled.** The
> dispute and the transcription were **one action**, and **only one of them carried my scepticism.**
> ⭐ *Challenging a claim and recording it are different faculties, and recording is the one that
> persists.*
>
> ⭐⭐ **And "undecided" would have been the wrong correction too** — there WAS a decision in force,
> the signed one. **A row reading *"pending"* without naming what governs meanwhile invites the next
> implementer to pick, which is how an open question becomes a silent choice.** Say what is pending
> AND what is in force.

A windowed `TargetSink` + an ImGui context + a main loop. **No panels.** The window's entire job is
to accept a buffer and pump events.

⚠⚠ **WHY NOT GLFW: `C2` AS DRAFTED BREAKS THE ONE LEG THAT MAKES A RENDER GOLDEN MEAN ANYTHING.**
Measured in `spade/vendor/CMakeLists.txt`: **GLFW `3.4` (`:9`-`:20`) and ImGui `v1.91.5-docking`
(`:42`-`:70`) are BOTH inside `if(SPADE_BUILD_V1)`**, and the file states its own reason at `:4-5` —
*"a bare Linux CI runner has no X11/Wayland dev packages installed, and
`FetchContent_MakeAvailable(glfw)` would otherwise fail its own configure."*

⭐⭐⭐ **THE v2 LINUX LEG PASSES *BECAUSE* IT CONFIGURES `V1=OFF`.** A GLFW-based sink in v2 forces
that fetch **out of the gate**, making X11/Wayland **v2's problem** — on the only automated leg that
proves cross-platform bit-identity. **That is the leg that independently computed `a4dec569…` and
`60e6accc…`.** ⚠ **Breaking it does not merely redden CI: it removes the instrument that makes a
render golden mean anything**, and a golden nobody can verify on a second platform is a number, not
a check.

**Vulkan-Headers and volk are ALREADY unconditional**, so the Vulkan route leaves the gate — and the
Linux leg — exactly as they are today.

⛔ **The third option was considered and REFUSED: keep the window Windows-only in CI.** That means
**the windowed path is never proven cross-platform**, which would then be *discovered* rather than
*decided*. ⭐ *An unstated cost does not get chosen; it gets defaulted into.*

⚠ **A NOTE ON WHY THE DRAFT'S OWN VULKAN FALLBACK DID NOT COVER THIS.** `C1` already named a Vulkan
fallback — but **against an ImGui *loader* risk which it also measured as unlikely.** The measurable
risk is the **CI environment**, and nobody had written it down. ⭐⭐ **A mitigation pointed at the
improbable failure is not cover for the probable one**; it answered both here by luck.

✅ **`SL11`'s "re-verify the ImGui loader claim at implementation" is RETIRED EARLY, answered on
paper 2026-09-18:** the pin **is** `v1.91.5-docking` and `imgui_impl_glfw.cpp` +
`imgui_impl_opengl3.cpp` **are** the vendored backends (`:59`-`:60`). ⭐ **That check was real and is
answered; the one that mattered was the one nobody had written down** — which is the argument for
running a design pass at all.

* **Verify:** the same-path hash assertion from C1 — **with its mutation control** — passes through
  the windowed path too, so the two sinks are proven interchangeable before any UI depends on it.

#### 🔬 HOW THE SINK REACHES GL — measured 2026-09-18, and the answer came from the library

The windowed sink must create and upload one texture. On the Linux gate there are **no GL headers**,
so *how* it reaches GL decides whether it compiles there. Four probes:

| probe | result |
|---|---|
| ImGui's `imgui_impl_opengl3_loader.h` from a foreign TU | ✅ `glGenTextures`, `glBindTexture`, `glTexImage2D`, `glTexParameteri`, `GL_RGBA`, `GL_LINEAR`, `glViewport`, `glClear` **all resolve with no system GL** |
| …`GL_RGB`, `GL_NEAREST`, `GL_UNPACK_ALIGNMENT` | ❌ absent — the loader is filtered to what ImGui itself uses |
| …`glTexSubImage2D` | ❌ absent (`glTexImage2D` per frame is the substitute; at sandbox resolutions the realloc is free) |
| `GL_BGRA` | ❌ absent — a BGRX→RGBA channel swap is required, and it is the same loop `HeadlessTargetSink` already runs |

⛔ **AND THE LOADER IS NOT AVAILABLE TO US, ON ITS OWN INSTRUCTION.** Its header says, in capitals:
*"YOU SHOULD NOT NEED TO INCLUDE/USE THIS DIRECTLY. THIS IS USED BY `imgui_impl_opengl3.cpp` ONLY."*
⭐ **That is a measurement, not a preference** — every missing symbol above is a consequence of the
same fact: **it is filtered to one consumer's needs and we are not that consumer.** Building against
it would make our sandbox's compilation depend on which GL calls a vendored library happens to use
this release.

**RULED (this realm, reviewable): the sink includes the SYSTEM GL header, and its GL half is guarded
on `SPADE_GLFW_HAS_BACKEND`.**

* **Cost, stated rather than discovered: the windowed sink gets NO Linux compile coverage.**
* ⚠ **And that cost is smaller than it first appears, for a reason already measured above: NO option
  gives cross-platform EXECUTION of a window** — the Linux gate has no display and, now, no GLFW
  backend. **What is lost is compile coverage of a path that cannot run there**, and it is lost to
  avoid violating a library's explicit instruction.
* ⭐ **Mitigation that recovers most of it:** the sink's **non-GL** logic — the BGRX→RGBA conversion,
  the frame budget, the refusal path — lives in a TU that compiles **everywhere**. Only the GL calls
  are guarded. **Linux keeps coverage of the logic and loses only the calls.**
* 🔴 **THE REFUSAL IS PART OF THE DELIVERABLE, NOT AN ERROR PATH.** With no backend, the sink must
  say so and name the cause — *"this build has no windowing backend; GLFW was configured without one
  because no X11 dev headers were present"* — and exit non-zero. **A missing capability must refuse,
  never degrade**, and a window that silently fails to appear is the worst version of this.

#### C2's work on the reverted basis, with every claim's MEASUREMENT STATUS marked

> ⭐ **Marked, because of what this section cost the first time.** The standing rule minted from that
> episode is *an escalation carries its premise's measurement, or it is not an escalation.* It binds
> a plan as much as a question: **a claim written here without its measurement will be read as
> measured by whoever implements it.** Each item below says ✅ MEASURED (with what settled it) or
> 🔬 NOT YET MEASURED (with the command that would).

| # | the work | status |
|---|---|---|
| 1 | Move the GLFW + ImGui `FetchContent` out of `if(SPADE_BUILD_V1)` in `spade/vendor/CMakeLists.txt` | ✅ **MEASURED** — both blocks are gated today at `:9`-`:20` and `:42`-`:70`, read directly |
| 2 | ~~Add X11/Wayland dev headers to `docker/ci-linux.Dockerfile`~~ | 🔴 **RETIRED — NOT NEEDED. MEASURED 2026-09-18, three probes, each with a control.** See below |
| 3 | `GlTargetSink`: GLFW window + ImGui OpenGL3 backend, BGRX8 as a texture on one fullscreen quad | ✅ **MEASURED** that the backend sources are vendored (`imgui_impl_glfw.cpp`, `imgui_impl_opengl3.cpp`, `spade/vendor/CMakeLists.txt:59`-`:60`) and the pin is `v1.91.5-docking` |
| 4 | `SL10`'s assertion through the windowed path, mutation control included | ✅ **MEASURED** that the headless half works — `SandboxTargetSink`, 4 cases, green on both platforms at `ac18c480` |

> 🔴🔴 **MEASURED 2026-09-18 ON THE REAL CI IMAGE, AND ITEM 2 IS RETIRED: `C2` NEEDS NO NEW PACKAGES
> AND NO DOCKERFILE CHANGE AT ALL.** Three probes, each paired with a control that had to fail:
>
> | probe | control (must fail) | result |
> |---|---|---|
> | GLFW with `GLFW_BUILD_X11=OFF GLFW_BUILD_WAYLAND=OFF` | same configure with `X11=ON` | ⛔ control **FAILED at `find_package`** (status 1) · ✅ **configure 0, build 0, `libglfw3.a` linked** |
> | `imgui_impl_opengl3.cpp` standalone | — | ✅ **status 0** — the bundled loader is real, now confirmed **by compilation** rather than by reading the vendor list |
> | `imgui_impl_glfw.cpp` with `-DGLFW_INCLUDE_NONE` | same file **without** the macro | ⛔ control **FAILED** (`glfw3.h:241` pulls `<GL/gl.h>`) · ✅ **status 0 with the macro** |
>
> `X11_DEV_HEADERS_PRESENT=no`, `GL_HEADERS_PRESENT=no`, `EGL_HEADERS_PRESENT=no` on the image — so
> every green above was obtained **with nothing installed.**
>
> **The recipe is therefore three CMake settings and no apt line:** `GLFW_BUILD_X11=OFF` and
> `GLFW_BUILD_WAYLAND=OFF` on UNIX, and `GLFW_INCLUDE_NONE` on the sink's own target. The windowed
> sink **compiles and links on the Linux gate**; `glfwInit()` would fail there at runtime, which
> costs nothing because that gate has no display under any option.
>
> 🔴🔴 **SO THE ARGUMENT THAT PERSUADED THE USER TO SUPERSEDE THEIR OWN SIGNED TEXT WAS NOT MERELY
> WRONG IN ITS TWO STATED HALVES — THE THING IT FEARED CANNOT HAPPEN UNDER ANY OPTION.** X11/Wayland
> was never going to arrive on the Linux leg. **The gate never had to move, and the divergence was
> unnecessary three times over.** ⭐ Each probe cost under a minute, and the first one contradicted a
> claim I had already made twice with confidence.
>
> ⚠ **The paragraph below is kept as written, unedited, because it is the record of what was still
> unknown an hour ago** — and because a plan that silently absorbs its own open questions teaches
> the next reader that there never were any.

**🔬 THE OPEN QUESTION, AND I AM NOT PUTTING IT TO ANYONE UNTIL I HAVE RUN IT.** GLFW 3.4 declares
`GLFW_BUILD_X11` and `GLFW_BUILD_WAYLAND` as `cmake_dependent_option(... ON "UNIX;NOT APPLE" OFF)`
(read in the vendored source). **They are ON by default and they can be turned OFF.** So there may be
a route where the Linux CI image needs **no windowing dev headers at all**: build GLFW with both
backends off, so the sink **compiles and links** on Linux and simply cannot open a window there.

⚠ **Whether GLFW even configures with both off is UNMEASURED**, and it needs one container run
(`cmake -S spade -B /build -DGLFW_BUILD_X11=OFF -DGLFW_BUILD_WAYLAND=OFF`). **Until that runs, item 2
above is a cost that may or may not be real, and stating it as settled would repeat the exact defect
this section is a monument to.**

**⭐⭐⭐ AND A REFRAMING THAT APPLIES TO EVERY OPTION, INCLUDING THE ONE ALREADY REJECTED.**
*"Proven cross-platform"* for a **windowed** path can only mean **COMPILED AND LINKED on both**, never
**EXECUTED on both** — because the Linux gate has **no display**, whichever graphics API is chosen.
Option (c) (*"keep the window Windows-only in CI"*) was refused on the grounds that the windowed path
would then never be proven cross-platform; ⚠ **no available option delivers cross-platform
EXECUTION of a window.** The difference between (c) and the others is compile coverage, which is
real and worth having — but it is a smaller difference than the refusal implied.

⛔ **This does not reopen the ruling.** OpenGL3 is right on merits either way, and the user has
ruled. It is recorded because **the next person to read that refusal will otherwise inherit a
stronger claim than the evidence supports** — and this section has already taught this realm what
that costs.

* **Size: M.**
* 🔴 **`C1` AND `C2` DO NOT SHIP AS A PAIR.** `C1` carries zero CI exposure; `C2` carries the whole
  decision above. ⭐ **Splitting two tasks by RISK rather than by ORDER is the durable output of this
  design pass** — the draft sequenced them adjacently because one follows the other, which hid that
  only one of them could break the estate's cross-platform instrument.

### C3 — scene sources and the picker

`SL13`'s first two of three sources: **curated presets** and **test scenarios** (through the
existing `ScenarioData` loader, so any failing case can be opened and watched).

⛔ **The sandbox does not write `.world.yaml` or Kat recipes.** *"A second authoring home for
production content is exactly the outcome this clause exists to prevent."*

* **Headless:** `--scene <name>` resolves against both sources; `--list-scenes` enumerates them.
* **Size: S** (the loaders exist).

### C4 — hierarchy and inspector

`SL12`'s first half, over `SL4`'s object model — **which is already built and merged**, and is why
this is affordable. A tree of objects; select one; see its components; **edit their configuration
live**.

⭐ **This is the task that decides whether the tool is an engine editor or a scene viewer**, and it
is where the fluency bar applies hardest. Concretely, for this task, *fluent* means: selection is
one click; an edited value takes effect on the next step with no apply button; the tree shows
nesting; and an object's components are visible without hunting.

* **Headless:** `--inspect <object>` prints the component set and configuration; `--set
  <object>.<component>.<field>=<value>` applies one edit and steps. **Every inspector edit is that
  operation with a widget in front of it.**
* **Size: L.** The largest single task in the plan.

### C5 — add object, add component

From `SL5`'s registered types — **nine functional**; `fluid` (id 9) is declared and reserved for
Plan B.

#### ⭐⭐⭐ `fluid` STOPS BEING A HARMLESS DECLARATION THE MOMENT THIS TASK SHIPS

**Credit where due: this is Runtime's test applied to my own list, and it reclassifies a row I had
already marked KEEP.**

> **Not "is it implemented" but "does anything ASSERT it to someone who will act on it."
> A roster entry is not a field.**

`ComponentTypeId::fluid` is **declared-and-unread today**, which is correctly a keep — it is
reserved so that adding SPH later cannot force a renumber of frozen ids, and deleting it would
discard a right decision. **C5 is what converts it into a roster entry**, and the conversion is
invisible unless someone says so in advance. That is what this paragraph is for.

⚠ **And the tree makes the WRONG implementation the natural one.** `graph.cpp:30` holds
`kNames`, a `std::array<std::string_view, kComponentTypeCount>` of **all ten** names including
`"fluid"`, with two public accessors over the full range:

```cpp
std::string_view      component_type_name(ComponentTypeId);        // graph.cpp:38
std::optional<ComponentTypeId> component_type_id_from_name(std::string_view);  // graph.cpp:44
```

The obvious menu is `for (i = 0; i < kComponentTypeCount; ++i) add(component_type_name(i))` — and
it advertises a type the engine cannot construct, to a user who will then attach it and build
around it. **The correction arrives after the work. Loud and correct is still a bad experience.**

⚠⚠ **The roster surface is bigger than the menu, and this is the part I did not expect:**
`component_type_id_from_name("fluid")` **resolves today**. `SL4` serializes a graph by component
*name*, so a scene file naming a fluid component already gets a valid id back. The GUI is not
introducing the roster; **it is the first consumer that will put the roster in front of a person.**

**THE REPAIR BELONGS ON THE TABLE, NOT ON THE MENU.** Filtering in the UI means the *next*
enumerator — a scene loader, a CLI `--list-component-types`, a docs generator — repeats the defect,
because the knowledge that one entry is reserved lives in `04-objects.md` and not beside the enum.
So C5 adds **availability as a declared property next to `kNames`**, and every enumerator reads it:

* a per-type availability value — available, or reserved-with-a-reason and the plan that lands it;
* `component_type_id_from_name` keeps resolving `"fluid"` (**do not break the id space**) but the
  construction path refuses it by that property, with the reason in the message;
* the menu shows nine and **says why the tenth is absent rather than hiding it** — a reserved entry
  the user can see and not select is honest; an absent one invites the question again next release.

**Verify by mutation:** flip a functional type to reserved and the menu must lose it *and* name it;
flip `fluid` to available and construction must fail on the missing pass, not on the property.
`SL18`: *every guard must be shown to fail under the mutation it exists to catch.*

**How many consumers does the property have? ONE — measured, and the measurement has an expiry.**
Runtime swept `runtime/`, `components/`, `tools/`, `train/` and `editor/` for
`component_type_id_from_name`, `component_type_name` and `ComponentTypeId` across `.cpp`, `.h` and
`.py`: **zero hits in all five trees** at master `a2c38d6f`. Kat's C3 vocabulary (dotted `type_id`
strings like `sensing.imu`) and Spade's `ComponentTypeId` enum are **separate namespaces that never
meet**, so a Kat manifest name cannot reach the Spade resolver and the availability property has one
consumer, not two.

⚠ **That zero is true *because nothing bridges the two vocabularies*, not because a bridge exists
and is unused — and C5 is a plausible place to build one.** If this task introduces a manifest
naming a Spade object type, or a graph referencing a scene component by name, **the answer changes
the moment the bridge lands and nothing will announce it.** Re-ask at the point a bridge is
designed; do not carry this line forward as settled. *A finding and a constraint are different
tenses of the same sentence, and only one of them expires.*

#### The rest of C5

* **Headless:** `--add-object`, `--add-component <type>`, `--list-component-types` (which must show
  the reserved one and its reason — that is the anti-vacuity control on the property above).
* ⚠ **`SL4`'s spec-vs-built deviation becomes live here.** The spec says attach/detach are *queued
  to step boundaries*; the built graph mutates **immediately**, harmless today only because nothing
  holds an `ObjectGraph`. **A GUI attaching components to a running scene is exactly the condition
  that makes it matter.** Either the sandbox attaches only between steps (cheap, and what this plan
  assumes), or the queue gets built. **Decide at C5; do not discover at C5.**
* **Size: M**, and it was S before the availability property. That difference is the cost of the
  reclassification, and it is worth paying here rather than in a bug report.

### C6 — live render and physics controls

Draw mode (shaded / wireframe / raymarch / velocity per `SL9c`), sun direction, sky, shadows on/off,
gravity, timestep, substeps. **These are existing `RenderOptions` and world fields** — this task is
wiring, not rendering work.

* **Headless:** `--set-option <k>=<v>`, already needed by C4's machinery.
* **Size: S.**

### C7 — pause, single-step

On the existing `kSnapshotVersion` machinery. **Pause and step only; scrub is deferred** (§4).

* **Size: S.**

### C8 — `SL2b`'s purity guard goes live

The sandbox's include graph touches **only installed headers**; it links only `spade::` targets.
This guard has existed as a specification with **no subject** since `SL2b` was written.

* **Verify:** seeded violation — an `#include` reaching into an engine internal by relative path
  must fail the guard. `SL18`: *every guard must be shown to fail under the mutation it exists to
  catch.*
* **Size: S.**

---

## 4. What this plan defers, stated rather than discovered

| Deferred | Why |
|---|---|
| **Saved sandbox scenes** (`SL13`'s third source) | A new persisted format is a compatibility obligation forever. Presets + scenarios prove the picker; saving lands once the panel set has stopped moving. |
| **Asset browser** over the prefab kit, mesh library and material palette | Genuinely useful, genuinely large, and it needs the content trees to hold still. |
| **Side-by-side CPU/Vulkan lockstep parity view** (`SL12`) | **There is no Vulkan render backend in the tree.** Scoping this is scoping S7b. |
| **Physics debug draw** — contacts, normals, AABBs, broadphase grid | The depth-biased overlay path exists, so it is feasible; each visualisation is its own correctness question. Second wave. |
| **SDF slice / distance-field views** | Same. |
| **Scrub with snapshot restore** | Pause/step is the cheap 80%; scrub needs a snapshot ring and a UI for it. |

**The honest sentence, so nobody reads this plan as promising Unity:** *what C0-C8 deliver is an
**inspector and a viewport** — open a scene, walk its object graph, change component values and
render settings live, step the simulation, and watch the result through the same render path the
tests use. You will not be able to save what you made, browse assets, or watch CPU and GPU disagree.
Those are the three things that would make it feel like Unity rather than like an engine debugger,
and each is a wave of its own.*

---

## 5. What Plan C does **not** unblock

⚠ **Finishing this plan does not advance the v1 quarantine.** P6 (`SL14b` successor presets) and P7
(`SL14a` quarantine) still wait on **Plan B**, because the transfer register keeps one open row —
SPH fluid — and *v1 must not be quarantined while it is the only implementation of anything*
(`SL7`).

⚠⚠ **And `SL14b` carries a use-it-or-lose-it obligation that Plan C is the precondition for.**
Successor-scene baselines must be captured **from the live v1 tools, before quarantine** — *"the
last moment this is possible."* Plan C builds the successors' home; **capturing P0's baselines is
still owed and still has to happen while v1 builds.** Nothing in this plan does it, and nothing
downstream will notice it was skipped until v1 is gone and the comparison has no left-hand side.

---

## 6. Verification summary

| Obligation | Discharged by |
|---|---|
| `SL10` same-path invariant | C1 hash assertion, re-asserted through the windowed path in C2 |
| `SL11` presenter seam | C1; the interface is what makes S7b a substitution rather than a rewrite |
| `SL15b` headless is the test surface | The spine — every task states its headless command before its UI |
| `SL18` obligation 6, public-surface purity | C8, with a seeded violation |
| `SL18` obligation 7, standalone consumption | C0, consumer builds with `SPADE_BUILD_SANDBOX=OFF` |
| `SL17` P5 / **CHECKPOINT ②** | The whole plan: *the sandbox opens a test scenario, a preset and a saved scene; every control operates live* |

⚠ **CHECKPOINT ② names "a saved scene" and §4 defers saving.** Stated here rather than left for the
checkpoint to fail on: **as scoped, this plan meets two of that checkpoint's three sources.** Either
the checkpoint is signed on presets and scenarios with saving noted as a follow-on wave, or C-next
adds saving before ② is called. **That is the user's call and it should be put to them at C3, not at
the checkpoint.**
