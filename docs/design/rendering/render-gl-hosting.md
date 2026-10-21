# Hosting render_gl: the contract

For a host that draws through `spade::render_gl::GlRenderer` (`engine/render_gl/gl_renderer.hpp`): Spade's sandbox and editor today, and Kat's viewport through `kat_host_spade` (Cameron's choice, 2026-10-20). Rendering owns this page; ask Rendering, copying the lead, when it doesn't answer you.

## Context

- **The host creates and owns the GL context, the window and the present.** The renderer never creates a context and knows nothing of GLFW, Qt or a swap chain.
- **OpenGL 4.3 or later is required**, with shader storage buffers. Shaders are `#version 430 core`, so either a core or a compatibility profile works.
- **Entry points come from the host.** `create(loader)` takes the host's loader (`glfwGetProcAddress`, Qt's, Kat's). Each renderer loads its own private table, so it exports no loader symbols to clash with the host's.
- A context below 4.3 is refused by `create()`, naming the version it found. Nothing falls back silently.

## Threads

- **Every member call needs the renderer's context current on the calling thread:** `create`, `upload_scene`, `draw`, `flow_ready`, the destructor, and the getters and `set_flow_quality` (no GL inside, but no locking either). Make all of them from one thread at a time, the GL thread.
- **Off the GL thread, freely:** building the `RenderScene` and everything that feeds it, which is pure CPU on data the host owns. That covers `scene_from_world`, `resolve_meshes`, `bind_parts`, `update_dynamics`, `apply_view`, `supports`, `base_legend`, `segment_legend`, `render::render()` (the CPU backend), and `GlRenderer::unhonoured()`, which is static and needs no context.
- **A scene must not change while `draw()` reads it.** Neither may any `FlowVolume` its layers point to.

## Lifetime

- **Create** once the context is current. `create()` compiles everything before returning, so a driver failure shows up there and not at the first frame. The exception is the flow programs, with `CreateOptions::flow_programs_in_background` (see "Refusals").
- **Resize:** there is no call. Pass the target's size to every `draw()`; internal targets follow it.
- **Destroy** before the context goes, with the context current. The destructor deletes the renderer's GL objects.
- **Context loss or recreation:** nothing survives a context. Destroy the renderer while the old context is current, or simply drop it if the context is already gone. Then `create()` a new one and `upload_scene()` again.

## Order of calls

1. Build the scene. Resolve its meshes (`resolve_meshes`), call `scene_from_world`, and `bind_parts` each vehicle at its starting pose.
2. Call `upload_scene(scene)` whenever the **mesh set** changes: at load, and when an object or vehicle is added or removed. It uploads geometry, materials, the static shadow map and the CSG programs. A refusal changes nothing: the last accepted scene still draws.
3. Call `draw(scene, camera, options, width, height)` each frame. Moving things needs no upload: bodies through `update_dynamics`, parts by re-binding them, with the same meshes, into a copy of the scene from before they were bound.
4. **Exception: CSG parts.** `bind_parts` bakes a CSG part's pose into its program, which lives on the GPU. After moving a vehicle that has CSG parts, call `upload_scene()` before `draw()`. Otherwise `draw()` refuses: "CSG subtree *i*'s program … is not the one uploaded: moved or edited since upload_scene() -- upload this scene again". The reference quad has no CSG parts.
5. **Flow volumes** are cached per `FlowVolume::generation`. After any write to a volume's data, give it a new generation (`next_flow_generation()`) or 0. Otherwise `draw()` refuses in debug builds, and in release builds it shows the stale volume.

## The framebuffer

- **Target:** `draw()` renders into the **currently bound** draw framebuffer (the window's or an FBO) at `(0, 0, width, height)`. It writes every pixel, so the host need not clear colour. It clears depth itself. It never presents.
- **Depth attachment:** required, 24 bits or more. Stencil is unused.
- **Colour:** 8-bit UNORM, alpha written as 1.
- **No sRGB:** the renderer writes display-ready values, the same bytes `render::render()` writes. `draw()` disables `GL_FRAMEBUFFER_SRGB` on entry, so they are never encoded twice. An sRGB attachment stores them as written.
- **MSAA:** fine for shaded, wireframe and velocity, whose edges blend. **Refused for the sensor modes** (depth, normals, segmentation): "GlRenderer::draw: *mode* draws data, which a multisampled target (*n* samples) would blend at every edge; draw it into a single-sample target". For a sensor view on an MSAA window, draw into a single-sample FBO and blit.
- **Its own framebuffers:** a frame with dynamic shadow casters or a flow layer draws into those first, then rebinds yours.

## Refusals a host can hit

Each is a `Result` error with the text below, and the frame or upload is not drawn. Show it; never retry blindly.

- **`create`:**
  - "null proc loader";
  - "the loader found no *fn* -- is a GL 4.3 context current on this thread?";
  - "needs OpenGL 4.3 for shader storage buffers; this context reports …";
  - "link *program*: *log*" (a driver bug: report it).
- **`upload_scene`:**
  - mesh *i*'s positions and normals differ in count;
  - the static shadow map isn't size × size;
  - "this driver cannot render to an R32F texture, which the shadows of dynamic bodies need";
  - a CSG subtree with no program, a bad draw item, or an invalid program;
  - more than 64 nodes (`kMaxCsgSubtreeNodes`) in a subtree;
  - CSG programs past the driver's storage-block limit.
- **`draw`:**
  - "raymarch is a CPU technique" (Code::unavailable);
  - "zero framebuffer size";
  - "this scene has *n* meshes [CSG subtrees] but *m* are uploaded -- call upload_scene() first";
  - the CSG move above;
  - "the flow shaders are still compiling (flow_ready()); say so and draw the frame again" (Code::unavailable; poll `flow_ready()`);
  - "GL marches one flow layer per frame";
  - a flow volume past the device's 3D texture size;
  - a stale flow generation;
  - a malformed field layer, flow layer or polyline, in its validator's words;
  - in a sensor mode: flow or field content ("… draws the geometry as data, so it takes no flow or field layers, lines, points or surface colouring"), a bad depth range, a label past the scene's names, or the MSAA refusal above.
- `unhonoured(options)` names any option `draw()` won't honour (today only "mode", for raymarch). Ask it before offering a mode.

## What the host must not do

- **Don't assume GL state survives `draw()`.** It changes the program, VAO, viewport, depth test, depth function, depth mask, depth clamp, cull, front face, polygon mode, blend, point size, texture units 0–4 and SSBO bindings 0–8. Set what you need before drawing your own geometry.
- **State `draw()` and `upload_scene()` set on entry,** even when they then refuse. Whatever the host left (an immediate-mode UI often leaves a scissor), they turn off the scissor test, stencil test, rasterizer discard and `GL_FRAMEBUFFER_SRGB`, turn the colour mask fully on, set `glDepthRange(0, 1)` and unbind `GL_PIXEL_UNPACK_BUFFER`. A host that relies on any of these must set it again after the call.
- **Leave `glClipControl` at its default.** It is the one piece of state `draw()` still assumes.
- **Don't touch the renderer's GL objects,** and don't call it from a context that doesn't share them.
- **Don't draw one scene object with two renderers** expecting a shared upload: each renderer uploads its own copy.
