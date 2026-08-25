#pragma once

// ---------------------------------------------------------------------------
// RenderScene -- forward-declared ONLY. Its definition and scene_from_world()
// (the WorldDesc -> RenderScene builder) are Task R1's, not this task's; this
// header exists now so a name is reservable in spade::render before that
// definition lands, and so this task's CMake target has a scene.hpp entry to
// install alongside target.hpp from day one.
// ---------------------------------------------------------------------------

namespace spade::render {

struct RenderScene;

}  // namespace spade::render
