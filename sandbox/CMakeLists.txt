# spade_sandbox -- the engine's reference application (SL10-SL13, Plan C task C0).
#
# SL2b: this target may include only headers the engine INSTALLS and may link
# only spade:: targets. That is why the link list below is aliases and nothing
# else -- no spade_render, no relative include into engine/, no vendor target.
# The guard over this include graph gets its first subject here; before C0 it
# was a rule with nothing to police.
#
# ⚠ NO GLFW, NO IMGUI, NO GLAD IN C0. This task is headless-only by design
# (SL15b: headless is the test surface), so the windowing dependencies do not
# appear until the windowed TargetSink arrives in task C2. The flag lands
# before the dependency does, which is the ordering that keeps CI free.

add_executable(spade_sandbox main.cpp)

target_link_libraries(spade_sandbox PRIVATE
    spade::core
    spade::world
    spade::render
)

target_compile_features(spade_sandbox PRIVATE cxx_std_20)

# NOT installed, and that is deliberate: SL2a gives Spade a release identity of
# eleven LIBRARY modules. The sandbox is a consumer of that surface, not part
# of it -- installing it would put a developer tool in a package whose whole
# claim is "a domain-neutral engine an out-of-tree consumer links against".
