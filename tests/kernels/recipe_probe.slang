// ---------------------------------------------------------------------------
// recipe_probe.slang (module API stage 7, Tasks B1 and B2) -- a TEST kernel that does nothing. A test module's own
// recipe (compute/recipe.hpp) dispatches it so the recorder's behaviour is observable without any physics: its pipeline is
// built when the schedule names the recipe, it is recorded where the schedule puts the pass and in the recordings the
// cadence puts it in, and it is timed under its pass's name. No bindings and no push block: the engine's pipeline layout is
// a superset of what it uses.
// ---------------------------------------------------------------------------

[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void recipe_probe(uint3 tid : SV_DispatchThreadID) {
}
