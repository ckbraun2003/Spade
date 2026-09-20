// ---------------------------------------------------------------------------
// collision_fill.slang -- the Jacobi gather's start-of-iteration shadow, taken
// once per pass, one thread per grid ENTRY.
//
// ONE ENTRY POINT PER MODULE, AND THAT IS AN INVARIANT RATHER THAN A STYLE.
// SlangSpirv.EveryCompiledVariantIsScanned compares two workgroup-size
// variants of a module and requires EXACTLY ONE differing word -- the LocalSize
// x-operand. Its stated purpose is to forbid compiled code that depends on
// SPADE_WG, which is what L306 was reverted for. But every additional entry
// point adds its own LocalSize declaration, so the guard ALSO enforces one
// entry point per module, silently and for free. It caught this file at 3 and
// again at 2 before it was split out; the invariant held and the guard was not
// touched.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import mirrors;

static const uint kInvalidWorld = 0xFFFFFFFFu;

// ---------------------------------------------------------------------------
// FILL -- one thread per entry, writing the start-of-iteration shadow.
//
// A SEPARATE DISPATCH, NOT A PROLOGUE, and the reason is the whole point of
// the shadow: a gather thread reads every one of its partners, and with
// threads mapped to entries another workgroup may already have written a
// partner. Vulkan has no device-wide barrier inside a dispatch, so the reads
// and the writes have to be separated by a dispatch boundary.
//
// THE COST OF THAT BOUNDARY IS A FENCE AND IT IS NOT ESTIMATED HERE. It is
// O(N) over entries against a 27-cell search per entry, so the argument for
// cheapness is strong -- which is exactly the shape that produces an
// unmeasured fence, and this branch has already lost a leg to one.
// ---------------------------------------------------------------------------
[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void collision_dynamic_fill(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= pass_params.grid_entry_count) { return; }

    const GridEntryRow e = grid_entries[i];
    if (e.world == kInvalidWorld) { return; }  // a padding lane; leave it alone

    const ContactParams material = contact_params[e.world];
    const BodyState body = bodies[e.slot];

    GatherBodyRow row;
    row.pos = body.pos;
    row.mass = body.mass;
    row.vel = body.vel;
    row.radius = effective_proxy_radius(body.proxy_radius, material.proxy_radius);
    body_snapshot[i] = row;
}
