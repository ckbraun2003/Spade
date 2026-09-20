// ---------------------------------------------------------------------------
// collision_gather.slang -- the CollisionDynamic pass as a JACOBI GATHER, one
// thread per grid ENTRY.
//
// A SEPARATE FILE RATHER THAN TWO MORE ENTRY POINTS IN collision_dynamic.slang,
// and the reason is a guard rather than taste. SlangSpirv.EveryCompiledVariant
// IsScanned compares two workgroup-size variants of one module and requires
// EXACTLY ONE differing word -- the LocalSize x-operand -- because more than
// one means the compiled CODE depends on SPADE_WG, which is precisely the
// lane-dependent construct the A7 invariance sweep forbids and precisely what
// L306 was reverted for. A module with three entry points has three LocalSize
// declarations and trips it at 3.
//
// THE GUARD IS RIGHT AND THE FILE WAS WRONG. Relaxing it to "one per entry
// point" would buy this change at the cost of the estate's only automatic
// detector of lane-dependent codegen. Splitting keeps every module at one
// entry point, keeps the invariant exact, and leaves collision_dynamic.slang
// byte-identical -- provable by `git diff` rather than by argument, which is
// the same choice made for resolve_pair on the CPU side.
//
// Nothing dispatches these yet; the selector is a semantics change and lands
// with the parity re-pin, alone.
// ---------------------------------------------------------------------------

import layouts;
import bindings;
import mirrors;

static const uint kInvalidWorld = 0xFFFFFFFFu;

// ---------------------------------------------------------------------------
// The run comparator, WORLD-AWARE -- grid.cpp:271-276's run_key_less, where
// cell_key_less above is its world-free restriction.
//
// THE DISTINCTION IS LOAD-BEARING AND IT IS WHY THIS FUNCTION EXISTS AT ALL.
// cell_key_less can omit the world because the sweep searches [lo, hi), one
// world's range, and its own header says so: "restricted to the cell because
// the world is already constant over the searched range". The gather does not
// have that range. It searches its SEGMENT, and in the BATCHED shape a segment
// is the whole array and holds every world -- so a world-free comparator would
// search a range the world varies over and find a run belonging to somebody
// else. Wrong physics, silently, and only in the batched shape.
//
// In the segmented shape the world compare is always equal and costs nothing.
// One comparator, both shapes, and neither term is optional.
// ---------------------------------------------------------------------------
bool run_key_less(GridEntryRow e, uint w, int cz, int cy, int cx) {
    if (e.world != w) return e.world < w;
    if (e.cell_z != cz) return e.cell_z < cz;
    if (e.cell_y != cy) return e.cell_y < cy;
    return e.cell_x < cx;
}

// ---------------------------------------------------------------------------
// ONE CONTACT, FROM `a`'S SIDE ONLY, computed entirely from the shadow and
// applied to `a`'s running output. This is resolve_pair with every `b`-side
// write deleted; the op order, the groupings and the `!(...)` guard spellings
// are that function's, verbatim, because they are the parity contract.
//
// THE OUTPUTS ARE RUNNING VALUES, NOT RETURNED DELTAS, and that is arithmetic
// rather than style: the sweep evaluates ((vel - x) + y) while a delta return
// evaluates vel + ((0 - x) + y), and in fp32 those are not the same number.
// Accumulating into the body's own running velocity reproduces the sweep's
// sequence exactly, which is what makes the CPU gather byte-identical to the
// CPU sweep whenever only one pair is in flight.
//
// AND `pos_out` ACCUMULATES. resolve_pair's step 3 writes ABSOLUTELY --
// `bodies[ia].pos = pa - correction*w_a*n`, with `pa` read at the top -- which
// is correct for a function that resolves one pair and returns. Carried into a
// gather unchanged it would apply ONLY THE LAST PARTNER'S correction: every
// body still moves, every one-pair test still passes, and a stack
// de-penetrates against one neighbour out of six.
// ---------------------------------------------------------------------------
void gather_apply_for_a(GatherBodyRow a, GatherBodyRow b, ContactParams pp,
                        inout float3 vel_out, inout float3 pos_out) {
    // -1. PER-PAIR CONTACT DISTANCE. Both radii are ALREADY effective: the 0
    //     sentinel was resolved once per body by the fill kernel, where the
    //     sweep resolves it twice per pair examination.
    const float contact_dist = a.radius + b.radius;
    const float contact_dist2 = contact_dist * contact_dist;

    // 0. NARROW PHASE. Both tests `!(...)` so a NaN coordinate FAILS to be a
    //    contact rather than falling through into a division by a NaN.
    const float3 pa = a.pos;
    const float3 pb = b.pos;
    const float3 d = pb - pa;
    const float dist2 = glm_dot3(d, d);
    if (!(dist2 < contact_dist2)) return;
    if (!(dist2 > 0.0f)) return;

    const float dist = sqrt(dist2);
    const float3 n = float3(d.x / dist, d.y / dist, d.z / dist);
    const float depth = contact_dist - dist;

    const float ma = a.mass;
    const float mb = b.mass;
    const float m_eff = (ma * mb) / (ma + mb);
    const float w_a = m_eff / ma;
    const float w_b = m_eff / mb;

    // 1. NORMAL IMPULSE, only for an APPROACHING pair.
    //
    //    v_rel_n IS SIGN-IDENTICAL FROM BOTH SIDES -- dot(-v_rel, -n) is the
    //    same sum of the same products in the same order -- so this guard
    //    fires, or does not fire, identically in the partner's own gather. If
    //    it did not, one side would impulse where the other did not and
    //    momentum would break at O(1) rather than at O(eps).
    float3 va = a.vel;
    float3 vb = b.vel;
    const float3 v_rel = vb - va;
    const float v_rel_n = glm_dot3(v_rel, n);

    float j_n = 0.0f;
    if (v_rel_n < 0.0f) {
        j_n = -(1.0f + pp.restitution_e) * v_rel_n;
        vel_out = vel_out - (j_n * w_a) * n;
        va = va - (j_n * w_a) * n;
        vb = vb + (j_n * w_b) * n;
    }

    // 2. FRICTION on the post-normal RELATIVE tangential velocity.
    //    va/vb are LOCALS reconstructed from the shadow. Using vel_out here
    //    would fold this body's OTHER partners into this pair's friction --
    //    Gauss-Seidel inside one thread's gather, a solver that is neither --
    //    AND IT WOULD PASS EVERY TWO-BODY TEST, because with one partner
    //    vel_out and va hold the same bits.
    const float3 v_rel_post = vb - va;
    const float v_n_post = glm_dot3(v_rel_post, n);
    const float3 v_t = v_rel_post - v_n_post * n;
    const float v_t_len = glm_length3(v_t);
    if (v_t_len > 0.0f) {
        const float dv_t = glm_min(pp.friction_mu * j_n, v_t_len);
        const float3 t = float3(v_t.x / v_t_len, v_t.y / v_t_len, v_t.z / v_t_len);
        vel_out = vel_out + (dv_t * w_a) * t;
    }

    // 3. POSITIONAL CORRECTION, ACCUMULATING. max on the SUBTRACTION so this
    //    stays branch-free, exactly as resolve_pair and contacts.cpp spell it.
    const float correction = pp.baumgarte_beta * glm_max(depth - pp.slop, 0.0f);
    pos_out = pos_out - (correction * w_a) * n;
}

// ---------------------------------------------------------------------------
// GATHER -- one thread per entry. THE THREAD INDEX IS THE ENTRY, NOT THE
// WORLD, and that single line is what the whole change is for.
// ---------------------------------------------------------------------------
[shader("compute")]
[numthreads(SPADE_WG, 1, 1)]
void collision_dynamic_gather(uint3 tid : SV_DispatchThreadID) {
    const uint i = tid.x;
    if (i >= pass_params.grid_entry_count) { return; }

    const GridEntryRow ea = grid_entries[i];
    if (ea.world == kInvalidWorld) { return; }

    // The world comes free off the entry, so the sweep's two per-world
    // bounding searches are gone. THE SEGMENT BOUND IS NOT GONE -- segments
    // are sorted independently, so the concatenation is not globally ordered
    // -- but it is one division rather than a search, and it is 0 in the
    // batched shape without a branch.
    const uint segment = pass_params.grid_segment;
    const uint seg_begin = (i / segment) * segment;
    const uint seg_end = seg_begin + segment;

    const ContactParams material = contact_params[ea.world];
    const GatherBodyRow self = body_snapshot[i];

    float3 vel_out = self.vel;
    float3 pos_out = self.pos;

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nx = ea.cell_x + dx;
                const int ny = ea.cell_y + dy;
                const int nz = ea.cell_z + dz;

                uint k = seg_begin;
                {
                    uint l = seg_begin;
                    uint r = seg_end;
                    while (l < r) {
                        const uint m = l + (r - l) / 2u;
                        if (run_key_less(grid_entries[m], ea.world, nz, ny, nx)) { l = m + 1u; }
                        else { r = m; }
                    }
                    k = l;
                }

                for (; k < seg_end; ++k) {
                    const GridEntryRow eb = grid_entries[k];
                    // The run ends where the key changes. The world test also
                    // stops the scan at the sentinel tail, whose world sorts
                    // strictly after every real one.
                    if (eb.world != ea.world) break;
                    if (eb.cell_x != nx || eb.cell_y != ny || eb.cell_z != nz) break;

                    // THE SELF-EXCLUSION, AND NOTHING ELSE. `ea.slot <
                    // eb.slot` is GONE rather than relaxed: a gather must see
                    // ALL of a body's partners, not only its higher-slot ones.
                    // Without this line a body resolves against itself, d is
                    // exactly zero, and the `dist2 > 0` narrow-phase guard
                    // swallows it -- a skipped pair with no symptom anywhere.
                    // That guard is a backstop; this line is the mechanism.
                    if (k == i) continue;

                    gather_apply_for_a(self, body_snapshot[k], material, vel_out, pos_out);
                }
            }
        }
    }

    bodies[ea.slot].vel = vel_out;
    bodies[ea.slot].pos = pos_out;
}
