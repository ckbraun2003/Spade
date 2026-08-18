// ---------------------------------------------------------------------------
// mirrors.slang (S6 Task 8) -- THE SHARED MIRRORS OF CPU EXPRESSIONS. Every
// glm vector/quaternion helper the ported kernels need, plus the one physics
// PREDICATE both contact kernels must read identically, in ONE place.
//
// A MODULE, NOT A KERNEL: no entry point, no binding, no buffer. It reaches
// SPIR-V only through the kernels that import it, exactly like fp32_math.slang
// and sdf_eval.slang -- so its arithmetic is scanned by the float-controls gate
// on the real compiled artifact rather than on a separately-compiled copy.
//
// ===========================================================================
// WHY IT EXISTS (S6 Task 7 review, minor M3 -- coordinator-ruled into Task 8)
//
// Through Task 7 these helpers were HAND-DUPLICATED across kernels --
// glm_min/glm_max/glm_dot3/glm_length3 in both contact kernels, glm_cross/
// glm_length3/glm_quat_conjugate/glm_quat_rotate in both force/integrate
// kernels, and effective_proxy_radius in both contact kernels -- under a
// comment claiming the copies "must not be able to read the same law two
// different ways". That comment stated an intention, not a mechanism: nothing
// stopped an edit to one copy from missing the other, and the review found the
// claim FALSE on its own terms.
//
// Task 8 is the wave that makes the argument decisive rather than aspirational,
// because it adds THREE more kernels that need the same helpers (medium_update,
// rotors, sensor_imu). Five copies of a left-associated dot product is five
// chances to associate one of them right. One definition, imported everywhere,
// is the mechanism the old comment wished for.
//
// ===========================================================================
// GLM IS PART OF THE PARITY CONTRACT, NOT AN IMPLEMENTATION DETAIL -- the point
// integrate.slang's header makes and this file inherits. The CPU twin expresses
// every vector and quaternion operation through glm, each glm helper has ONE
// pinned expression, and fp32 addition and multiplication are not associative.
// So a port must reproduce the EXPRESSION, not the mathematical function.
// Writing Slang's own `dot()`, `mul()` or `normalize()` instead would be a
// different summation order -- and OpDot is forbidden outright anyway
// (engine/testing/spirv_scan.hpp rule P2: SPIR-V does not specify a dot
// product's accumulation order, and slangc does not decorate it NoContraction).
//
// Each helper below names the vendored glm source it reproduces (glm 1.0.1).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// A. SCALAR SELECTORS -- detail/func_common.inl.
//
// TERNARIES, NOT Slang's min/max, and the difference is SIGNED ZERO: glm's
// max(-0.0f, +0.0f) evaluates `(-0.0f < +0.0f)` -> false -> returns -0.0f,
// while GLSL.std.450 FMax is free to return either. Same instruction count,
// question removed.
// ---------------------------------------------------------------------------
float glm_min(float x, float y) { return (y < x) ? y : x; }
float glm_max(float x, float y) { return (x < y) ? y : x; }
float glm_clamp(float x, float lo, float hi) { return glm_min(glm_max(x, lo), hi); }

// ---------------------------------------------------------------------------
// B. VECTOR PRODUCTS AND NORMS -- detail/func_geometric.inl.
//
// compute_dot<vec3> is `tmp.x + tmp.y + tmp.z`, which C++ associates LEFT --
// hence the explicit parentheses, which are the contract and not decoration.
// glm::length(v) is sqrt(dot(v, v)) at both widths.
//
// NO glm_dot2 HERE (S6 Task 8 review, minor M1). This module carried one in its
// first draft and no kernel ever called it: every vec2 dot product this engine
// performs is inside glm_length2 below, already spelled out. A helper with no
// caller is a helper nothing tests, and the point of this module is to abolish
// second sources of truth, not to be a complete glm surface.
// ---------------------------------------------------------------------------
float glm_dot3(float3 a, float3 b) { return (a.x * b.x + a.y * b.y) + a.z * b.z; }

float glm_length2(float2 v) { return sqrt(v.x * v.x + v.y * v.y); }
float glm_length3(float3 v) { return sqrt((v.x * v.x + v.y * v.y) + v.z * v.z); }

// detail/func_geometric.inl, compute_cross.
float3 glm_cross(float3 x, float3 y) {
    return float3(x.y * y.z - y.y * x.z,
                  x.z * y.x - y.z * x.x,
                  x.x * y.y - y.x * x.y);
}

// ---------------------------------------------------------------------------
// C. QUATERNIONS -- detail/type_quat.inl and ext/quaternion_geometric.inl.
//
// STORAGE IS (x, y, z, w). glm 1.0.1's quat MEMORY order is x,y,z,w even though
// its CONSTRUCTOR takes w,x,y,z (shaders/shared/layouts.slang's BodyState note;
// test_state.cpp's StateLayout.GlmQuatMemoryOrderIsXyzw pins it at runtime,
// since no static_assert can see into GLM's anonymous union).
// ---------------------------------------------------------------------------

// glm::conjugate(qua) == wxyz(q.w, -q.x, -q.y, -q.z).
float4 glm_quat_conjugate(float4 q) { return float4(-q.x, -q.y, -q.z, q.w); }

// operator*(qua, vec3):
//     uv  = cross(q.xyz, v);  uuv = cross(q.xyz, uv);
//     result = v + ((uv * q.w) + uuv) * 2
float3 glm_quat_rotate(float4 q, float3 v) {
    const float3 quat_vector = float3(q.x, q.y, q.z);
    const float3 uv = glm_cross(quat_vector, v);
    const float3 uuv = glm_cross(quat_vector, uv);
    return v + ((uv * q.w) + uuv) * 2.0f;
}

// operator*=(qua) -- the expression operator*(qua, qua) delegates to. Every
// line associates LEFT, which is what the extra parentheses spell out.
float4 glm_quat_mul(float4 p, float4 q) {
    const float w = ((p.w * q.w - p.x * q.x) - p.y * q.y) - p.z * q.z;
    const float x = ((p.w * q.x + p.x * q.w) + p.y * q.z) - p.z * q.y;
    const float y = ((p.w * q.y + p.y * q.w) + p.z * q.x) - p.x * q.z;
    const float z = ((p.w * q.z + p.z * q.w) + p.x * q.y) - p.y * q.x;
    return float4(x, y, z, w);
}

// normalize(qua):
//     len = sqrt(dot(q, q));  if (len <= 0) return wxyz(1,0,0,0);
//     oneOverLen = 1 / len;   return q * oneOverLen
// dot(qua) folds the vec4 (w, x, y, z) as `(tmp.x + tmp.y) + (tmp.z + tmp.w)`
// -- i.e. (w*w + x*x) + (y*y + z*z), a PAIRWISE TREE, not a left fold.
//
// The degenerate guard is `len <= 0.0f`, exactly as glm spells it and NOT as
// `!(len > 0.0f)`: the two differ on a NaN length (glm falls through to the
// divide; the negated form would return the identity), and a mirror that
// "improves" on its reference is no longer a mirror.
float4 glm_quat_normalize(float4 q) {
    const float len = sqrt((q.w * q.w + q.x * q.x) + (q.y * q.y + q.z * q.z));
    if (len <= 0.0f) {
        return float4(0.0f, 0.0f, 0.0f, 1.0f);  // wxyz(1,0,0,0)
    }
    const float one_over_len = 1.0f / len;
    return float4(q.x * one_over_len, q.y * one_over_len, q.z * one_over_len, q.w * one_over_len);
}

// ---------------------------------------------------------------------------
// D. NOT A GLM EXPRESSION -- the one PHYSICS PREDICATE that has to be shared.
//
// physics::effective_proxy_radius() (physics/contacts.hpp) is the ONE canonical
// rule for a body's contact size (coordinator ruling, S6 Task 2 merge carry
// item 3), mirrored character for character:
//
//     body_radius != 0.0f ? body_radius : world_default
//
// `!=`, not `>`: 0 is a SENTINEL meaning "defer to this world's
// ContactParams::proxy_radius", and a negative radius is not validated against
// here any more than it is on the C++ side (authoring-time validation --
// vehicles/model_type.hpp's ModelType -- is what keeps one from reaching either
// side of this seam).
//
// It lives in this module, beside the glm mirrors rather than in a header of
// its own, for the reason this whole module exists: collision_static.slang and
// collision_dynamic.slang must not be able to read a body's size two different
// ways, and one definition is the only thing that makes that true.
// ---------------------------------------------------------------------------
float effective_proxy_radius(float body_radius, float world_default) {
    return body_radius != 0.0f ? body_radius : world_default;
}
