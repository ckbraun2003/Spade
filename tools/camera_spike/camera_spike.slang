// camera_spike.slang -- the camera sensor's core spike kernel (design doc section 8).
// One thread a pixel, one dispatch z-slice an env (so a warp never straddles two envs). It traces the scene with ray queries (procedural box
// candidates with an in-shader sphere trace: gates are a CSG, distractors plain boxes; hardware triangles for the mesh; an analytic ground),
// shades with one Lambert + ambient + sky model, optionally textures the gates, distractors and ground from a procedural pool with the
// spike's own filter (tex.slang), and writes NCHW uint8 and a uint16 instance id.
// Variants (compile-time): L (one lattice TLAS) and E (an array of TLAS, one an env).

#ifndef MODE_E
#define NUM_TLAS 1
#endif

struct Params
{
    uint W;
    uint H;
    uint aa;        // aa x aa rays a pixel (uniform batches)
    uint blur;      // shutter samples (uniform batches)
    uint envs;
    uint side;      // the lattice is side x side cells (mode L)
    uint mode;      // 0 lattice, 1 one TLAS an env
    uint grey;      // 0 colour (3 planes), 1 grey (1 plane)
    float cell;     // lattice cell, metres
    float tmax;
    uint flags;     // 1 debug depth, 2 textured, 4 per-env sample counts, 8 count texture samples
    uint blurStride;
    float pixAngle; // radians a pixel at the image centre
    uint pad0;
    uint pad1;
    uint pad2;
};
[[vk::push_constant]] ConstantBuffer<Params> gP;

[[vk::binding(0, 0)]] RaytracingAccelerationStructure gTlas[NUM_TLAS];
[[vk::binding(1, 0)]] StructuredBuffer<float4> gTable1;     // unit ray directions in the camera frame, W x H
[[vk::binding(2, 0)]] StructuredBuffer<float4> gPoses;      // per (env, blur sample): position, then quaternion (x, y, z, w)
[[vk::binding(3, 0)]] RWStructuredBuffer<uint8_t> gImage;   // NCHW uint8
[[vk::binding(4, 0)]] RWStructuredBuffer<uint16_t> gIds;    // N x H x W
[[vk::binding(5, 0)]] StructuredBuffer<float4> gVerts;
[[vk::binding(6, 0)]] StructuredBuffer<uint> gIdx;
[[vk::binding(7, 0)]] StructuredBuffer<float> gSrgb;        // 1024 entries: linear [0,1] -> sRGB [0,1]
[[vk::binding(8, 0)]] RWStructuredBuffer<float> gDepth;     // debug, written when flags & 1
[[vk::binding(9, 0)]] StructuredBuffer<float4> gTable2;     // the same on a (2W) x (2H) grid, for AA 2x2
[[vk::binding(12, 0)]] StructuredBuffer<uint4> gEnvTex;     // per env, two entries: (gate tex, ground tex, distractor tex, 0), (ground offset u, v as float bits, 0, 0)
[[vk::binding(13, 0)]] StructuredBuffer<uint2> gEnvCfg;     // per env (aa, blur), read when flags & 4
[[vk::binding(14, 0)]] StructuredBuffer<float> gSrgbLin;    // 256 entries: sRGB byte -> linear
[[vk::binding(15, 0)]] RWStructuredBuffer<uint> gStats;     // [0] texture samples

#define TEX_B_TEX 10
#define TEX_B_TAB 11
#include "tex.slang"

static const float3 kOuter = float3(1.0, 1.0, 0.15);
static const float3 kInner = float3(0.65, 0.65, 0.25);
static const float3 kBox = float3(0.5, 0.35, 0.04);
static const float3 kPole = float3(0.05, 1.5, 0.05);
static const float kEps = 1e-4;   // kRaymarchSurfaceEpsilon
static const float3 kSun = float3(0.4, 0.8, 0.6);
static const float kAmbient = 0.25;
static const float kSunI = 0.9;

float sdBox(float3 p, float3 h)
{
    float3 q = abs(p) - h;
    return length(max(q, float3(0.0))) + min(max(q.x, max(q.y, q.z)), 0.0);
}

// kind: 0 gate (outer box minus inner box), 2 box distractor, 3 pole distractor
float primSdf(uint kind, float3 p)
{
    if (kind == 0u)
        return max(sdBox(p, kOuter), -sdBox(p, kInner));
    if (kind == 2u)
        return sdBox(p, kBox);
    return sdBox(p, kPole);
}

float3 primHalf(uint kind)
{
    if (kind == 0u)
        return kOuter;
    if (kind == 2u)
        return kBox;
    return kPole;
}

float3 primNormal(uint kind, float3 p)
{
    const float e = 1e-3;
    return normalize(float3(primSdf(kind, p + float3(e, 0, 0)) - primSdf(kind, p - float3(e, 0, 0)),
                            primSdf(kind, p + float3(0, e, 0)) - primSdf(kind, p - float3(0, e, 0)),
                            primSdf(kind, p + float3(0, 0, e)) - primSdf(kind, p - float3(0, 0, e))));
}

float3 rotate(float4 q, float3 v)
{
    float3 u = q.xyz;
    float3 t = 2.0 * cross(u, v);
    return v + q.w * t + cross(u, t);
}

struct Shaded
{
    float3 colour;
    uint label;
    float depth;
};

float3 sky(float3 d)
{
    float h = saturate(d.y * 0.5 + 0.5);
    return lerp(float3(0.75, 0.85, 0.95), float3(0.30, 0.50, 0.90), h);
}

// A texture sample as linear colour. uv in repeat units; foot is the pixel's footprint in mip-0 texels.
float3 texColour(uint idx, float2 uv, float foot)
{
    uint3 b = sampleTexBytes(idx, uv.x, uv.y, foot);
    return float3(gSrgbLin[b.x], gSrgbLin[b.y], gSrgbLin[b.z]);
}

// aaEff: rays a pixel along one axis (for the footprint)
Shaded shadeRay(float3 ro, float3 rd, uint env, float3 offset, uint aaEff)
{
    Shaded s;
    s.label = 0;
    s.depth = gP.tmax;
    s.colour = sky(rd);
    bool textured = (gP.flags & 2u) != 0u;
    float pixAng = gP.pixAngle / float(aaEff);

    RayDesc ray;
    ray.Origin = ro;
    ray.Direction = rd;
    ray.TMin = 1e-4;
    ray.TMax = gP.tmax;
    RayQuery<RAY_FLAG_NONE> q;
#ifdef MODE_E
    q.TraceRayInline(gTlas[env], RAY_FLAG_NONE, 0xFF, ray);
#else
    q.TraceRayInline(gTlas[0], RAY_FLAG_NONE, 0xFF, ray);
#endif
    while (q.Proceed())
    {
        if (q.CandidateType() == CANDIDATE_PROCEDURAL_PRIMITIVE)
        {
            uint kind = (q.CandidateInstanceID() >> 20u) & 15u;
            float3 oo = q.CandidateObjectRayOrigin();
            float3 od = q.CandidateObjectRayDirection();
            // the ray against the candidate's box (the primitive's bound, padded a little)
            float3 h = primHalf(kind) + float3(0.002);
            float3 inv = 1.0 / od;
            float3 ta = (-h - oo) * inv;
            float3 tb = (h - oo) * inv;
            float3 tlo = min(ta, tb);
            float3 thi = max(ta, tb);
            float t0 = max(max(tlo.x, max(tlo.y, tlo.z)), q.RayTMin());
            float t1 = min(thi.x, min(thi.y, thi.z));
            float tcur = (q.CommittedStatus() == COMMITTED_NOTHING) ? gP.tmax : q.CommittedRayT();
            t1 = min(t1, tcur);
            if (t0 < t1)
            {
                float t = t0;
                for (uint i = 0; i < 96; ++i)
                {
                    float d = primSdf(kind, oo + od * t);
                    if (d < kEps)
                    {
                        q.CommitProceduralPrimitiveHit(t);
                        break;
                    }
                    t += d;
                    if (t > t1)
                        break;
                }
            }
        }
    }

    float tHit = gP.tmax;
    if (q.CommittedStatus() != COMMITTED_NOTHING)
    {
        tHit = q.CommittedRayT();
        uint custom = q.CommittedInstanceID();
        uint kind = (custom >> 20u) & 15u;
        uint var = (custom >> 16u) & 15u;
        s.label = custom & 0xFFFFu;
        float3x4 w2o = q.CommittedWorldToObject3x4();
        float3 nObj;
        float3 albedo;
        float3 pObj = float3(0.0);
        if (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
        {
            uint prim = q.CommittedPrimitiveIndex();
            float3 v0 = gVerts[gIdx[prim * 3 + 0]].xyz;
            float3 v1 = gVerts[gIdx[prim * 3 + 1]].xyz;
            float3 v2 = gVerts[gIdx[prim * 3 + 2]].xyz;
            nObj = normalize(cross(v1 - v0, v2 - v0));
            albedo = float3(0.30, 0.60, 0.90);
        }
        else
        {
            float3 pw = ro + rd * tHit;
            pObj = mul(w2o, float4(pw, 1.0));
            nObj = primNormal(kind, pObj);
            if (kind == 0u)
                albedo = float3(0.90, 0.40, 0.10);
            else if (kind == 2u)
                albedo = (var == 0u) ? float3(0.90, 0.55, 0.15) : float3(0.50, 0.50, 0.50);
            else
                albedo = float3(0.55, 0.55, 0.55);
            if (textured)
            {
                // triplanar in object space on the dominant axis of the object-space normal: fixed to the surface for the whole path
                uint4 et = gEnvTex[env * 2u];
                uint tex = (kind == 0u) ? et.x : et.z;
                float scale = (kind == 0u) ? 0.5 : 1.0;   // repeats a metre
                float3 an = abs(nObj);
                float2 uv;
                if (an.z >= an.x && an.z >= an.y)
                    uv = pObj.xy;
                else if (an.x >= an.y)
                    uv = pObj.zy;
                else
                    uv = pObj.xz;
                float cosv = max(abs(dot(normalize(mul(nObj, (float3x3)w2o)), rd)), 0.1);
                float foot = pixAng * tHit / cosv * scale * 512.0;
                albedo = texColour(tex, uv * scale, foot);
                if ((gP.flags & 8u) != 0u)
                    InterlockedAdd(gStats[0], 1u);
            }
        }
        float3 n = normalize(mul(nObj, (float3x3)w2o));   // rigid: the inverse-transpose is the rotation
        if (dot(n, rd) > 0.0)
            n = -n;
        float lam = max(dot(n, normalize(kSun)), 0.0);
        s.colour = albedo * (kAmbient + kSunI * lam);
        s.depth = tHit;
    }

    // the analytic ground, y = 0 (the lattice offset has no y)
    if (rd.y < -1e-6)
    {
        float tg = -ro.y / rd.y;
        if (tg > 1e-4 && tg < tHit)
        {
            float3 pl = ro + rd * tg - offset;
            float3 albedo;
            if (textured)
            {
                uint4 et = gEnvTex[env * 2u];
                uint4 eo = gEnvTex[env * 2u + 1u];
                float2 off = float2(asfloat(eo.x), asfloat(eo.y));
                float foot = pixAng * tg / max(abs(rd.y), 0.1) * 0.25 * 512.0;   // planar x, z at 0.25 repeats a metre
                albedo = texColour(et.y, pl.xz * 0.25 + off, foot);
                if ((gP.flags & 8u) != 0u)
                    InterlockedAdd(gStats[0], 1u);
            }
            else
            {
                int cx = (int)floor(pl.x * 0.5);
                int cz = (int)floor(pl.z * 0.5);
                float chk = (((cx + cz) & 1) == 0) ? 0.45 : 0.35;
                albedo = float3(chk, chk, chk * 0.9);
            }
            s.colour = albedo * (kAmbient + kSunI * normalize(kSun).y);
            s.label = 0;
            s.depth = tg;
        }
    }
    return s;
}

uint8_t encode(float lin)
{
    float v = saturate(lin) * 1023.0;
    uint i = min((uint)v, 1022u);
    float f = v - (float)i;
    float e = lerp(gSrgb[i], gSrgb[i + 1], f);
    return (uint8_t)(uint)(e * 255.0 + 0.5);
}

[shader("compute")]
[numthreads(8, 8, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    uint x = id.x, y = id.y, env = id.z;
    if (x >= gP.W || y >= gP.H || env >= gP.envs)
        return;
    float3 offset = float3(0.0);
    if (gP.mode == 0)
        offset = float3((float)(env % gP.side) * gP.cell, 0.0, (float)(env / gP.side) * gP.cell);

    uint aa = gP.aa;
    uint blur = gP.blur;
    if ((gP.flags & 4u) != 0u)
    {
        uint2 c = gEnvCfg[env];
        aa = c.x;
        blur = c.y;
    }

    float3 acc = float3(0.0);
    uint labelMid = 0;
    float depthMid = gP.tmax;
    uint midK = blur / 2;
    uint tw = gP.W * aa;
    for (uint k = 0; k < blur; ++k)
    {
        uint pi = (env * gP.blurStride + k) * 2;
        float3 pos = gPoses[pi].xyz + offset;
        float4 quat = gPoses[pi + 1];
        for (uint sy = 0; sy < aa; ++sy)
        {
            for (uint sx = 0; sx < aa; ++sx)
            {
                float3 dc;
                if (aa == 1u)
                    dc = gTable1[y * gP.W + x].xyz;
                else
                    dc = gTable2[(y * 2u + sy) * tw + x * 2u + sx].xyz;
                float3 rd = normalize(rotate(quat, dc));
                Shaded s = shadeRay(pos, rd, env, offset, aa);
                acc += s.colour;
                if (k == midK && sx == 0 && sy == 0)
                {
                    labelMid = s.label;
                    depthMid = s.depth;
                }
            }
        }
    }
    acc /= (float)(blur * aa * aa);

    uint plane = gP.W * gP.H;
    if (gP.grey != 0)
    {
        float lum = 0.2126 * acc.x + 0.7152 * acc.y + 0.0722 * acc.z;
        gImage[env * plane + y * gP.W + x] = encode(lum);
    }
    else
    {
        uint b = env * 3 * plane + y * gP.W + x;
        gImage[b] = encode(acc.x);
        gImage[b + plane] = encode(acc.y);
        gImage[b + 2 * plane] = encode(acc.z);
    }
    gIds[env * plane + y * gP.W + x] = (uint16_t)labelMid;
    if ((gP.flags & 1u) != 0u)
        gDepth[env * plane + y * gP.W + x] = depthMid;
}
