// texprobe.slang -- the texture filter on its own, for the CPU-against-GPU check on a dense (u, v) grid.
// One thread a sample: input (u, v, footprint, texture index as bits), output the filtered RGB packed in a word.

#define TEX_B_TEX 0
#define TEX_B_TAB 1
#include "tex.slang"

struct ProbeParams
{
    uint count;
    uint p0;
    uint p1;
    uint p2;
};
[[vk::push_constant]] ConstantBuffer<ProbeParams> gPP;
[[vk::binding(2, 0)]] StructuredBuffer<float4> gIn;
[[vk::binding(3, 0)]] RWStructuredBuffer<uint> gOut;

[shader("compute")]
[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= gPP.count)
        return;
    float4 s = gIn[id.x];
    uint3 c = sampleTexBytes(asuint(s.w), s.x, s.y, s.z);
    gOut[id.x] = c.x | (c.y << 8u) | (c.z << 16u);
}
