// tex.slang -- the spike's texture filter, shared by the camera kernel and the probe kernel. No hardware sampler: exact integer loads from one
// storage buffer of RGBA8 words, a bilinear blend within a mip and a linear blend between two mips, both with 8-bit fractional weights, and the
// mip from the pixel footprint with log2 read off the float's exponent bits (no log2 or pow). The CPU reference is texpool.hpp's sample_bytes.
// Textures are square, a power of two, repeating. The including file defines TEX_B_TEX and TEX_B_TAB (binding numbers in set 0).

[[vk::binding(TEX_B_TEX, 0)]] StructuredBuffer<uint> gTex;      // every texture's mip chain, mip 0 first
[[vk::binding(TEX_B_TAB, 0)]] StructuredBuffer<uint4> gTexTab;  // per texture: base word, size, mip count, type

uint3 unpackRgb(uint t)
{
    return uint3(t & 255u, (t >> 8u) & 255u, (t >> 16u) & 255u);
}

uint3 bilinearMip(uint base, uint size, uint m, float u, float v)
{
    uint ms = size >> m;
    float x = u * float(ms) - 0.5;
    float y = v * float(ms) - 0.5;
    float fx = floor(x);
    float fy = floor(y);
    int ix = int(fx);
    int iy = int(fy);
    uint wx = min(uint((x - fx) * 256.0 + 0.5), 256u);
    uint wy = min(uint((y - fy) * 256.0 + 0.5), 256u);
    uint off = base + ((size * size) >> (2u * m)) * ((4u * ((1u << (2u * m)) - 1u)) / 3u);
    uint mask = ms - 1u;
    uint3 t00 = unpackRgb(gTex[off + (uint(iy) & mask) * ms + (uint(ix) & mask)]);
    uint3 t10 = unpackRgb(gTex[off + (uint(iy) & mask) * ms + (uint(ix + 1) & mask)]);
    uint3 t01 = unpackRgb(gTex[off + (uint(iy + 1) & mask) * ms + (uint(ix) & mask)]);
    uint3 t11 = unpackRgb(gTex[off + (uint(iy + 1) & mask) * ms + (uint(ix + 1) & mask)]);
    uint3 top = t00 * (256u - wx) + t10 * wx;
    uint3 bot = t01 * (256u - wx) + t11 * wx;
    return (top * (256u - wy) + bot * wy + 32768u) >> 16u;
}

float log2Approx(float f)
{
    uint b = asuint(f);
    int e = int((b >> 23u) & 255u) - 127;
    float m = float(b & 0x7FFFFFu) * (1.0 / 8388608.0);
    return float(e) + m;
}

// u, v in repeat units; foot the pixel's footprint in texels of mip 0.
uint3 sampleTexBytes(uint idx, float u, float v, float foot)
{
    uint4 tab = gTexTab[idx];
    float lod = clamp(log2Approx(max(foot, 1e-6)), 0.0, float(tab.z - 1u));
    uint m0 = uint(lod);
    uint m1 = min(m0 + 1u, tab.z - 1u);
    uint lw = min(uint((lod - float(m0)) * 256.0 + 0.5), 256u);
    uint3 c0 = bilinearMip(tab.x, tab.y, m0, u, v);
    uint3 c1 = bilinearMip(tab.x, tab.y, m1, u, v);
    return (c0 * (256u - lw) + c1 * lw + 128u) >> 8u;
}
