// camera_spike -- the camera sensor's core spike (docs/design/rendering/2026-11-19-camera-sensor-design.md, section 8, V1 to V7).
//
//   camera_spike bench   --config colour|fisheye --envs N --aa A --blur B --structure L|E [--dynamic] [--export]
//                        [--frames F] [--warmup W] [--repeats R] [--label NAME]       V1 to V5: timings
//   camera_spike oracle  --config ... --envs N --structure L|E [--frame F]            V7 (and the lattice's far-cell error, V4)
//   camera_spike det     --config ... [--out FILE]                                    V6, in-process half; FILE holds the output
//   camera_spike cmp     FILE_A FILE_B                                                V6, across two processes
//   camera_spike export                                                               V5: handles, an import back, a semaphore round trip
//
// Every run prints "RESULT key=value ..." lines. Nothing here is the engine: it is a measurement program.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "renderer.hpp"

// The build passes the directory the CMake rule wrote the SPIR-V to; a bare syntax check (the gcc check) has none.
#ifndef CAMERA_SPIKE_SPV_DIR
#define CAMERA_SPIKE_SPV_DIR "."
#endif

#ifdef _WIN32
#include <windows.h>
#endif

using namespace cs;

namespace {

struct Args {
    std::map<std::string, std::string> kv;
    std::vector<std::string> pos;
    bool has(const std::string& k) const { return kv.count(k) != 0; }
    std::string get(const std::string& k, const std::string& dflt) const {
        const auto it = kv.find(k);
        return it == kv.end() ? dflt : it->second;
    }
    uint32_t num(const std::string& k, uint32_t dflt) const { return has(k) ? static_cast<uint32_t>(std::stoul(kv.at(k))) : dflt; }
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 2; i < argc; ++i) {
        std::string s = argv[i];
        if (s.rfind("--", 0) == 0) {
            s = s.substr(2);
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                a.kv[s] = argv[++i];
            } else {
                a.kv[s] = "1";
            }
        } else {
            a.pos.push_back(s);
        }
    }
    return a;
}

std::string spv_dir(const Args& a) { return a.get("spv-dir", CAMERA_SPIKE_SPV_DIR); }

Setup setup_from(const Args& a) {
    Setup s;
    s.cfg = config_by_name(a.get("config", "colour"));
    s.envs = a.num("envs", 128);
    s.aa = a.num("aa", 1);
    s.blur = a.num("blur", 1);
    s.lattice = a.get("structure", "L") == "L";
    s.dynamic = a.has("dynamic");
    s.exportable = a.has("export");
    s.spv_dir = spv_dir(a);
    return s;
}

void banner(const Device& d) {
    std::printf("RESULT device name=\"%s\" driver=%u ts_period_ns=%.3f scratch_align=%llu budget_ext=%d external=%d luid_valid=%d max_descriptor_set_as=%llu "
                "max_tlas_instances=%llu max_blas_primitives=%llu\n",
                d.name.c_str(), d.driver, static_cast<double>(d.ts_period_ns), static_cast<unsigned long long>(d.scratch_align), d.has_budget ? 1 : 0,
                d.has_external ? 1 : 0, d.luid_valid ? 1 : 0, static_cast<unsigned long long>(d.max_descriptor_as), static_cast<unsigned long long>(d.max_instances),
                static_cast<unsigned long long>(d.max_primitives));
}

double mb(uint64_t bytes) { return static_cast<double>(bytes) / (1024.0 * 1024.0); }

bool open_device(Device& d, bool external) {
    std::string why;
    if (!create_device(d, external, why)) {
        std::printf("RESULT refused reason=\"%s\"\n", why.c_str());
        std::fprintf(stderr, "camera_spike: refused: %s\n", why.c_str());
        return false;
    }
    return true;
}

int run_bench(const Args& a) {
    Device d;
    if (!open_device(d, a.has("export"))) return 3;
    banner(d);
    const Setup s = setup_from(a);
    if (s.exportable && !d.has_external) {
        std::printf("RESULT refused reason=\"external memory is not offered\"\n");
        return 3;
    }
    const uint32_t frames = a.num("frames", 60), warm = a.num("warmup", 10), repeats = a.num("repeats", 3);
    const std::string label = a.get("label", "bench");
    Renderer r(d, s);
    std::printf("RESULT setup label=%s cfg=%s envs=%u aa=%u blur=%u structure=%s dynamic=%d export=%d setup_ms=%.1f tlas_build_gpu_ms=%.3f build_wall_ms=%.1f "
                "structures_mb=%.2f scratch_mb=%.2f out_mb=%.2f vram_delta_mb=%.1f instances=%llu\n",
                label.c_str(), s.cfg.name, s.envs, s.aa, s.blur, s.lattice ? "L" : "E", s.dynamic ? 1 : 0, s.exportable ? 1 : 0, r.setup_ms(), r.build_gpu_ms(), r.build_ms(),
                mb(r.structure_bytes()), mb(r.scratch_bytes()), mb(r.out_bytes()), mb(r.vram_used()), static_cast<unsigned long long>(r.instances()));
    uint32_t f = 0;
    for (uint32_t i = 0; i < warm; ++i) (void)r.render(f++, false);
    const double rays = static_cast<double>(s.envs) * s.cfg.W * s.cfg.H * s.aa * s.aa * s.blur;
    for (uint32_t rep = 0; rep < repeats; ++rep) {
        FrameTimes sum;
        std::vector<double> walls;
        for (uint32_t i = 0; i < frames; ++i) {
            const FrameTimes t = r.render(f++, false);
            sum.wall_ms += t.wall_ms; sum.kernel_ms += t.kernel_ms; sum.upload_ms += t.upload_ms;
            sum.as_ms += t.as_ms; sum.pose_cpu_ms += t.pose_cpu_ms; sum.inst_cpu_ms += t.inst_cpu_ms;
            walls.push_back(t.wall_ms);
        }
        const double n = frames;
        const double wall = sum.wall_ms / n, kernel = sum.kernel_ms / n;
        std::sort(walls.begin(), walls.end());
        std::printf("RESULT bench label=%s cfg=%s envs=%u aa=%u blur=%u structure=%s dynamic=%d export=%d rep=%u frames=%u wall_ms=%.3f wall_p50_ms=%.3f wall_max_ms=%.3f "
                    "kernel_ms=%.3f upload_gpu_ms=%.4f as_update_gpu_ms=%.4f pose_cpu_ms=%.4f inst_cpu_ms=%.4f renders_per_s_wall=%.0f renders_per_s_kernel=%.0f "
                    "grays_per_s=%.3f\n",
                    label.c_str(), s.cfg.name, s.envs, s.aa, s.blur, s.lattice ? "L" : "E", s.dynamic ? 1 : 0, s.exportable ? 1 : 0, rep, frames, wall, walls[walls.size() / 2],
                    walls.back(), kernel, sum.upload_ms / n, sum.as_ms / n, sum.pose_cpu_ms / n, sum.inst_cpu_ms / n, s.envs / wall * 1e3, s.envs / kernel * 1e3,
                    rays / (kernel * 1e-3) * 1e-9);
        std::fflush(stdout);
    }
    return 0;
}

struct Stat {
    double max = 0, sum = 0;
    uint64_t n = 0;
    std::vector<double> v;
    void add(double x) {
        max = std::max(max, x);
        sum += x;
        ++n;
        v.push_back(x);
    }
    double mean() const { return n ? sum / static_cast<double>(n) : 0.0; }
    double pct(double p) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())))];
    }
};

// V7 (and V4's far-cell error): the GPU frame against the fp64 oracle, for env 0 and the last env.
int run_oracle(const Args& a) {
    Device d;
    if (!open_device(d, false)) return 3;
    banner(d);
    Setup s = setup_from(a);
    s.aa = 1;
    s.blur = 1;
    s.depth = true;
    const uint32_t frame = a.num("frame", 3);
    Renderer r(d, s);
    (void)r.render(frame, true);
    const auto table = make_table(s.cfg, 1);
    const auto srgb = make_srgb_table();
    const uint32_t W = s.cfg.W, H = s.cfg.H, planes = s.cfg.grey ? 1u : 3u;
    const V3 sun = normalize({0.4, 0.8, 0.6});
    for (const uint32_t env : {0u, s.envs - 1}) {
        const EnvScene sc = make_env(env, frame, false);
        const Pose pose = camera_pose(env, shutter_time(frame, 0, 1));
        const Mat3 R = quat_to_mat(pose.quat);
        const V3 o{pose.pos[0], pose.pos[1], pose.pos[2]};
        std::vector<OHit> hit(static_cast<size_t>(W) * H);
        std::vector<V3> dirs(hit.size());
        auto work = [&](uint32_t y0, uint32_t y1) {
            for (uint32_t y = y0; y < y1; ++y)
                for (uint32_t x = 0; x < W; ++x) {
                    const size_t i = static_cast<size_t>(y) * W + x;
                    const V3 dc{table[i * 4], table[i * 4 + 1], table[i * 4 + 2]};
                    const V3 dw = normalize(mul(R, dc));
                    dirs[i] = dw;
                    hit[i] = oracle_trace(sc, r.mesh(), o, dw);
                }
        };
        std::vector<std::thread> th;
        const uint32_t nt = std::max(1u, std::thread::hardware_concurrency());
        for (uint32_t t = 0; t < nt; ++t) th.emplace_back(work, H * t / nt, H * (t + 1) / nt);
        for (auto& t : th) t.join();

        const size_t base = static_cast<size_t>(env) * W * H;
        uint64_t label_bad = 0, edge_bad = 0, interior_bad = 0, edge_px = 0, ground_px = 0, ground_ok = 0, colour_n = 0, colour_exact = 0;
        Stat depth_err, colour_err;
        const double g_lin_hi = 0.45 * (0.25 + 0.9 * sun.y), g_lin_lo = 0.35 * (0.25 + 0.9 * sun.y);
        const int pal_hi = encode_srgb(srgb, g_lin_hi), pal_lo = encode_srgb(srgb, g_lin_lo);
        for (uint32_t y = 0; y < H; ++y)
            for (uint32_t x = 0; x < W; ++x) {
                const size_t i = static_cast<size_t>(y) * W + x;
                bool edge = false;
                for (int dy = -1; dy <= 1 && !edge; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int yy = static_cast<int>(y) + dy, xx = static_cast<int>(x) + dx;
                        if (yy < 0 || xx < 0 || yy >= static_cast<int>(H) || xx >= static_cast<int>(W)) continue;
                        const OHit& n = hit[static_cast<size_t>(yy) * W + xx];
                        if (n.label != hit[i].label || std::fabs(n.t - hit[i].t) > 0.25) {
                            edge = true;
                            break;
                        }
                    }
                if (edge) ++edge_px;
                const uint16_t gl = r.ids()[base + i];
                if (gl != hit[i].label) {
                    ++label_bad;
                    if (edge) ++edge_bad; else ++interior_bad;
                    continue;
                }
                if (edge) continue;
                const double gt = r.depth()[base + i];
                depth_err.add(std::fabs(gt - hit[i].t));
                // colour: per channel level difference
                int worst = 0;
                if (!s.cfg.grey) {
                    const double lin[3] = {hit[i].colour.x, hit[i].colour.y, hit[i].colour.z};
                    for (uint32_t c = 0; c < 3; ++c) {
                        const int exp = encode_srgb(srgb, lin[c]);
                        const int got = r.image()[(static_cast<size_t>(env) * 3 + c) * W * H + i];
                        worst = std::max(worst, std::abs(exp - got));
                    }
                } else {
                    const double lum = 0.2126 * hit[i].colour.x + 0.7152 * hit[i].colour.y + 0.0722 * hit[i].colour.z;
                    worst = std::abs(static_cast<int>(encode_srgb(srgb, lum)) - static_cast<int>(r.image()[static_cast<size_t>(env) * W * H + i]));
                }
                colour_err.add(worst);
                ++colour_n;
                if (worst == 0) ++colour_exact;
                if (hit[i].label == 0 && hit[i].t < kTmax && dirs[i].y < 0) {
                    ++ground_px;
                    const int got = s.cfg.grey ? r.image()[static_cast<size_t>(env) * W * H + i] : r.image()[(static_cast<size_t>(env) * 3) * W * H + i];
                    if (!s.cfg.grey && (std::abs(got - pal_hi) <= 1 || std::abs(got - pal_lo) <= 1)) ++ground_ok;
                    if (s.cfg.grey) ++ground_ok;  // the grey plane has no single known channel; counted only for the colour config
                }
            }
        std::printf("RESULT oracle cfg=%s structure=%s env=%u (%s cell) pixels=%u edge_pixels=%llu label_mismatch=%llu on_edge=%llu interior=%llu "
                    "depth_err_max=%.6f depth_err_mean=%.6f depth_err_p999=%.6f colour_level_max=%.0f colour_level_mean=%.4f colour_exact_pct=%.2f "
                    "ground_pixels=%llu ground_palette_ok=%llu\n",
                    s.cfg.name, s.lattice ? "L" : "E", env, env == 0 ? "near" : "far", W * H, static_cast<unsigned long long>(edge_px),
                    static_cast<unsigned long long>(label_bad), static_cast<unsigned long long>(edge_bad), static_cast<unsigned long long>(interior_bad), depth_err.max,
                    depth_err.mean(), depth_err.pct(0.999), colour_err.max, colour_err.mean(), colour_n ? 100.0 * static_cast<double>(colour_exact) / static_cast<double>(colour_n) : 0.0,
                    static_cast<unsigned long long>(ground_px), static_cast<unsigned long long>(s.cfg.grey ? 0 : ground_ok));
        (void)planes;
        std::fflush(stdout);
    }
    return 0;
}

bool same(const Renderer& r, const std::vector<uint8_t>& img, const std::vector<uint16_t>& ids) { return r.image() == img && r.ids() == ids; }

// V6: the same scene and tick twice in a process, and again after tearing everything down and rebuilding it (the stand-in for reset or restore).
int run_det(const Args& a) {
    Device d;
    if (!open_device(d, false)) return 3;
    banner(d);
    const Setup s = setup_from(a);
    const uint32_t frame = a.num("frame", 7);
    std::vector<uint8_t> img;
    std::vector<uint16_t> ids;
    bool again = false, rebuilt = false;
    {
        Renderer r(d, s);
        (void)r.render(frame, true);
        img = r.image();
        ids = r.ids();
        (void)r.render(frame + 5, false);  // something else in between
        (void)r.render(frame, true);
        again = same(r, img, ids);
    }
    {
        Renderer r(d, s);
        (void)r.render(frame, true);
        rebuilt = same(r, img, ids);
    }
    if (a.has("out")) {
        std::ofstream f(a.get("out", ""), std::ios::binary);
        if (!f) die("cannot write " + a.get("out", ""));
        f.write(reinterpret_cast<const char*>(img.data()), static_cast<std::streamsize>(img.size()));
        f.write(reinterpret_cast<const char*>(ids.data()), static_cast<std::streamsize>(ids.size() * 2));
    }
    std::printf("RESULT det cfg=%s structure=%s envs=%u frame=%u same_process_repeat=%s after_rebuild=%s bytes=%zu\n", s.cfg.name, s.lattice ? "L" : "E", s.envs, frame,
                again ? "equal" : "DIFFERENT", rebuilt ? "equal" : "DIFFERENT", img.size() + ids.size() * 2);
    return (again && rebuilt) ? 0 : 1;
}

int run_cmp(const Args& a) {
    if (a.pos.size() != 2) die("cmp needs two files");
    std::vector<char> x, y;
    for (int k = 0; k < 2; ++k) {
        std::ifstream f(a.pos[k], std::ios::binary);
        if (!f) die("cannot read " + a.pos[k]);
        auto& v = k == 0 ? x : y;
        v.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    }
    size_t first = x.size();
    for (size_t i = 0; i < std::min(x.size(), y.size()); ++i)
        if (x[i] != y[i]) {
            first = i;
            break;
        }
    const bool eq = x.size() == y.size() && first == x.size();
    std::printf("RESULT cmp a=%zu bytes b=%zu bytes across_processes=%s first_difference=%zu\n", x.size(), y.size(), eq ? "equal" : "DIFFERENT", first);
    return eq ? 0 : 1;
}

#ifdef _WIN32
int run_export(const Args& a) {
    Device d;
    if (!open_device(d, true)) return 3;
    banner(d);
    if (!d.has_external) {
        std::printf("RESULT refused reason=\"win32 external memory or semaphore is not offered\"\n");
        return 3;
    }
    // What the device says about the handle types, before building anything.
    VkPhysicalDeviceExternalBufferInfo bi{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_BUFFER_INFO};
    bi.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    bi.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
    VkExternalBufferProperties bp{VK_STRUCTURE_TYPE_EXTERNAL_BUFFER_PROPERTIES};
    vkGetPhysicalDeviceExternalBufferProperties(d.phys, &bi, &bp);
    std::printf("RESULT export buffer_opaque_win32 features=0x%x (1 dedicated-only, 2 exportable, 4 importable) compatible=0x%x\n",
                bp.externalMemoryProperties.externalMemoryFeatures, bp.externalMemoryProperties.compatibleHandleTypes);
    VkSemaphoreTypeCreateInfo tci{VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO};
    tci.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
    VkPhysicalDeviceExternalSemaphoreInfo si{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_SEMAPHORE_INFO};
    si.pNext = &tci;
    si.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT;
    VkExternalSemaphoreProperties sp{VK_STRUCTURE_TYPE_EXTERNAL_SEMAPHORE_PROPERTIES};
    vkGetPhysicalDeviceExternalSemaphoreProperties(d.phys, &si, &sp);
    std::printf("RESULT export timeline_semaphore_opaque_win32 features=0x%x (1 exportable, 2 importable) compatible=0x%x\n", sp.externalSemaphoreFeatures,
                sp.compatibleHandleTypes);
    VkSemaphoreTypeCreateInfo bin{VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO};
    bin.semaphoreType = VK_SEMAPHORE_TYPE_BINARY;
    si.pNext = &bin;
    VkExternalSemaphoreProperties sb{VK_STRUCTURE_TYPE_EXTERNAL_SEMAPHORE_PROPERTIES};
    vkGetPhysicalDeviceExternalSemaphoreProperties(d.phys, &si, &sb);
    std::printf("RESULT export binary_semaphore_opaque_win32 features=0x%x compatible=0x%x\n", sb.externalSemaphoreFeatures, sb.compatibleHandleTypes);
    std::printf("RESULT export device_uuid=");
    for (int i = 0; i < VK_UUID_SIZE; ++i) std::printf("%02x", d.device_uuid[i]);
    std::printf(" luid=");
    for (int i = 0; i < VK_LUID_SIZE; ++i) std::printf("%02x", d.device_luid[i]);
    std::printf(" luid_valid=%d\n", d.luid_valid ? 1 : 0);

    Setup s = setup_from(a);
    s.exportable = true;
    s.envs = a.num("envs", 128);
    Renderer r(d, s);
    (void)r.render(5, true);
    const std::vector<uint8_t> img = r.image();
    const std::vector<uint16_t> ids = r.ids();

    // The memory handle: export it, import it into a second buffer (what a CUDA importer does), read it back and compare.
    VkMemoryGetWin32HandleInfoKHR gi{VK_STRUCTURE_TYPE_MEMORY_GET_WIN32_HANDLE_INFO_KHR};
    // The Buffer struct does not keep the memory handle of the renderer's out buffer; take it from the buffer's own memory through a
    // second exportable allocation instead: export from the renderer's buffer by its VkDeviceMemory.
    // (Renderer::out_buffer() is the exported one.)
    gi.memory = r.out_buffer().mem;
    gi.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
    HANDLE mem_handle = nullptr;
    const auto t0 = std::chrono::steady_clock::now();
    CS_VK(vkGetMemoryWin32HandleKHR(d.dev, &gi, &mem_handle));
    const double get_ms = ms_since(t0);

    VkExternalMemoryBufferCreateInfo ext{VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO};
    ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
    VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bci.pNext = &ext;
    bci.size = r.out_buffer().size;
    bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    VkBuffer imported = VK_NULL_HANDLE;
    CS_VK(vkCreateBuffer(d.dev, &bci, nullptr, &imported));
    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(d.dev, imported, &req);
    VkImportMemoryWin32HandleInfoKHR imp{VK_STRUCTURE_TYPE_IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR};
    imp.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
    imp.handle = mem_handle;
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.pNext = &imp;
    mai.allocationSize = req.size;
    mai.memoryTypeIndex = find_memory(d, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VkDeviceMemory imported_mem = VK_NULL_HANDLE;
    const VkResult imr = vkAllocateMemory(d.dev, &mai, nullptr, &imported_mem);
    bool mem_ok = false;
    if (imr == VK_SUCCESS) {
        CS_VK(vkBindBufferMemory(d.dev, imported, imported_mem, 0));
        Buffer host = make_buffer(d, bci.size, VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        VkCommandBuffer cb = alloc_cmd(d);
        begin_cmd(cb);
        VkBufferCopy bc{0, 0, bci.size};
        vkCmdCopyBuffer(cb, imported, host.buf, 1, &bc);
        submit_wait(d, cb);
        const auto* p = static_cast<const uint8_t*>(host.map);
        mem_ok = std::equal(img.begin(), img.end(), p);
        std::vector<uint16_t> ids2(ids.size());
        std::memcpy(ids2.data(), p + ((r.img_bytes() + 255) & ~uint64_t{255}), ids2.size() * 2);
        mem_ok = mem_ok && ids2 == ids;
        destroy_buffer(d, host);
        vkDestroyBuffer(d.dev, imported, nullptr);
        vkFreeMemory(d.dev, imported_mem, nullptr);
    }
    std::printf("RESULT export memory_handle get_ms=%.3f import_result=%d imported_bytes_equal=%s\n", get_ms, static_cast<int>(imr), mem_ok ? "equal" : "DIFFERENT");

    // The timeline semaphore: export its handle, import it into a second semaphore, wait on the second for the value the first signalled
    // (the renderer has signalled value r.signalled()).
    VkSemaphoreGetWin32HandleInfoKHR sgi{VK_STRUCTURE_TYPE_SEMAPHORE_GET_WIN32_HANDLE_INFO_KHR};
    sgi.semaphore = r.semaphore();
    sgi.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT;
    HANDLE sem_handle = nullptr;
    const VkResult gr = vkGetSemaphoreWin32HandleKHR(d.dev, &sgi, &sem_handle);
    bool sem_ok = false;
    double rt_us = 0;
    if (gr == VK_SUCCESS) {
        VkSemaphoreTypeCreateInfo t2{VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO};
        t2.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
        VkSemaphoreCreateInfo sc{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        sc.pNext = &t2;
        VkSemaphore second = VK_NULL_HANDLE;
        CS_VK(vkCreateSemaphore(d.dev, &sc, nullptr, &second));
        VkImportSemaphoreWin32HandleInfoKHR isi{VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_WIN32_HANDLE_INFO_KHR};
        isi.semaphore = second;
        isi.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT;
        isi.handle = sem_handle;
        const VkResult ir = vkImportSemaphoreWin32HandleKHR(d.dev, &isi);
        if (ir == VK_SUCCESS) {
            uint64_t value = 0;
            CS_VK(vkGetSemaphoreCounterValue(d.dev, second, &value));
            // a round trip: the host signals the imported one forward, the original sees it
            const uint64_t next = r.signalled() + 1;
            VkSemaphoreSignalInfo ssi{VK_STRUCTURE_TYPE_SEMAPHORE_SIGNAL_INFO};
            ssi.semaphore = second;
            ssi.value = next;
            const auto tr = std::chrono::steady_clock::now();
            CS_VK(vkSignalSemaphore(d.dev, &ssi));
            uint64_t seen = 0;
            CS_VK(vkGetSemaphoreCounterValue(d.dev, r.semaphore(), &seen));
            rt_us = ms_since(tr) * 1e3;
            sem_ok = value >= r.signalled() && seen >= next;
            std::printf("RESULT export semaphore_import import_result=0 imported_value=%llu signalled=%llu original_after_signal=%llu\n", static_cast<unsigned long long>(value),
                        static_cast<unsigned long long>(r.signalled()), static_cast<unsigned long long>(seen));
        } else {
            std::printf("RESULT export semaphore_import import_result=%d\n", static_cast<int>(ir));
        }
        vkDestroySemaphore(d.dev, second, nullptr);
    }
    std::printf("RESULT export semaphore_handle get_result=%d round_trip=%s signal_to_visible_us=%.1f\n", static_cast<int>(gr), sem_ok ? "ok" : "FAILED", rt_us);
    if (mem_handle != nullptr) CloseHandle(mem_handle);
    return (mem_ok && sem_ok) ? 0 : 1;
}
#else
int run_export(const Args&) {
    std::printf("RESULT refused reason=\"the export test is win32 only\"\n");
    return 3;
}
#endif

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: camera_spike bench|oracle|det|cmp|export [options]\n");
        return 2;
    }
    const std::string mode = argv[1];
    const Args a = parse(argc, argv);
    if (mode == "bench") return run_bench(a);
    if (mode == "oracle") return run_oracle(a);
    if (mode == "det") return run_det(a);
    if (mode == "cmp") return run_cmp(a);
    if (mode == "export") return run_export(a);
    std::fprintf(stderr, "camera_spike: unknown mode %s\n", mode.c_str());
    return 2;
}
