#pragma once

// ---------------------------------------------------------------------------
// vram_ledger.hpp (camera design section 12, item 3) -- ONE PLACE THAT SUMS THE
// VRAM SHARES on a card the world set, the flow solver and the camera share.
//
// Each consumer RESERVES a stated share (a name and a byte count) before it
// allocates; the ledger refuses a reservation that would push the sum of the
// shares held over the device's budget (budget - reserve), by name, and lists who
// holds what. The camera's 1 GiB at 128 envs is a reservation like the others.
//
// WHAT THIS IS NOT. The per-allocation checks (check_vram in device_limits.hpp:
// each allocation against the live heap) stay as they are and are still the
// last word on one allocation: with VK_EXT_memory_budget the heap's usage is the
// whole process's, so a component that allocates sees what the others hold. The
// ledger answers the question those checks cannot: "do the shares we PROMISED,
// all together, fit?", before any of them is allocated, and on a device without
// the extension (half the heap, the usage not visible) where each component's
// own count is blind to the others'. A reservation is a promise, not an
// allocation: it does not move the heap's usage, and a share is the consumer's
// own upper bound on what it will hold.
//
// Pure data (no Vulkan), so it runs on every machine; VulkanContext owns one
// (vram_ledger()). Not thread-safe, like the context. A VramShare must be
// released (or destroyed) before the ledger it came from.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "compute/device_limits.hpp"
#include "core/error.hpp"

namespace spade::compute {

class VramLedger;

// A held reservation: releases its bytes when destroyed or release()d. Move-only.
class VramShare {
public:
    VramShare() = default;
    VramShare(const VramShare&) = delete;
    VramShare& operator=(const VramShare&) = delete;
    VramShare(VramShare&& other) noexcept { *this = std::move(other); }
    VramShare& operator=(VramShare&& other) noexcept;
    ~VramShare() { release(); }

    [[nodiscard]] bool held() const noexcept { return ledger_ != nullptr; }
    [[nodiscard]] uint64_t bytes() const noexcept { return bytes_; }
    void release() noexcept;

private:
    friend class VramLedger;
    VramLedger* ledger_ = nullptr;
    uint64_t id_ = 0;
    uint64_t bytes_ = 0;
};

class VramLedger {
public:
    struct Share {
        uint64_t id = 0;
        std::string name;
        uint64_t bytes = 0;
    };

    // Reserves `bytes` for `name`. Refused with Code::capacity_exceeded, before anything is held, when the shares already
    // held plus `bytes` exceed budget.budget - reserve_bytes (the message names the asker, its bytes, every share held, the
    // sum, the limit, where the budget came from and the device); with Code::invalid_argument for 0 bytes, an empty name
    // or a name already held.
    [[nodiscard]] Result<VramShare> reserve(std::string_view name, uint64_t bytes, const MemoryBudget& budget,
                                            uint64_t reserve_bytes, std::string_view device) {
        (void)name;
        (void)bytes;
        (void)budget;
        (void)reserve_bytes;
        (void)device;
        return VramShare{};  // RED stub: holds nothing, checks nothing
    }

    // The sum of the shares held, in bytes.
    [[nodiscard]] uint64_t reserved() const noexcept { return 0; }

    // The shares held, in the order they were reserved.
    [[nodiscard]] std::span<const Share> shares() const noexcept { return shares_; }

    // One line: "world set 1073741824, flow 4294967296, camera 1073741824 bytes (6442450944 in all)".
    [[nodiscard]] std::string describe() const { return {}; }

private:
    friend class VramShare;
    void release_(uint64_t id) noexcept { (void)id; }
    std::vector<Share> shares_;
    uint64_t next_id_ = 1;
};

inline VramShare& VramShare::operator=(VramShare&& other) noexcept {
    if (this != &other) {
        release();
        ledger_ = other.ledger_;
        id_ = other.id_;
        bytes_ = other.bytes_;
        other.ledger_ = nullptr;
        other.id_ = 0;
        other.bytes_ = 0;
    }
    return *this;
}

inline void VramShare::release() noexcept {
    if (ledger_ != nullptr) ledger_->release_(id_);
    ledger_ = nullptr;
    id_ = 0;
    bytes_ = 0;
}

}  // namespace spade::compute
