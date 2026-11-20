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
        if (name.empty()) return std::unexpected(Error{Code::invalid_argument, "a VRAM share needs a name"});
        if (bytes == 0) {
            return std::unexpected(
                Error{Code::invalid_argument, "VRAM share '" + std::string(name) + "' asks for 0 bytes"});
        }
        for (const Share& s : shares_) {
            if (s.name == name) {
                return std::unexpected(Error{Code::invalid_argument,
                                             "VRAM share '" + std::string(name) + "' is already held (" +
                                                 std::to_string(s.bytes) + " bytes); release it before reserving again"});
            }
        }
        const uint64_t held = reserved();
        const uint64_t limit = budget.budget > reserve_bytes ? budget.budget - reserve_bytes : 0;
        // held + bytes <= limit, written so it cannot overflow.
        if (bytes > limit || held > limit - bytes) {
            const bool ext = budget.source == MemoryBudget::Source::ext;
            return std::unexpected(Error{
                Code::capacity_exceeded,
                "VRAM share '" + std::string(name) + "' asks for " + std::to_string(bytes) +
                    " bytes, over what the device's budget leaves: the shares held are " +
                    (shares_.empty() ? std::string("none") : list_()) + " (" + std::to_string(held) +
                    " bytes in all), so this would make " + std::to_string(held + bytes) +
                    ", over the limit " + std::to_string(limit) + " = budget " + std::to_string(budget.budget) +
                    " - reserve " + std::to_string(reserve_bytes) + ", from " +
                    (ext ? std::string("VK_EXT_memory_budget")
                         : std::string("half the device-local heap (no VK_EXT_memory_budget)")) +
                    ", on device '" + std::string(device) + "'"});
        }
        VramShare share;
        share.ledger_ = this;
        share.id_ = next_id_++;
        share.bytes_ = bytes;
        shares_.push_back(Share{share.id_, std::string(name), bytes});
        return share;
    }

    // The sum of the shares held, in bytes.
    [[nodiscard]] uint64_t reserved() const noexcept {
        uint64_t sum = 0;
        for (const Share& s : shares_) sum += s.bytes;
        return sum;
    }

    // The shares held, in the order they were reserved.
    [[nodiscard]] std::span<const Share> shares() const noexcept { return shares_; }

    // One line: "world set 1073741824, flow solver 4294967296, camera 1073741824 bytes (6442450944 in all)".
    [[nodiscard]] std::string describe() const {
        return (shares_.empty() ? std::string("no shares") : list_()) + " bytes (" + std::to_string(reserved()) +
               " in all)";
    }

private:
    friend class VramShare;
    void release_(uint64_t id) noexcept {
        for (auto it = shares_.begin(); it != shares_.end(); ++it) {
            if (it->id == id) {
                shares_.erase(it);
                return;
            }
        }
    }
    [[nodiscard]] std::string list_() const {
        std::string out;
        for (const Share& s : shares_) {
            if (!out.empty()) out += ", ";
            out += s.name + " " + std::to_string(s.bytes);
        }
        return out;
    }
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
