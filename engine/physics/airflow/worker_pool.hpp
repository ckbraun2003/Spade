#pragma once

// ---------------------------------------------------------------------------
// physics/airflow/worker_pool.hpp -- the CPU fluid step's threads (L18, the
// lead's grant 2026-11-03; Rendering builds it, Physics reviews). Scoped to
// physics/airflow: it moves to Core only when a second user appears.
//
// FIXED PARTITIONS, NO REDUCTIONS. run(items, fn) calls fn(item) once for each
// item in [0, items), item t + T q on thread t (T threads, the caller thread
// 0), and returns when every item is done: the barrier. The caller splits the
// work so that each item writes its own samples and reads only what no item of
// the same run writes. The bits then cannot depend on the thread count, the
// item-to-thread map or the timing: every sample takes the same operations, in
// the same order, from the same inputs. A sum over many samples is never split
// across items here (fixed_tile_sum() and the rest stay serial).
//
// THE CALLER'S MXCSR. Each worker runs its items under the MXCSR word the
// caller had when it called run(), so the step's flushed-subnormal scope
// (core/detail/flush_subnormals.hpp) holds on every thread, and restores its
// own word after. The items are one opaque call each (the wrapper rule).
//
// One run() at a time: a pool shared by copies of a scratch serialises them.
// threads() == 1 runs every item on the caller, in item order, with no thread
// made: today's serial path.
//
// ItemOrder::reversed (for tests) makes no thread either: every run's items
// go on the caller from the last to the first. A split whose bits depend on
// the order its items run in gives other bits there every time, where on
// threads it would only race; threads() still reports the count asked for,
// so callers take their split paths.
// ---------------------------------------------------------------------------

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace spade::physics::airflow {

enum class ItemOrder : uint8_t { forward, reversed };

// The fluid step's thread count by default: the hardware's, at most 8, at
// least 1.
[[nodiscard]] uint32_t default_fluid_threads() noexcept;

class WorkerPool {
public:
    // `threads` at least 1 (0 is taken as 1); threads - 1 workers are started,
    // none for ItemOrder::reversed.
    explicit WorkerPool(uint32_t threads, ItemOrder order = ItemOrder::forward);
    ~WorkerPool();
    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;

    [[nodiscard]] uint32_t threads() const noexcept { return threads_; }

    template <class F>
    void run(uint32_t items, F&& fn) {
        using Fn = std::remove_reference_t<F>;
        run_erased(items, &call<Fn>, static_cast<const void*>(&fn));
    }

private:
    using Item = void (*)(const void*, uint32_t);
    template <class Fn>
    static void call(const void* fn, uint32_t item) {
        (*static_cast<Fn*>(const_cast<void*>(fn)))(item);
    }
    void run_erased(uint32_t items, Item item, const void* context);
    void work(uint32_t thread);

    uint32_t threads_ = 1;
    ItemOrder order_ = ItemOrder::forward;
    std::vector<std::thread> workers_;
    std::mutex run_lock_;  // one run() at a time
    std::mutex m_;
    std::condition_variable start_;
    std::condition_variable done_;
    uint64_t generation_ = 0;
    uint32_t remaining_ = 0;
    bool stop_ = false;
    // The current run's, set under m_ before generation_ moves.
    uint32_t items_ = 0;
    Item item_ = nullptr;
    const void* context_ = nullptr;
    unsigned int csr_ = 0;
};

}  // namespace spade::physics::airflow
