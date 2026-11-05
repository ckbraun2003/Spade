#include "physics/airflow/worker_pool.hpp"

#include <algorithm>

#include "core/detail/flush_subnormals.hpp"  // x86-64 only; _mm_getcsr, _mm_setcsr, SPADE_NOINLINE

namespace spade::physics::airflow {

namespace {

// One thread's share of a run: items t, t + T, t + 2T, ... in that order,
// under the caller's MXCSR word, its own restored after (the wrapper rule: no
// float op here, each item one opaque call).
SPADE_NOINLINE void run_share(uint32_t t, uint32_t threads, uint32_t items, void (*item)(const void*, uint32_t),
                              const void* context, unsigned int csr) noexcept {
    const unsigned int saved = _mm_getcsr();
    _mm_setcsr(csr);
    for (uint32_t i = t; i < items; i += threads) item(context, i);
    _mm_setcsr(saved);
}

}  // namespace

uint32_t default_fluid_threads() noexcept {
    const unsigned int hw = std::thread::hardware_concurrency();
    return std::clamp<uint32_t>(hw == 0u ? 1u : static_cast<uint32_t>(hw), 1u, 8u);
}

WorkerPool::WorkerPool(uint32_t threads, ItemOrder order)
    : threads_(std::max<uint32_t>(threads, 1u)), order_(order) {
    if (order_ == ItemOrder::reversed) return;
    workers_.reserve(threads_ - 1u);
    for (uint32_t t = 1; t < threads_; ++t) workers_.emplace_back([this, t] { work(t); });
}

WorkerPool::~WorkerPool() {
    {
        const std::lock_guard<std::mutex> lock(m_);
        stop_ = true;
    }
    start_.notify_all();
    for (std::thread& w : workers_) w.join();
}

void WorkerPool::run_erased(uint32_t items, Item item, const void* context) {
    if (items == 0u) return;
    const unsigned int csr = _mm_getcsr();
    if (order_ == ItemOrder::reversed) {
        for (uint32_t i = items; i-- > 0u;) run_share(i, items, i + 1u, item, context, csr);
        return;
    }
    if (threads_ == 1u || items == 1u) {
        // Today's serial path: every item on the caller, in order, under its own word.
        run_share(0u, 1u, items, item, context, csr);
        return;
    }
    const std::lock_guard<std::mutex> one(run_lock_);
    {
        const std::lock_guard<std::mutex> lock(m_);
        items_ = items;
        item_ = item;
        context_ = context;
        csr_ = csr;
        remaining_ = threads_ - 1u;
        ++generation_;
    }
    start_.notify_all();
    run_share(0u, threads_, items, item, context, csr);
    std::unique_lock<std::mutex> lock(m_);
    done_.wait(lock, [this] { return remaining_ == 0u; });
}

void WorkerPool::work(uint32_t thread) {
    uint64_t seen = 0;
    for (;;) {
        uint32_t items = 0;
        Item item = nullptr;
        const void* context = nullptr;
        unsigned int csr = 0;
        {
            std::unique_lock<std::mutex> lock(m_);
            start_.wait(lock, [&] { return stop_ || generation_ != seen; });
            if (stop_) return;
            seen = generation_;
            items = items_;
            item = item_;
            context = context_;
            csr = csr_;
        }
        run_share(thread, threads_, items, item, context, csr);
        {
            const std::lock_guard<std::mutex> lock(m_);
            --remaining_;
        }
        done_.notify_one();
    }
}

}  // namespace spade::physics::airflow
