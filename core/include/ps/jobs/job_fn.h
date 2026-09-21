// ps/jobs/job_fn.h — allocation-free callable used for job bodies.
//
// PLAN.md D8: no allocation inside jobs, including submitting them. A
// std::function may heap-allocate for captures beyond its small-object
// optimisation (size/triviality is unspecified per implementation); JobFn
// instead requires the callable to be trivially copyable and fit inline, so
// submission itself never touches the heap.

#pragma once

#include <cstddef>
#include <type_traits>
#include <utility>

namespace ps::jobs {

class JobFn {
public:
    static constexpr std::size_t kInlineBytes = 48;

    JobFn() = default;

    template <typename Callable, typename = std::enable_if_t<!std::is_same_v<std::decay_t<Callable>, JobFn>>>
    JobFn(Callable c) { // NOLINT(google-explicit-constructor) - implicit lambda->JobFn conversion is the point
        using C = std::decay_t<Callable>;
        static_assert(sizeof(C) <= kInlineBytes, "job callable too large for inline storage (ps::jobs::JobFn)");
        static_assert(std::is_trivially_copyable_v<C>, "job callable must be trivially copyable - captures must be by value/pointer, no heap-owning captures");
        new (static_cast<void*>(storage_)) C(std::move(c));
        invoke_ = [](void* self) { (*static_cast<C*>(self))(); };
    }

    void operator()() const {
        if (invoke_) invoke_(const_cast<std::byte*>(storage_));
    }

    [[nodiscard]] explicit operator bool() const { return invoke_ != nullptr; }

private:
    alignas(16) std::byte storage_[kInlineBytes]{};
    void (*invoke_)(void*) = nullptr;
};

} // namespace ps::jobs
