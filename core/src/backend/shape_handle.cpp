// ps/backend/shape_handle.h implementation: the backend-neutral part of a
// PreparedShape (process-wide diagnostic counters only).

#include "ps/backend/shape_handle.h"

#include <atomic>

namespace ps {
namespace {
std::atomic<std::uint64_t> g_live_prepared{0};
std::atomic<std::uint64_t> g_backend_frees{0};
} // namespace

PreparedShape::PreparedShape(ShapeKind kind) : kind_(kind) { g_live_prepared.fetch_add(1, std::memory_order_relaxed); }

PreparedShape::~PreparedShape() { g_live_prepared.fetch_sub(1, std::memory_order_relaxed); }

std::uint64_t debug_live_prepared_shape_count() { return g_live_prepared.load(std::memory_order_relaxed); }

std::uint64_t debug_prepared_shape_backend_frees() { return g_backend_frees.load(std::memory_order_relaxed); }

void debug_note_prepared_shape_backend_free() { g_backend_frees.fetch_add(1, std::memory_order_relaxed); }

} // namespace ps
