// core/src/drivetrain/energy_store.h -- PHYS-033 P4 I7a C6: lumped energy
// stores (INTERNAL, ps::drivetrain). The fuel tank is the first one; an I9
// battery is expected to implement the same IEnergyStore interface with its
// own unit (charge or energy instead of kg).
//
// PHYS-033 owner ruling "Tanks and stores": no detailed simulation of the
// content - a store is a fill state (plus, for a gas/liquefied-gas store, a
// pressure as a simple function of fill, which a liquid fuel tank feeding a
// pumped injection system does not need and does not carry). No sloshing,
// no temperature dynamics. In I7a the fuel mass does not change the chassis
// mass; a later increment adds it as a point mass at the tank position
// (PLAN.md "Fuel mass moves the chassis").
//
// Hot-path contract (PLAN.md D8): withdraw()/set_content() allocate nothing.
// Determinism (PLAN.md D7): content() is simulation state - Powertrain folds
// it into its state hash only when the vehicle HAS a store, so a vehicle
// without one hashes exactly as before C6.

#pragma once

#include "ps/drivetrain/fuel_desc.h"
#include "ps/types.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace ps::drivetrain {

class IEnergyStore {
public:
    virtual ~IEnergyStore() = default;
    // The stored quantity, in the store's own unit (kg for a fuel tank).
    [[nodiscard]] virtual real content() const = 0;
    [[nodiscard]] virtual real capacity() const = 0;
    // Removes `amount` (>= 0), clamping the content at 0 - never negative.
    virtual void withdraw(real amount) = 0;
    // Sets the content, clamped to [0, capacity]; returns the signed change
    // actually applied (new - old), for the caller's external-exchange
    // ledger.
    virtual real set_content(real value) = 0;
    [[nodiscard]] virtual std::uint64_t fold_state_hash(std::uint64_t seed) const = 0;
};

// A liquid fuel tank: its content is fuel mass in kg. `fuel` is fixed at
// construction (no runtime fuel change - a refuel with a different fuel is
// deferred, c5cd plan owner ruling).
class FuelTank final : public IEnergyStore {
public:
    FuelTank(const FuelDesc& fuel, real capacity_kg, real initial_kg)
        : fuel_(fuel), capacity_kg_(capacity_kg), mass_kg_(std::clamp(initial_kg, real(0.0), capacity_kg)) {}

    [[nodiscard]] real content() const override { return mass_kg_; }
    [[nodiscard]] real capacity() const override { return capacity_kg_; }
    // Exactly the booked mass: when the engine books the whole remaining
    // content (the emptying substep), mass_kg_ - mass_kg_ is exactly 0.0.
    void withdraw(real amount) override { mass_kg_ = std::max(real(0.0), mass_kg_ - amount); }
    real set_content(real value) override {
        const real old = mass_kg_;
        mass_kg_ = std::clamp(value, real(0.0), capacity_kg_);
        return mass_kg_ - old;
    }
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const override {
        std::uint64_t h = seed;
        const auto* bytes = reinterpret_cast<const unsigned char*>(&mass_kg_);
        for (std::size_t i = 0; i < sizeof(mass_kg_); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
        return h;
    }
    [[nodiscard]] const FuelDesc& fuel() const { return fuel_; }

private:
    FuelDesc fuel_;
    real capacity_kg_;
    real mass_kg_;
};

} // namespace ps::drivetrain
