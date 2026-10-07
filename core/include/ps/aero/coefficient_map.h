#pragma once
#include "ps/math/pose.h"
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cstdint>
namespace ps::aero {
// Immutable validated table. Axes fixed: yaw, pitch, speed, optional height, offset.
struct CoefficientData {
    std::array<std::vector<real>,5> axes;
    std::vector<std::array<real,6>> rows; // last present axis fastest
    real area_m2=1;
    Vec3 lengths_m{1,1,1}, reference_point_local{};
    std::string geometry_sha256, solver, source;
};
struct CoefficientSample { std::array<real,6> coefficients{}; std::uint32_t clamped_axes=0; };
class CoefficientMap {
public:
    static std::shared_ptr<const CoefficientMap> create(CoefficientData);
    CoefficientSample evaluate(const std::array<real,5>&) const noexcept;
    const CoefficientData& data() const { return data_; }
    std::uint64_t content_hash() const { return hash_; }
private:
    explicit CoefficientMap(CoefficientData);
    CoefficientData data_;
    std::uint64_t hash_=14695981039346656037ULL;
};
}
