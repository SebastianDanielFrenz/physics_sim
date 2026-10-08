#pragma once
#include "ps/drivetrain/table2d.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace ps::drivetrain {
// Error is relative with a 1% peak floor, so near-zero signed pumping terms
// cannot demand infinitely many nodes. All RPM rows participate.
inline real pressure_error(const Table2D& t, const Table2D& approx, real p) {
    real peak=0, error=0;
    for (real v:t.values) peak=std::max(peak,std::abs(v));
    const real floor=std::max(real(1e-12),peak*real(0.01));
    for (real rpm:t.x) {
        const real actual=t.value(rpm,p), predicted=approx.value(rpm,p);
        error=std::max(error,std::abs(actual-predicted)/std::max(floor,std::abs(actual)));
    }
    return error;
}
inline Table2D pressure_subset(const Table2D& t, const std::vector<std::size_t>& nodes) {
    Table2D out;out.x=t.x;
    for(auto j:nodes)out.y.push_back(t.y[j]);
    for(std::size_t i=0;i<t.x.size();++i)for(auto j:nodes)out.values.push_back(t.at(i,j));
    return out;
}
// Greedy worst-error insertion against every computed validation sample.
// Each invocation independently chooses nodes for this table.
inline void reduce_pressure_axis(Table2D& t, real tolerance) {
    if(t.y.size()<3)return;
    std::vector<std::size_t> keep{0,t.y.size()-1};
    for(;;) {
        const auto candidate=pressure_subset(t,keep);
        real worst=tolerance;std::size_t selected=t.y.size();
        for(std::size_t j=1;j+1<t.y.size();++j) {
            if(std::binary_search(keep.begin(),keep.end(),j))continue;
            const real error=pressure_error(t,candidate,t.y[j]);
            if(error>worst){worst=error;selected=j;}
        }
        if(selected==t.y.size()){t=candidate;return;}
        keep.insert(std::lower_bound(keep.begin(),keep.end(),selected),selected);
    }
}
} // namespace ps::drivetrain
