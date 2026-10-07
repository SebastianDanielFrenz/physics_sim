#include "ps/aero/coefficient_map.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ps::aero {
namespace {
void require(bool b,const char* s) { if(!b) throw std::invalid_argument(s); }
bool finite(Vec3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
}
std::shared_ptr<const CoefficientMap> CoefficientMap::create(CoefficientData d) {
    return std::shared_ptr<const CoefficientMap>(new CoefficientMap(std::move(d)));
}
CoefficientMap::CoefficientMap(CoefficientData d):data_(std::move(d)) {
    require(std::isfinite(data_.area_m2)&&data_.area_m2>0&&finite(data_.lengths_m)&&data_.lengths_m.x>0&&data_.lengths_m.y>0&&data_.lengths_m.z>0&&finite(data_.reference_point_local),"invalid coefficient reference dimensions");
    require(data_.geometry_sha256.size()==64&&std::all_of(data_.geometry_sha256.begin(),data_.geometry_sha256.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');})&&!data_.solver.empty()&&!data_.source.empty(),"coefficient provenance requires lowercase geometry SHA256, solver and source");
    std::size_t rows=1;
    for(std::size_t a=0;a<5;++a) {
        const auto& axis=data_.axes[a];
        require(a>=3||!axis.empty(),"missing mandatory coefficient axis");
        require(axis.size()<=4096,"coefficient axis too large");
        for(std::size_t i=0;i<axis.size();++i) {
            require(std::isfinite(axis[i])&&(i==0||axis[i]>axis[i-1]),"coefficient axes must be finite and strictly increasing");
            require((a!=2&&a!=3)||axis[i]>=0,"negative speed/height axis");
        }
        const auto n=std::max(std::size_t(1),axis.size());
        require(rows<=1000000/n,"coefficient table exceeds one million rows"); rows*=n;
    }
    require(data_.axes[0].size()>=3&&data_.axes[0].front()==-180&&data_.axes[0].back()==180,"yaw must cover [-180,180] including seam and interior");
    require(data_.axes[1].size()>=2&&data_.axes[1].front()==-90&&data_.axes[1].back()==90,"pitch must cover [-90,90]");
    require(data_.rows.size()==rows,"coefficient dense row count mismatch");
    for(const auto& row:data_.rows) for(real v:row) require(std::isfinite(v),"nonfinite coefficient");
    const auto yaw_stride=rows/data_.axes[0].size();
    for(std::size_t i=0;i<yaw_stride;++i) require(data_.rows[i]==data_.rows[(data_.axes[0].size()-1)*yaw_stride+i],"yaw seam coefficients must match exactly");
    // All yaw coordinates describe the same vector at either pitch pole.
    const auto pitch_stride=yaw_stride/data_.axes[1].size();
    for(std::size_t y=1;y<data_.axes[0].size();++y) for(auto p:{std::size_t(0),data_.axes[1].size()-1})
        for(std::size_t i=0;i<pitch_stride;++i) require(data_.rows[p*pitch_stride+i]==data_.rows[y*yaw_stride+p*pitch_stride+i],"pitch pole coefficients must be yaw independent");
    auto bytes=[&](const void* p,std::size_t n) { auto b=static_cast<const unsigned char*>(p);for(std::size_t i=0;i<n;++i){hash_^=b[i];hash_*=1099511628211ULL;} };
    auto number=[&](real v){if(v==0)v=0;bytes(&v,sizeof(v));};
    for(const auto& axis:data_.axes){auto n=static_cast<std::uint64_t>(axis.size());bytes(&n,sizeof(n));for(real v:axis)number(v);}
    for(const auto& row:data_.rows)for(real v:row)number(v);
    number(data_.area_m2);for(auto v:{data_.lengths_m,data_.reference_point_local}){number(v.x);number(v.y);number(v.z);}
    for(const auto* s:{&data_.geometry_sha256,&data_.solver,&data_.source}) {auto n=static_cast<std::uint64_t>(s->size());bytes(&n,sizeof(n));bytes(s->data(),s->size());}
}
CoefficientSample CoefficientMap::evaluate(const std::array<real,5>& input) const noexcept {
    CoefficientSample out;
    std::array<std::size_t,5> low{},high{},stride{};
    std::array<real,5> f{};
    std::size_t count=1;
    for(int a=4;a>=0;--a) {stride[a]=count;count*=std::max(std::size_t(1),data_.axes[a].size());}
    for(std::size_t a=0;a<5;++a) {
        const auto& axis=data_.axes[a];if(axis.empty())continue;
        real v=input[a];if(!std::isfinite(v)){out.clamped_axes|=1u<<a;v=axis.front();}
        if(a==0){v=std::fmod(v+180,real(360));if(v<0)v+=360;v-=180;}
        if(v<axis.front()||v>axis.back())out.clamped_axes|=1u<<a;
        v=std::clamp(v,axis.front(),axis.back());
        auto upper=std::upper_bound(axis.begin(),axis.end(),v);
        if(upper==axis.end()){low[a]=high[a]=axis.size()-1;continue;}
        if(upper==axis.begin()){continue;}
        high[a]=static_cast<std::size_t>(upper-axis.begin());low[a]=high[a]-1;
        f[a]=(v-axis[low[a]])/(axis[high[a]]-axis[low[a]]);
    }
    for(unsigned corner=0;corner<32;++corner) {
        real weight=1;std::size_t index=0;
        for(std::size_t a=0;a<5;++a) {bool hi=(corner&(1u<<a))!=0;weight*=hi?f[a]:1-f[a];index+=(hi?high[a]:low[a])*stride[a];}
        if(weight==0)continue;
        for(std::size_t c=0;c<6;++c)out.coefficients[c]+=weight*data_.rows[index][c];
    }
    return out;
}
}
