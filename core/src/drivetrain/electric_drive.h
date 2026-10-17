#pragma once
#include "components.h"
#include "energy_store.h"
#include <cmath>
#include <stdexcept>
namespace ps::drivetrain {
class Battery final : public IEnergyStore {
public:
    explicit Battery(BatteryDesc d) : desc(d), joules_(d.capacity_kwh*3600000*d.initial_soc) {
        if(!std::isfinite(d.capacity_kwh)||d.capacity_kwh<=0||!std::isfinite(d.initial_soc)||d.initial_soc<0||d.initial_soc>1||
           !std::isfinite(d.max_discharge_w)||d.max_discharge_w<=0||!std::isfinite(d.max_charge_w)||d.max_charge_w<0||
           !std::isfinite(d.mass_kg)||d.mass_kg<0) throw std::invalid_argument("invalid battery capacity, state of charge, power or mass");
    }
    real content() const override {return joules_;}
    real capacity() const override {return desc.capacity_kwh*3600000;}
    real soc() const {return joules_/capacity();}
    void withdraw(real j) override {joules_=std::max(real(0),joules_-std::max(real(0),j));}
    real set_content(real j) override {const real old=joules_;joules_=std::clamp(j,real(0),capacity());return joules_-old;}
    void begin_substep(real h) {discharge_left_=std::min(joules_,desc.max_discharge_w*h);charge_left_=std::min(capacity()-joules_,desc.max_charge_w*h);}
    real reserve_discharge(real j) {const real n=std::min(discharge_left_,std::max(real(0),j));discharge_left_-=n;return n;}
    real reserve_charge(real j) {const real n=std::min(charge_left_,std::max(real(0),j));charge_left_-=n;return n;}
    std::uint64_t fold_state_hash(std::uint64_t h) const override {
        for(real v:{capacity(),joules_,desc.max_discharge_w,desc.max_charge_w}) {
            const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
        }return h;
    }
    BatteryDesc desc;
private:
    real joules_=0,discharge_left_=0,charge_left_=0;
};
class ElectricMotor final : public IComponent {
public:
    ElectricMotor(NodeId shaft, ElectricMotorDesc d, Battery& battery, int throttle=-1,int regen=-1,int direction=-1,int ready_slot=-1)
        :shaft_(shaft),desc_(std::move(d)),battery_(battery),throttle_slot_(throttle),regen_slot_(regen),direction_slot_(direction),ready_slot_(ready_slot) {
        for(real v:{desc_.peak_torque_nm,desc_.max_power_w,desc_.max_rpm,desc_.inertia_kgm2,desc_.efficiency})
            if(!std::isfinite(v)||v<=0)throw std::invalid_argument("electric motor parameters must be finite and positive");
        if(desc_.efficiency>1||!std::isfinite(desc_.regen_torque_nm)||desc_.regen_torque_nm<0||
           !std::isfinite(desc_.mass_kg)||desc_.mass_kg<0)throw std::invalid_argument("invalid electric motor efficiency, regeneration or mass");
    }
    void pre_solve(const ComponentSubstepCtx& ctx,const ShaftNetwork& net,Assembly& assembly) override {
        torque_nm=power_w=battery_power_w=regen_d_=0;
        const auto channel=[&](int slot,real fallback){return ctx.channels&&slot>=0&&slot<ctx.channel_count?ctx.channels[slot]:fallback;};
        const real omega=net.omega(shaft_);
        rpm=std::abs(omega)*60/(2*3.14159265358979323846);
        ready=channel(ready_slot_,real(1))>.5;
        if(ctx.h<=0||!ready)return;
        const real request=channel(direction_slot_,ctx.electric_direction);
        // N/unspecified is no propulsion; a gearbox-equipped hybrid can use its positive gear request too.
        const real direction=request==kGearRequestNone||request==0?0:(request<0?-1:1);
        const real pedal=std::clamp(channel(throttle_slot_,ctx.throttle),real(0),real(1));
        const real regen=std::clamp(channel(regen_slot_,ctx.brake),real(0),real(1));
        const real max_omega=desc_.max_rpm*(2*3.14159265358979323846)/60;
        if(pedal>.02&&regen<=.05&&direction!=0&&rpm<desc_.max_rpm) {
            real demand=desc_.peak_torque_nm*std::min(pedal,std::clamp(ctx.traction_throttle_ceiling,real(0),real(1)));
            // Conservative forward-power prediction using rotor inertia; shaft dynamics are solved below.
            const real projected=std::abs(omega)+ctx.h*demand/desc_.inertia_kgm2;
            demand=std::min(demand,desc_.max_power_w/std::max(real(1),projected));
            const real wanted=ctx.h*demand*std::max(real(1),projected)/desc_.efficiency;
            const real reserved=battery_.reserve_discharge(wanted);
            if(wanted>0)demand*=reserved/wanted;
            // Strict residual-energy bound near empty, including a back-driven hybrid shaft.
            demand=std::min(demand,battery_.content()*desc_.efficiency/(ctx.h*std::max(real(1),max_omega)));
            torque_nm=direction*demand;
        } else if(direction!=0&&regen>0&&std::abs(omega)>1) {
            const real demand=desc_.regen_torque_nm*regen;
            const real wanted=ctx.h*demand*std::abs(omega)*desc_.efficiency;
            const real reserved=battery_.reserve_charge(wanted);
            regen_d_=demand*(wanted>0?reserved/wanted:0)/std::abs(omega);
        }
        assembly.torque(shaft_,torque_nm,regen_d_);
    }
    void post_solve(const ComponentSubstepCtx& ctx,const ShaftNetwork& net,const Assembly&,PowertrainEnergy& ledger) override {
        const real omega=net.omega(shaft_);
        torque_nm-=regen_d_*omega;
        rpm=std::abs(omega)*60/(2*3.14159265358979323846);
        power_w=torque_nm*omega;
        const real work=ctx.h*power_w;
        const real electrical=work>=0?work/desc_.efficiency:work*desc_.efficiency;
        battery_power_w=ctx.h>0?electrical/ctx.h:0;
        battery_.set_content(battery_.content()-electrical);
        ledger.electric_out+=work;
        ledger.battery_draw+=electrical;
        ledger.electric_loss+=electrical-work;
    }
    std::uint64_t fold_state_hash(std::uint64_t h) const override {
        for(real v:{desc_.peak_torque_nm,desc_.max_power_w,desc_.max_rpm,desc_.efficiency,desc_.regen_torque_nm}){
            const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
        }
        for(const auto* name:{&desc_.battery,&desc_.ready_channel,&desc_.throttle_channel,&desc_.regen_channel,&desc_.direction_channel}){
            for(unsigned char c:*name){h^=c;h*=1099511628211ULL;}h^=0;h*=1099511628211ULL;
        }
        return h;
    }
    bool ready=true;
    real torque_nm=0,power_w=0,battery_power_w=0,rpm=0;
private:
    real regen_d_=0;
    NodeId shaft_; ElectricMotorDesc desc_; Battery& battery_;
    int throttle_slot_=-1,regen_slot_=-1,direction_slot_=-1,ready_slot_=-1;
};
}
