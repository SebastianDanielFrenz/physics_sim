#pragma once
#include "components.h"
#include "energy_store.h"
#include "ps/drivetrain/brake_system_desc.h"
#include <algorithm>
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
    real supply(real j) {const real added=std::max(real(0),set_content(content()+j));discharge_left_+=added;return added;}
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
        if(!std::isfinite(desc_.regen_power_w)||desc_.regen_power_w<0)throw std::invalid_argument("electric motor regen_power_w must be finite and >= 0");
        if(desc_.efficiency>1||!std::isfinite(desc_.regen_torque_nm)||desc_.regen_torque_nm<0||
           !std::isfinite(desc_.mass_kg)||desc_.mass_kg<0)throw std::invalid_argument("invalid electric motor efficiency, regeneration or mass");
    }
    void pre_solve(const ComponentSubstepCtx& ctx,const ShaftNetwork& net,Assembly& assembly) override {
        torque_nm=power_w=battery_power_w=regen_d_=regen_pred_nm=0;
        const auto channel=[&](int slot,real fallback){return ctx.channels&&slot>=0&&slot<ctx.channel_count?ctx.channels[slot]:fallback;};
        const real omega=net.omega(shaft_);
        rpm=std::abs(omega)*60/(2*3.14159265358979323846);
        ready=channel(ready_slot_,real(1))>.5;
        if(ctx.h<=0||!ready)return;
        const real request=channel(direction_slot_,desc_.crank_assist?real(1):ctx.electric_direction);
        // N/unspecified is no propulsion; a gearbox-equipped hybrid can use its positive gear request too.
        const real direction=request==kGearRequestNone||request==0?0:(request<0?-1:1);
        const real pedal=std::clamp(channel(throttle_slot_,desc_.crank_assist&&ctx.electric_charge>=0?real(0):(ctx.electric_throttle>=0?ctx.electric_throttle:ctx.throttle)),real(0),real(1));
        const real regen=std::clamp(channel(regen_slot_,desc_.crank_assist&&ctx.electric_charge>=0?ctx.electric_charge:ctx.brake),real(0),real(1));
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
            real demand=desc_.regen_torque_nm*regen;
            if(blend_) {
                // Brake blending: the motor takes the pedal's request first (request_nm_, motor-shaft terms, set by the
                // Powertrain from the rim brake request), limited by its torque and power rating, the pack's charge
                // acceptance, the low-speed fade; ABS acting on a wheel it drives withdraws it altogether.
                const real power=desc_.regen_power_w>0?desc_.regen_power_w:desc_.max_power_w;
                const real accept=(abs_now_||abs_hold_left_>0)?real(0):
                    std::clamp((cfg_.full_soc-battery_.soc())/(cfg_.full_soc-cfg_.taper_start_soc),real(0),real(1))*fade_;
                demand=std::min({desc_.regen_torque_nm,request_nm_,power/std::abs(omega)})*accept;
            }
            const real wanted=ctx.h*demand*std::abs(omega)*desc_.efficiency;
            const real reserved=battery_.reserve_charge(wanted);
            regen_d_=demand*(wanted>0?reserved/wanted:0)/std::abs(omega);
            regen_pred_nm=regen_d_*std::abs(omega);
        }
        // The network linearises about omega_old (torque = tau0 - d (omega_new - omega_old)), so a damping torque
        // -d*omega_new needs tau0 = -d*omega_old; with tau0 = 0 the regeneration only resisted CHANGES of speed.
        assembly.torque(shaft_,torque_nm-regen_d_*omega,regen_d_);
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
        for(real v:{desc_.peak_torque_nm,desc_.max_power_w,desc_.max_rpm,desc_.efficiency,desc_.regen_torque_nm,real(desc_.crank_assist)}){
            const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
        }
        // Brake blending: configuration and the ABS hold timer are hashed only on a blended motor, the regen power limit
        // only when set, so a motor outside the scheme hashes as before.
        if(desc_.regen_power_w!=0||blend_) {
            for(real v:{desc_.regen_power_w,real(blend_),blend_?cfg_.taper_start_soc:real(0),blend_?cfg_.full_soc:real(0),
                        blend_?cfg_.fade_start_speed_mps:real(0),blend_?cfg_.fade_end_speed_mps:real(0),blend_?cfg_.abs_hold_s:real(0),
                        blend_?abs_hold_left_:real(0)}){
                const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
            }
        }
        for(const auto* name:{&desc_.battery,&desc_.ready_channel,&desc_.throttle_channel,&desc_.regen_channel,&desc_.direction_channel}){
            for(unsigned char c:*name){h^=c;h*=1099511628211ULL;}h^=0;h*=1099511628211ULL;
        }
        return h;
    }
    real rated_power() const {return desc_.max_power_w;}
    real available_power() const {return std::min(desc_.max_power_w,desc_.peak_torque_nm*rpm*(2*3.14159265358979323846)/60);}
    bool is_crank_machine() const {return desc_.crank_assist;}
    // Brake blending (Powertrain wires it when the motor's rims are known and regen_blend is enabled).
    void enable_blend(const RegenBlendDesc& cfg) {blend_=true;cfg_=cfg;}
    [[nodiscard]] bool blended() const {return blend_;}
    // Per substep, before pre_solve: the rim brake request in motor-shaft terms, the speed fade, ABS acting on a driven wheel.
    void set_blend_feed(real request_nm,real fade,bool abs_active,real h) {
        request_nm_=request_nm;fade_=fade;abs_now_=abs_active;
        abs_hold_left_=abs_active?cfg_.abs_hold_s:std::max(real(0),abs_hold_left_-h);
    }
    void reset_blend() {abs_hold_left_=0;abs_now_=false;request_nm_=0;fade_=0;regen_pred_nm=0;}
    [[nodiscard]] real regen_efficiency() const {return desc_.efficiency;}
    bool ready=true;
    real torque_nm=0,power_w=0,battery_power_w=0,rpm=0;
    real regen_pred_nm=0; // this substep's regeneration torque magnitude at the shaft (implicit damping d x |omega_old|)
private:
    real regen_d_=0;
    bool blend_=false,abs_now_=false;
    RegenBlendDesc cfg_{};
    real request_nm_=0,fade_=0,abs_hold_left_=0;
    NodeId shaft_; ElectricMotorDesc desc_; Battery& battery_;
    int throttle_slot_=-1,regen_slot_=-1,direction_slot_=-1,ready_slot_=-1;
};
class FuelCell final : public IComponent {
public:
    FuelCell(FuelCellDesc desc,Battery& battery,int ready_slot)
        :hydrogen_kg(desc.initial_hydrogen_kg),desc_(std::move(desc)),battery_(battery),ready_slot_(ready_slot) {
        for(real v:{desc_.max_power_w,desc_.specific_energy_j_kg,desc_.hydrogen_capacity_kg,desc_.efficiency})
            if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid fuel cell power, fuel capacity or efficiency");
        if(desc_.efficiency>1||!std::isfinite(desc_.target_soc)||desc_.target_soc<=0||desc_.target_soc>1||
           !std::isfinite(hydrogen_kg)||hydrogen_kg<0||hydrogen_kg>desc_.hydrogen_capacity_kg||
           !std::isfinite(desc_.mass_kg)||desc_.mass_kg<0)throw std::invalid_argument("invalid fuel cell state, mass or target SOC");
    }
    void pre_solve(const ComponentSubstepCtx& ctx,const ShaftNetwork&,Assembly&) override {
        reserved_=0;power_w=0;
        const bool ready=ctx.channels&&ready_slot_>=0&&ready_slot_<ctx.channel_count?ctx.channels[ready_slot_]>.5:true;
        if(!ready||ctx.h<=0||hydrogen_kg<=0)return;
        const real fraction=std::clamp((desc_.target_soc-battery_.soc())/real(.05),real(0),real(1));
        const real available=hydrogen_kg*desc_.specific_energy_j_kg*desc_.efficiency;
        reserved_=battery_.reserve_charge(std::min(available,ctx.h*desc_.max_power_w*fraction));
        reserved_=battery_.supply(reserved_);
    }
    void post_solve(const ComponentSubstepCtx& ctx,const ShaftNetwork&,const Assembly&,PowertrainEnergy& ledger) override {
        const real added=reserved_;
        const real chemical=added/desc_.efficiency;
        const real consumed=std::min(hydrogen_kg,chemical/desc_.specific_energy_j_kg);
        hydrogen_kg=std::max(real(0),hydrogen_kg-consumed);
        power_w=ctx.h>0?added/ctx.h:0;
        ledger.fuel_cell_electric+=added;
        ledger.fuel_cell_chemical+=chemical;
        ledger.fuel_cell_loss+=chemical-added;
    }
    std::uint64_t fold_state_hash(std::uint64_t h) const override {
        for(real v:{hydrogen_kg,desc_.max_power_w,desc_.efficiency,desc_.target_soc,desc_.specific_energy_j_kg}) {
            const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
        }return h;
    }
    real rated_power() const {return desc_.max_power_w;}
    real hydrogen_kg=0,power_w=0;
private:
    FuelCellDesc desc_;Battery& battery_;int ready_slot_=-1;real reserved_=0;
};

}
