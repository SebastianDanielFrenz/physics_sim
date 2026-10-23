#pragma once
#include "components.h"
#include "energy_store.h"
#include "ps/drivetrain/brake_system_desc.h"
#include "ps/drivetrain/electric_envelope.h"
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
    // The substep budgets are the public envelope functions (ps/drivetrain/electric_envelope.h), so the dyno uses this law.
    void begin_substep(real h) {discharge_left_=battery_discharge_budget_j(desc,joules_,h);charge_left_=battery_charge_budget_j(desc,joules_,h);claims_left_=0;}
    real supply(real j) {const real added=std::max(real(0),set_content(content()+j));discharge_left_+=added;return added;}
    // Discharge allocation (2026-10-19): each driving motor registers its request (add_drive_claim, by the Powertrain before
    // the component loop); while the claims still to come exceed what is left, a request gets its PROPORTIONAL share of what
    // is left, so a pack that cannot feed every motor splits its budget in the motors' request ratio instead of in component
    // order (which left the last motor of a battery-limited dual-motor car with nothing). An unlimited pack, a sole or last
    // claimant, and a request without a claim take min(left, request) as before.
    void add_drive_claim(real j) {claims_left_+=std::max(real(0),j);}
    real reserve_discharge(real j) {
        const real w=std::max(real(0),j);
        const real n=(claims_left_>discharge_left_&&claims_left_>w)?std::min(w,discharge_left_*(w/claims_left_)):std::min(discharge_left_,w);
        claims_left_=std::max(real(0),claims_left_-w);
        discharge_left_-=n;return n;
    }
    real reserve_charge(real j) {const real n=std::min(charge_left_,std::max(real(0),j));charge_left_-=n;return n;}
    std::uint64_t fold_state_hash(std::uint64_t h) const override {
        for(real v:{capacity(),joules_,desc.max_discharge_w,desc.max_charge_w}) {
            const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}
        }return h;
    }
    BatteryDesc desc;
private:
    real joules_=0,discharge_left_=0,charge_left_=0,claims_left_=0; // claims_left_: unhashed, rebuilt every substep
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
        torque_nm=power_w=battery_power_w=regen_d_=regen_pred_nm=regen_brake_pred_nm=0;drive_limit=MotorLimit::Idle;
        const Inputs in=inputs(ctx,net);
        const real omega=in.omega,direction=in.direction,regen=in.regen;
        rpm=in.rpm;
        ready=in.ready;
        if(ctx.h<=0||!ready)return;
        drive_limit=rpm>=desc_.max_rpm?MotorLimit::MaxRpm:MotorLimit::Idle;
        if(in.drives) {
            // The drive law is the public envelope (ps/drivetrain/electric_envelope.h): the power corner with the rotor-inertia
            // prediction, the pack's grant, the residual-energy bound near empty. One implementation for the car and the dyno.
            const MotorDriveRequest r=motor_drive_request(desc_,std::abs(omega),in.command,ctx.h,in.scale);
            const real reserved=battery_.reserve_discharge(r.wanted_j);
            torque_nm=direction*motor_drive_torque(desc_,r,reserved,battery_.content(),ctx.h,&drive_limit);
        } else if(direction!=0&&(regen>0||b_regen_nm_>0)&&std::abs(omega)>1) {
            real demand=std::min(desc_.regen_torque_nm*regen,
                (desc_.regen_power_w>0?desc_.regen_power_w:desc_.max_power_w)/std::abs(omega));
            real total_request_nm=request_nm_;
            if(blend_) {
                // Brake blending: the motor takes the pedal's request first (request_nm_, motor-shaft terms, set by the
                // Powertrain from the rim brake request), limited by its torque and power rating, the pack's charge
                // acceptance, the low-speed fade; ABS acting on a wheel it drives withdraws it altogether.
                const real power=desc_.regen_power_w>0?desc_.regen_power_w:desc_.max_power_w;
                const real accept=(abs_now_||abs_hold_left_>0)?real(0):
                    std::clamp((cfg_.full_soc-battery_.soc())/(cfg_.full_soc-cfg_.taper_start_soc),real(0),real(1))*fade_*ramp_;
                // B mode (docs/electric_drive.md): the one-pedal regeneration request adds to the brake pedal's.
                if(b_regen_nm_>0)total_request_nm=request_nm_+b_regen_nm_;
                demand=std::min({desc_.regen_torque_nm,total_request_nm,power/std::abs(omega)})*accept;
            }
            const real wanted=ctx.h*demand*std::abs(omega)*desc_.efficiency;
            const real reserved=battery_.reserve_charge(wanted);
            regen_d_=demand*(wanted>0?reserved/wanted:0)/std::abs(omega);
            regen_pred_nm=regen_d_*std::abs(omega);
            // The pads only account for the brake pedal's part of the regeneration (the B part is not a brake request).
            regen_brake_pred_nm=(b_regen_nm_>0&&total_request_nm>0)?regen_pred_nm*(request_nm_/total_request_nm):regen_pred_nm;
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
        // A motor behind a gearbox, clutch or transfer case also hashes its path ramp (state) and the gating configuration.
        if(blend_&&dynamic_) {
            for(real v:{ramp_,cfg_.path_ramp_s,cfg_.path_min_engagement,cfg_.clutch_capacity_margin}){
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
    // dynamic_paths: some path to the rims runs through a gearbox, the main clutch or a transfer case (the regeneration then
    // ramps in over path_ramp_s whenever such a path closes again).
    void enable_blend(const RegenBlendDesc& cfg,bool dynamic_paths=false) {blend_=true;cfg_=cfg;dynamic_=dynamic_paths;ramp_=1;}
    [[nodiscard]] bool blended() const {return blend_;}
    // Per substep, before pre_solve: the rim brake request in motor-shaft terms, the speed fade, ABS acting on a driven wheel.
    // path_open: at least one path to a rim is open this substep (always true outside a dynamic path set).
    void set_blend_feed(real request_nm,real fade,bool abs_active,real h,bool path_open=true) {
        request_nm_=request_nm;fade_=fade;abs_now_=abs_active;
        abs_hold_left_=abs_active?cfg_.abs_hold_s:std::max(real(0),abs_hold_left_-h);
        if(dynamic_) ramp_=!path_open?real(0):(cfg_.path_ramp_s>0?std::min(real(1),ramp_+h/cfg_.path_ramp_s):real(1));
    }
    void reset_blend() {abs_hold_left_=0;abs_now_=false;request_nm_=0;fade_=0;regen_pred_nm=0;regen_brake_pred_nm=0;ramp_=1;b_active_=false;b_drive_cmd_=b_regen_nm_=0;}
    // B mode (Powertrain::prepare_b, per substep before pre_solve): the system power law's per-motor outputs. b_active: the
    // motor ignores the pedal channel and takes drive_cmd (0..1 of its envelope torque) or regen_nm (motor-shaft torque to
    // regenerate on top of the brake pedal's request). Not hashed itself (a pure function of the hashed B state and inputs).
    void set_b_feed(bool active,real drive_cmd,real regen_nm,real drive_scale=real(1)) {b_active_=active;b_drive_cmd_=active?drive_cmd:real(0);b_regen_nm_=active?regen_nm:real(0);b_scale_=active?drive_scale:real(1);}
    // The regeneration torque the motor can take at |omega| right now, before the pack's per-substep acceptance: torque and
    // power limits x the SOC taper x the low-speed fade x the path ramp; 0 under ABS or in the hold after it.
    [[nodiscard]] real b_regen_capacity_nm(real abs_omega) const {
        if(!blend_||!(abs_omega>1))return 0;
        const real power=desc_.regen_power_w>0?desc_.regen_power_w:desc_.max_power_w;
        const real accept=(abs_now_||abs_hold_left_>0)?real(0):
            std::clamp((cfg_.full_soc-battery_.soc())/(cfg_.full_soc-cfg_.taper_start_soc),real(0),real(1))*fade_*ramp_;
        return std::min(desc_.regen_torque_nm,power/abs_omega)*accept;
    }
    // The drive torque envelope at |omega|: the torque rating, then the power corner; zero at or above max_rpm.
    [[nodiscard]] real drive_envelope_torque(real abs_omega) const {
        if(abs_omega*60/(2*3.14159265358979323846)>=desc_.max_rpm)return 0;
        return std::min(desc_.peak_torque_nm,desc_.max_power_w/std::max(real(1),abs_omega));
    }
    [[nodiscard]] bool b_active() const {return b_active_;}
    [[nodiscard]] real regen_efficiency() const {return desc_.efficiency;}
    [[nodiscard]] NodeId shaft_node() const {return shaft_;}
    // The drive request this motor will make in pre_solve this substep (0 if it will not drive): the Powertrain registers it
    // with the pack before the component loop (Battery::add_drive_claim). Same inputs as pre_solve.
    [[nodiscard]] real drive_claim_j(const ComponentSubstepCtx& ctx,const ShaftNetwork& net) const {
        const Inputs in=inputs(ctx,net);
        if(ctx.h<=0||!in.ready||!in.drives)return 0;
        return motor_drive_request(desc_,std::abs(in.omega),in.command,ctx.h,in.scale).wanted_j;
    }
    [[nodiscard]] Battery& battery() const {return battery_;}
    [[nodiscard]] const ElectricMotorDesc& desc() const {return desc_;}
    bool ready=true;
    real torque_nm=0,power_w=0,battery_power_w=0,rpm=0;
    MotorLimit drive_limit=MotorLimit::Idle; // this substep's drive-torque limit (diagnostic for the dyno, never hashed)
    real regen_pred_nm=0; // this substep's regeneration torque magnitude at the shaft (implicit damping d x |omega_old|)
    real regen_brake_pred_nm=0; // the part of it that answers the brake pedal (== regen_pred_nm outside B mode)
private:
    struct Inputs {real omega=0,rpm=0,direction=0,pedal=0,regen=0,command=0,scale=1;bool ready=false,drives=false;};
    Inputs inputs(const ComponentSubstepCtx& ctx,const ShaftNetwork& net) const {
        Inputs in;
        const auto channel=[&](int slot,real fallback){return ctx.channels&&slot>=0&&slot<ctx.channel_count?ctx.channels[slot]:fallback;};
        in.omega=net.omega(shaft_);
        in.rpm=std::abs(in.omega)*60/(2*3.14159265358979323846);
        in.ready=channel(ready_slot_,real(1))>.5;
        const real request=channel(direction_slot_,desc_.crank_assist?real(1):ctx.electric_direction);
        // N/unspecified is no propulsion; a gearbox-equipped hybrid can use its positive gear request too.
        in.direction=request==kGearRequestNone||request==0?0:(request<0?-1:1);
        in.pedal=std::clamp(channel(throttle_slot_,desc_.crank_assist&&ctx.electric_charge>=0?real(0):(ctx.electric_throttle>=0?ctx.electric_throttle:ctx.throttle)),real(0),real(1));
        in.regen=std::clamp(channel(regen_slot_,desc_.crank_assist&&ctx.electric_charge>=0?ctx.electric_charge:ctx.brake),real(0),real(1));
        // The ceiling is a fraction of the torque available now (ICE meaning), applied after the power corner.
        const real ceiling=std::clamp(ctx.traction_throttle_ceiling,real(0),real(1));
        if(b_active_) {
            // B mode: the drive command is the B law's (already a fraction of the envelope); the TC ceiling scales it through
            // b_scale_ (set by the Powertrain from the driver's pedal). B regeneration is never scaled by the ceiling.
            in.pedal=b_drive_cmd_;
            in.drives=b_drive_cmd_>0&&in.regen<=.05&&in.direction!=0&&in.rpm<desc_.max_rpm;
            in.command=in.pedal;
            in.scale=b_scale_;
        } else {
            in.drives=motor_drives(desc_,in.pedal,in.regen,in.direction,in.rpm);
            in.command=in.pedal;
            in.scale=in.pedal>0&&ceiling<in.pedal?ceiling/in.pedal:real(1);
        }
        return in;
    }
    real regen_d_=0;
    bool blend_=false,abs_now_=false,dynamic_=false;
    RegenBlendDesc cfg_{};
    real request_nm_=0,fade_=0,abs_hold_left_=0,ramp_=1;
    bool b_active_=false;real b_drive_cmd_=0,b_regen_nm_=0,b_scale_=1;
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
        reserved_=battery_.reserve_charge(fuel_cell_request_j(desc_,hydrogen_kg,battery_.soc(),ctx.h)); // electric_envelope.h
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
    [[nodiscard]] const Battery& battery() const {return battery_;}
    [[nodiscard]] const FuelCellDesc& desc() const {return desc_;}
    real hydrogen_kg=0,power_w=0;
private:
    FuelCellDesc desc_;Battery& battery_;int ready_slot_=-1;real reserved_=0;
};

}
