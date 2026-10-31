// core/src/drivetrain/limit_findings.cpp - to_string for the limit finding enums (ps/drivetrain/limit_findings.h).
#include "ps/drivetrain/limit_findings.h"

namespace ps::drivetrain {

const char* to_string(LimitKind k) {
    switch (k) {
    case LimitKind::WastegateSaturated: return "WastegateSaturated";
    case LimitKind::BoostBelowTarget: return "BoostBelowTarget";
    case LimitKind::BoostAboveTarget: return "BoostAboveTarget";
    case LimitKind::TurboRotorCap: return "TurboRotorCap";
    case LimitKind::CompressorSurge: return "CompressorSurge";
    case LimitKind::CompressorChoke: return "CompressorChoke";
    case LimitKind::ChargeTemperature: return "ChargeTemperature";
    case LimitKind::ClutchCapacity: return "ClutchCapacity";
    case LimitKind::GearboxInputTorque: return "GearboxInputTorque";
    case LimitKind::KitFuelLean: return "KitFuelLean";
    case LimitKind::InjectorDuty: return "InjectorDuty";
    case LimitKind::N2OBottleDuration: return "N2OBottleDuration";
    case LimitKind::N2OBottlePressureDrop: return "N2OBottlePressureDrop";
    case LimitKind::RevLimiter: return "RevLimiter";
    case LimitKind::FuelCut: return "FuelCut";
    case LimitKind::ValveFlowChoke: return "ValveFlowChoke";
    case LimitKind::MeanPistonSpeed: return "MeanPistonSpeed";
    case LimitKind::MapGridEdge: return "MapGridEdge";
    case LimitKind::Knock: return "Knock";
    case LimitKind::BeyondEngineData: return "BeyondEngineData";
    case LimitKind::TowedMass: return "TowedMass";
    case LimitKind::TongueLoad: return "TongueLoad";
    case LimitKind::LiftCapacity: return "LiftCapacity";
    case LimitKind::PtoShaftTorque: return "PtoShaftTorque";
    case LimitKind::ToolPower: return "ToolPower";
    }
    return "Unknown";
}

const char* to_string(LimitSeverity s) {
    switch (s) {
    case LimitSeverity::Near: return "Near";
    case LimitSeverity::Limiting: return "Limiting";
    case LimitSeverity::Exceeded: return "Exceeded";
    }
    return "Unknown";
}

const char* to_string(LimitUnit u) {
    switch (u) {
    case LimitUnit::Dimensionless: return "1";
    case LimitUnit::Fraction: return "fraction";
    case LimitUnit::Pascal: return "Pa";
    case LimitUnit::Kelvin: return "K";
    case LimitUnit::NewtonMetre: return "N*m";
    case LimitUnit::MetrePerSecond: return "m/s";
    case LimitUnit::RevPerMinute: return "rpm";
    case LimitUnit::SquareMetre: return "m^2";
    case LimitUnit::Millimetre: return "mm";
    case LimitUnit::Kilogram: return "kg";
    case LimitUnit::Second: return "s";
    case LimitUnit::CrankDegree: return "deg";
    case LimitUnit::Newton: return "N";
    case LimitUnit::Kilowatt: return "kW";
    }
    return "Unknown";
}

} // namespace ps::drivetrain
