#pragma once
#include "ps/aero/aerodynamics.h"
#include <string>
namespace ps::io {
// Upfront parse/validation. Selection {enabled: bool, coefficient_map: reference}.
// Paths relative to selection. False removes map only; authored aero preserved.
// Strong exception guarantee: descriptor unchanged on failure.
void load_aero_coefficient_map_reference(const std::string& selection_json_path,aero::Desc&);
}
