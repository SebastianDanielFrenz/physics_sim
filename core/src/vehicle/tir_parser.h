// core/src/vehicle/tir_parser.h — minimal parser for the standard MF-Tyre
// ".tir" parameter-file format (PLAN.md D6: "a .tir parser for the standard
// MF parameter file format plus our own JSON, both data-driven").
//
// Supports the subset of section/key names tyre_desc.h's TyreDesc actually
// uses (see the .cpp's own table) - a real .tir carries many more keys
// (e.g. full scaling coefficients, structural/gyroscopic properties) that
// this codebase does not model yet; unrecognised keys are ignored, not an
// error, so a real published .tir file can be dropped in unmodified and
// only the recognised subset is picked up.

#pragma once

#include "ps/vehicle/tyre_desc.h"

#include <string>

namespace ps::vehicle {

// Throws std::runtime_error on an unreadable file. `source` (tyre_desc.h's
// required provenance field) is set to source_note if non-empty, else left
// for the caller to fill in (a .tir file has no standard "cite your
// source" field).
TyreDesc parse_tir_file(const std::string& path, const std::string& source_note = std::string());

} // namespace ps::vehicle
