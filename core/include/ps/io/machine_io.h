// ps/io/machine_io.h — JSON loading for machine parts (physics_sim.machine/1) and
// the shop's power estimate (physics_sim.tool_power/1), docs/data_formats/machine.md.
// "extends" works as in every part file; unknown keys are rejected. A part names the
// sockets of its model: the caller hands their positions over, in the vehicle's
// chassis axes, so that no coordinate lives in two files.

#pragma once

#include "ps/machine/tool.h"
#include "ps/math/vec3.h"

#include <map>
#include <string>

namespace ps::io {

using MachineSockets = std::map<std::string, Vec3>;

[[nodiscard]] machine::MachineDesc load_machine_json(const std::string& path, const MachineSockets& sockets);
[[nodiscard]] machine::ToolPowerSpec load_tool_power_json(const std::string& path);

} // namespace ps::io
