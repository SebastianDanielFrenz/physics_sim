// adapters/godot/src/register_types.h — GDExtension entry point.

#pragma once

#include <godot_cpp/core/class_db.hpp>

void initialize_ps_godot_module(godot::ModuleInitializationLevel p_level);
void uninitialize_ps_godot_module(godot::ModuleInitializationLevel p_level);
