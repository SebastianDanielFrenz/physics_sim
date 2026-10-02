// adapters/godot/src/register_types.cpp — GDExtension entry point. See
// demo/ps_sim.gdextension's entry_symbol for how Godot finds
// ps_godot_library_init below.

#include "register_types.h"

#include "debug_draw.h"
#include "input_map.h"
#include "ps_body_node.h"
#include "ps_simulation.h"
#include "spatial_audio.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_ps_godot_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
    ClassDB::register_class<ps_godot::PsSimulation>();
    ClassDB::register_class<ps_godot::PsSpatialAudio>();
    ClassDB::register_class<ps_godot::PsBodyNode3D>();
    ClassDB::register_class<ps_godot::PsDebugDraw>();
    ClassDB::register_class<ps_godot::PsInputMap>();
}

void uninitialize_ps_godot_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
}

extern "C" {
GDExtensionBool GDE_EXPORT ps_godot_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
                                                  GDExtensionClassLibraryPtr p_library,
                                                  GDExtensionInitialization* r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_ps_godot_module);
    init_obj.register_terminator(uninitialize_ps_godot_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
}
}
