# ps_engine_tools (engine garage P6, design A14): engine-level services that
# need BOTH the simulation (ps_core) and the live voice (ps_audio) - today the
# audio bank baker (ps_engine_tools/audio_bank_prep.h). It links both; neither
# of them may link it (ps_audio stays free of ps_core, guarded by
# ps_audio_standalone). A top-level directory like audio/, not tools/: a game
# or a garage links it as a library, it is not a command-line tool.
#
# core/src is on the PRIVATE include path only: the baker settles the same
# SimulatedEngine the vehicles run (drivetrain/engine_simulated.h is internal);
# nothing internal appears in the public header.

find_package(Threads REQUIRED)

add_library(ps_engine_tools STATIC
  src/audio_bank_prep.cpp
)

target_include_directories(ps_engine_tools
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/core/src
)

target_link_libraries(ps_engine_tools
  PUBLIC
    ps_core
    ps_audio
  PRIVATE
    Threads::Threads
    ps_warnings
    ps_options
)
