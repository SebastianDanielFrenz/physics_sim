# ps_audio (ps_audio A7): the streaming engine voice - LiveEngine on the shared
# EngineCore, the EngineInput SPSC queue, the time map with its rate PLL, the
# Runner thread and the float ring. Links ps_engine_cycle (+ ps_detmath for
# ps::math) ONLY, never ps_core; guarded by ps_audio_standalone (defined in
# tests/unit/CMakeLists.txt, next to ps_engine_cycle_standalone). No device
# code and no third-party dependency (the device backend is A10).

find_package(Threads REQUIRED)

add_library(ps_audio STATIC
  src/audio_bank.cpp
  src/halfband.cpp
  src/live_engine.cpp
  src/runner.cpp
  src/source.cpp
  src/time_map.cpp
  src/voice.cpp
)

target_include_directories(ps_audio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ps_audio
  PUBLIC
    ps_engine_cycle
  PRIVATE
    ps_detmath
    Threads::Threads
    ps_warnings
    ps_options
)
