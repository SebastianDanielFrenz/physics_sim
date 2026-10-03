# ps_engine_audio - ps_audio A10 (out/audio_plan.md section 6): an
# engine-only offline/live listening-test host. Links ps_core (EngineRig,
# ps::io::load_engine_json) AND ps_audio (Voice) - the ONLY place in this
# project both are linked into the same binary; the EngineRig -> Voice glue
# (EngineSoundState -> ps_audio::EngineInput) lives here, in the TOOL, not in
# either library (see engine_rig.h's own header comment and audio/voice.h's
# engine_input.h comment "A10 fills it from ps::drivetrain::EngineSoundState").
#
# miniaudio (single-header, public domain / MIT-0, https://miniaudio.h) is
# fetched HERE, scoped to this target only - never linked into ps_core or
# ps_audio (out/audio_plan.md section 10 Q5: owner-approved). Pinned to a
# release tag, same FetchContent pattern the root CMakeLists.txt already
# uses for JoltPhysics/nlohmann_json/Catch2.
include(FetchContent)
FetchContent_Declare(
  miniaudio
  GIT_REPOSITORY https://github.com/mackron/miniaudio.git
  GIT_TAG        0.11.21
)
FetchContent_MakeAvailable(miniaudio)

add_executable(ps_engine_audio
  ${PROJECT_SOURCE_DIR}/tools/common/engine_bank_generator.cpp
  main.cpp
  wav_writer.cpp
  script.cpp
  miniaudio_impl.cpp
)

target_include_directories(ps_engine_audio PRIVATE ${miniaudio_SOURCE_DIR})

# miniaudio_impl.cpp's generated implementation code is third-party and not
# held to ps_warnings' warnings-as-errors bar: warnings off for that one file
# ("/w" for the MSVC-style front ends clang-cl and cl, "-w" for clang/gcc on
# Linux - the Linux presets build every target, this one included).
set_source_files_properties(miniaudio_impl.cpp PROPERTIES COMPILE_OPTIONS
  "$<IF:$<STREQUAL:${CMAKE_CXX_COMPILER_FRONTEND_VARIANT},MSVC>,/w,-w>")

target_link_libraries(ps_engine_audio
  PRIVATE
    ps_core
    ps_audio
    ps_headless_env
    nlohmann_json::nlohmann_json
    ps_warnings
    ps_options
)

if(WIN32)
  # miniaudio's WASAPI/DirectSound/WinMM backends need these; Jolt/ps_core
  # never pull them in, so they are not already on the link line.
  target_link_libraries(ps_engine_audio PRIVATE ole32 winmm)
else()
  # miniaudio on Linux loads its backends with dlopen and runs a device thread.
  find_package(Threads REQUIRED)
  target_link_libraries(ps_engine_audio PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
endif()

target_include_directories(ps_engine_audio PRIVATE ${PROJECT_SOURCE_DIR}/core/src ${PROJECT_SOURCE_DIR}/tools/common)
