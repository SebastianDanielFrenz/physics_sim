# ps_field_run - headless field scenario runner (farming plan F1a): scenario
# JSON -> daily CSV. Links ps_field_io only, never ps_core.

add_executable(ps_field_run main.cpp)

target_link_libraries(ps_field_run
  PRIVATE
    ps_field_io
    ps_headless_env
    ps_warnings
    ps_options
)
