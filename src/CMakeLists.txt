file(GLOB_RECURSE app_sources "*.cpp" "*.c")

# espressif__esp32-camera is the managed component's real registered name
# (ESP-IDF names a component after its directory, verbatim -- see
# component.cmake's own component_name = base_dir). Listed explicitly rather
# than relying on automatic requirements resolution, which does not appear to
# pick up a managed (idf_component.yml) dependency for this component the
# way it does for built-in ESP-IDF components.
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES espressif__esp32-camera)
