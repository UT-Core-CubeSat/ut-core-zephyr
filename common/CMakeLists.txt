zephyr_library_named(common)

zephyr_library_sources(
    can_proto.c
    health.c
    # Add more source files here as they are created
)

add_dependencies(common zephyr_generated_headers)