zephyr_library_named(common)

zephyr_library_sources(
    can_proto.c
    # Add more source files here as they are created
)

target_link_libraries(common PUBLIC zephyr_interface)
