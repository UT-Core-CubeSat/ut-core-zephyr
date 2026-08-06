add_library(common STATIC
    can_proto.c
    # Add more source files here as they are created
)

target_link_libraries(common PUBLIC zephyr_interface)
