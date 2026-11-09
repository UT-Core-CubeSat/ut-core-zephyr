add_library(common STATIC
    can_proto.c
    soh.c
    # Add more source files here as they are created
)

target_link_libraries(common PUBLIC zephyr_interface)
