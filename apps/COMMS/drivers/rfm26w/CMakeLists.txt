# The public header is available to the whole build, even when the driver is disabled.
zephyr_include_directories(include)

if(CONFIG_RFM26W)
  zephyr_library()
  zephyr_library_sources(src/rfm26w.c src/rfm26w_profiles.c)
endif()
