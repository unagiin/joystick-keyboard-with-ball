if(CONFIG_ZMK_KSCAN_JOYSTICK)
  zephyr_library()
  zephyr_library_sources(
    drivers/kscan/kscan_joystick.c
    drivers/kscan/joystick_logic.c
  )
endif()
