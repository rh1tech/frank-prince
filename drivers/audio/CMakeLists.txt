add_library(audio_driver INTERFACE)

target_sources(audio_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_driver.c
)

target_include_directories(audio_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(audio_driver INTERFACE
    pico_stdlib
    pico_sync
    hardware_gpio
    hardware_dma
    hardware_clocks
    hardware_irq
    hardware_pio
)

# Store the PIO file path for the parent to generate the header
# (pico_generate_pio_header needs a concrete target, not INTERFACE)
set(AUDIO_I2S_PIO_FILE ${CMAKE_CURRENT_LIST_DIR}/audio_i2s.pio PARENT_SCOPE)
