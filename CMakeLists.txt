cmake_minimum_required(VERSION 3.13)

# Set board to Pico 2 (RP2350)
set(PICO_BOARD pico2 CACHE STRING "Pico board type")

# Board configuration variant (M1, M2)
set(BOARD_VARIANT "M1" CACHE STRING "Board variant: M1 or M2")

# CPU overclock configuration (252, 378, 504 MHz)
set(CPU_SPEED "378" CACHE STRING "CPU clock in MHz: 252, 378, 504")

# PSRAM speed configuration (84, 100, 133, 166 MHz target)
set(PSRAM_SPEED "133" CACHE STRING "PSRAM max frequency in MHz: 84, 100, 133, 166")

# SDL/Video diagnostics
set(RP2350_FORCE_TEST_PATTERN "0" CACHE STRING "If 1, bypass SDLPoP pixels and draw a known test pattern")
set(RP2350_DUMP_FIRST_FRAME_BYTES "1" CACHE STRING "If 1, dump first-frame source bytes in SDL_UpdateTexture")
set(RP2350_DEBUG_INDEX_BAR "0" CACHE STRING "If 1, overlay a top-row palette index bar in SDL_UpdateTexture")

# Boot-time diagnostics (isolates HDMI scanout)
set(RP2350_BOOT_TEST_PATTERN "0" CACHE STRING "If 1, show a boot-time 16-color test pattern")
set(RP2350_BOOT_TEST_PATTERN_HALT "1" CACHE STRING "If 1, halt after showing boot-time pattern")
set(RP2350_BOOT_TEST_PATTERN_MODE "0" CACHE STRING "Boot test pattern mode: 0=checker16, 1=ramp239+grayscale")

# CPU voltage selection based on speed
if(CPU_SPEED GREATER_EQUAL 504)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_65")
elseif(CPU_SPEED GREATER_EQUAL 378)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_60")
else()
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_50")
endif()

message(STATUS "Board: ${BOARD_VARIANT}, CPU: ${CPU_SPEED} MHz, PSRAM: ${PSRAM_SPEED} MHz, Voltage: ${CPU_VOLTAGE}")

include(pico_sdk_import.cmake)

project(frank-prince C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Default is to keep using the current lightweight SDL shim in `src/SDL_port.c`.
# When enabled, this expects an upstream SDL2 checkout under `third_party/SDL2`.
option(USE_REAL_SDL2 "Build against upstream SDL2 instead of the local shim" OFF)

# Debug malloc failures (prints the failing allocation size/type right before the
# Pico SDK triggers the Out-of-memory panic). Useful for tracking down the exact
# allocation site during bring-up.
#
# Note: this needs to be a compile definition (not just a CMake cache variable)
# so it affects the Pico SDK's `pico_malloc` library sources.
add_compile_definitions(
    PICO_DEBUG_MALLOC=0
    PICO_DEBUG_MALLOC_LOW_WATER=0
)

pico_sdk_init()

# FatFS + SD card driver
add_subdirectory(src/fatfs)
add_subdirectory(drivers/sdcard)
add_subdirectory(drivers/ps2kbd)
add_subdirectory(drivers/audio)
add_subdirectory(drivers/usbhid)

# SDLPoP (vendored)
file(GLOB SDLPoP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/third_party/SDLPoP/src/*.c"
)
list(REMOVE_ITEM SDLPoP_SOURCES "${CMAKE_CURRENT_LIST_DIR}/third_party/SDLPoP/src/main.c")

set(SDLPoP_MAIN "${CMAKE_CURRENT_LIST_DIR}/third_party/SDLPoP/src/main.c")

add_library(sdlpop STATIC ${SDLPoP_SOURCES} ${SDLPoP_MAIN})

# Rename SDLPoP's entrypoint so the firmware can call it.
# (SDLPoP already has a different function named pop_main.)
set_source_files_properties(${SDLPoP_MAIN} PROPERTIES COMPILE_DEFINITIONS "main=sdlpop_entry")

# RP2350 bring-up: trace a specific suspicious allocation size inside SDLPoP
# without affecting the Pico SDK / firmware sources.
set_source_files_properties(${SDLPoP_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-include${CMAKE_CURRENT_LIST_DIR}/src/rp2350_alloc_trace.h"
    COMPILE_DEFINITIONS "RP2350_ALLOC_TRACE_ENABLE=1"
)
set_source_files_properties(${SDLPoP_MAIN} PROPERTIES
    COMPILE_OPTIONS "-include${CMAKE_CURRENT_LIST_DIR}/src/rp2350_alloc_trace.h"
    COMPILE_DEFINITIONS "main=sdlpop_entry;RP2350_ALLOC_TRACE_ENABLE=1"
)
target_include_directories(sdlpop PUBLIC
    third_party/SDLPoP/src
    src/fatfs
    drivers
)

if(USE_REAL_SDL2)
    set(FRANK_PRINCE_SDL2_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/SDL2")
    if(NOT EXISTS "${FRANK_PRINCE_SDL2_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "USE_REAL_SDL2=ON, but SDL2 sources were not found at ${FRANK_PRINCE_SDL2_DIR}.\n"
            "Add SDL2 there (e.g. as a git submodule), then re-configure.\n"
            "Example: git submodule add https://github.com/libsdl-org/SDL.git third_party/SDL2")
    endif()

    # Prefer a static build for bare-metal targets.
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)

    add_subdirectory(third_party/SDL2 EXCLUDE_FROM_ALL)

    # Try to find a usable SDL2 target name across SDL2 CMake variants.
    # Prefer the *real* build target (non-alias) so we can add overlay sources.
    if(TARGET SDL2-static)
        set(FRANK_PRINCE_SDL2_TARGET SDL2-static)
    elseif(TARGET SDL2)
        set(FRANK_PRINCE_SDL2_TARGET SDL2)
    elseif(TARGET SDL2::SDL2-static)
        set(FRANK_PRINCE_SDL2_TARGET SDL2::SDL2-static)
    elseif(TARGET SDL2::SDL2)
        set(FRANK_PRINCE_SDL2_TARGET SDL2::SDL2)
    else()
        message(FATAL_ERROR "SDL2 was added, but no known SDL2 CMake target was found")
    endif()

    # Overlay (sketch) RP2350 driver sources into SDL2 once it’s present.
    # These currently compile only inside SDL2 (`SDL_INTERNAL_BUILD`).
    # If `FRANK_PRINCE_SDL2_TARGET` is an ALIAS target in your SDL2 version,
    # CMake will error on target_sources(); in that case we’ll switch to the
    # underlying concrete target after we vendor SDL2.
    if(TARGET ${FRANK_PRINCE_SDL2_TARGET})
        target_sources(${FRANK_PRINCE_SDL2_TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/src/sdl2_rp2350_backend/rp2350_video_driver.c
            ${CMAKE_CURRENT_LIST_DIR}/src/sdl2_rp2350_backend/rp2350_events.c
            ${CMAKE_CURRENT_LIST_DIR}/src/sdl2_rp2350_backend/rp2350_platform_glue.c
        )
        target_include_directories(${FRANK_PRINCE_SDL2_TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/src/sdl2_rp2350_backend
            ${CMAKE_CURRENT_LIST_DIR}/drivers
            ${CMAKE_CURRENT_LIST_DIR}/src
        )
    endif()

    target_link_libraries(sdlpop PRIVATE ${FRANK_PRINCE_SDL2_TARGET})

    # Keep firmware/port-only headers in a separate include root so we can
    # avoid shadowing upstream SDL headers.
    target_include_directories(sdlpop PUBLIC src/compat)

    # Provide a minimal SDL2_image API surface implemented via stb_image.
    target_include_directories(sdlpop PUBLIC src/sdl2_image_stub)
else()
    # SDL shim headers live under `src/SDL2` and are found via the `src` include.
    # (Do not add `src/compat` here, because it could shadow shim SDL headers.)
    target_include_directories(sdlpop PUBLIC src src/SDL2)
endif()
target_compile_definitions(sdlpop PUBLIC POP_RP2350)
target_compile_options(sdlpop PRIVATE -Ofast)
target_link_libraries(sdlpop PRIVATE pico_stdlib hardware_interp)

add_library(drivers
    drivers/HDMI.c
    drivers/psram_init.c
    drivers/psram_allocator.c
)

target_include_directories(drivers PUBLIC drivers src)

target_compile_definitions(drivers PRIVATE
    BOARD_${BOARD_VARIANT}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
)

target_compile_options(drivers PRIVATE -Ofast)

target_link_libraries(drivers pico_stdlib hardware_dma hardware_pio hardware_spi hardware_clocks)

add_executable(frank-prince
    src/main.c
    src/pop_fs.c
    src/rp2350_alloc_trace.c
    src/start_screen.c
)

if(USE_REAL_SDL2)
    # SDL2_image stub uses stb_image.
    target_sources(frank-prince PRIVATE src/stb_image_impl.c)
    target_sources(frank-prince PRIVATE src/sdl2_image_stub/SDL_image_stub.c)
else()
    add_library(rp_sdl STATIC
        src/SDL_port.c
        src/stb_image_impl.c
    )

    target_include_directories(rp_sdl PUBLIC
        src
        src/SDL2
        src/fatfs
        drivers
        drivers/audio
    )

    target_compile_definitions(rp_sdl PUBLIC
        POP_RP2350
        BOARD_${BOARD_VARIANT}
        RP_SDL_FEATURE_WINDOW=0
        RP_SDL_FEATURE_MESSAGEBOX=0
        RP_SDL_FEATURE_AUDIO=1
        RP_SDL_FEATURE_JOYSTICK=0
        RP_SDL_FEATURE_GAMECONTROLLER=0
        RP_SDL_FEATURE_HAPTIC=0
    )

    # Board-specific SD card pin definitions
    if(BOARD_VARIANT STREQUAL "M1")
        target_compile_definitions(rp_sdl PUBLIC
            SDCARD_PIN_SPI0_SCK=2
            SDCARD_PIN_SPI0_MOSI=3
            SDCARD_PIN_SPI0_MISO=4
            SDCARD_PIN_SPI0_CS=5
        )
    elseif(BOARD_VARIANT STREQUAL "M2")
        target_compile_definitions(rp_sdl PUBLIC
            SDCARD_PIN_SPI0_MISO=4
            SDCARD_PIN_SPI0_CS=5
            SDCARD_PIN_SPI0_SCK=6
            SDCARD_PIN_SPI0_MOSI=7
        )
    endif()

    target_compile_options(rp_sdl PRIVATE -Ofast)
    # SDL_port includes HDMI headers which depend on Pico SDK hardware headers
    # (e.g. hardware/dma.h). Pull these in explicitly so rp_sdl compiles.
    target_link_libraries(rp_sdl PUBLIC
        pico_stdlib
        drivers
        hardware_dma
        hardware_pio
        hardware_spi
        hardware_clocks
        audio_driver
        usbhid
    )

    # Generate PIO header for I2S audio driver
    pico_generate_pio_header(rp_sdl ${AUDIO_I2S_PIO_FILE})

    # Make SDLPoP explicitly depend on the rp_sdl shim.
    target_link_libraries(sdlpop PRIVATE rp_sdl)
endif()

target_include_directories(frank-prince PRIVATE src drivers)

target_compile_options(frank-prince PRIVATE -Ofast)

# Inject version.txt as a compile-time string (zero-padded: "1 3" -> "1.03").
file(READ "${CMAKE_CURRENT_LIST_DIR}/version.txt" FRANK_PRINCE_VERSION_RAW)
string(STRIP "${FRANK_PRINCE_VERSION_RAW}" FRANK_PRINCE_VERSION_RAW)
string(REGEX MATCH "^([0-9]+)[ \t]+([0-9]+)$" _VER_MATCH "${FRANK_PRINCE_VERSION_RAW}")
set(_VER_MAJOR "${CMAKE_MATCH_1}")
set(_VER_MINOR "${CMAKE_MATCH_2}")
if(_VER_MINOR LESS 10)
    set(FRANK_PRINCE_VERSION "${_VER_MAJOR}.0${_VER_MINOR}")
else()
    set(FRANK_PRINCE_VERSION "${_VER_MAJOR}.${_VER_MINOR}")
endif()

target_compile_definitions(frank-prince PRIVATE
    BOARD_${BOARD_VARIANT}
    CPU_CLOCK_MHZ=${CPU_SPEED}
    CPU_VOLTAGE=${CPU_VOLTAGE}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
    DBOARD_VARIANT="${BOARD_VARIANT}"
    DCPU_SPEED=${CPU_SPEED}
    DPSRAM_SPEED=${PSRAM_SPEED}
    FRANK_PRINCE_VERSION="${FRANK_PRINCE_VERSION}"
    POP_RP2350
    RP2350_FORCE_TEST_PATTERN=${RP2350_FORCE_TEST_PATTERN}
    RP2350_DUMP_FIRST_FRAME_BYTES=${RP2350_DUMP_FIRST_FRAME_BYTES}
    RP2350_DEBUG_INDEX_BAR=${RP2350_DEBUG_INDEX_BAR}
    RP2350_BOOT_TEST_PATTERN=${RP2350_BOOT_TEST_PATTERN}
    RP2350_BOOT_TEST_PATTERN_HALT=${RP2350_BOOT_TEST_PATTERN_HALT}
    RP2350_BOOT_TEST_PATTERN_MODE=${RP2350_BOOT_TEST_PATTERN_MODE}
)

target_link_libraries(frank-prince pico_stdlib hardware_vreg hardware_clocks hardware_flash hardware_sync drivers sdcard ps2kbd usbhid sdlpop)

if(NOT USE_REAL_SDL2)
    target_link_libraries(frank-prince rp_sdl)
endif()

# USB HID Host and USB CDC stdio are mutually exclusive
if(USB_HID_ENABLED)
    pico_enable_stdio_usb(frank-prince 0)
else()
    pico_enable_stdio_usb(frank-prince 1)
endif()
pico_enable_stdio_uart(frank-prince 0)

pico_add_extra_outputs(frank-prince)
