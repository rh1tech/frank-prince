# FRANK Prince

Official page: **[frank.rh1.tech](https://frank.rh1.tech/)** — hub for all FRANK boards and firmware.

Prince of Persia for Raspberry Pi Pico 2 (RP2350) with HDMI output, SD card, and PS/2 keyboard.

Based on [SDLPoP](https://github.com/NagyD/SDLPoP) — an open-source port of Prince of Persia.

## Supported Boards

This firmware is designed for the following RP2350-based boards with integrated HDMI, SD card, PS/2, and PSRAM:

- **[FRANK](https://rh1.tech/projects/frank?area=about)** — A versatile development board based on RP Pico 2, HDMI output, and extensive I/O options.
- **[Murmulator](https://murmulator.ru)** — A compact retro-computing platform based on RP Pico 2, designed for emulators and classic games.

Both boards provide all necessary peripherals out of the box—no additional wiring required.

## Features

- Native 320×200 HDMI video output via PIO
- 8MB QSPI PSRAM support for game data
- SD card support for game resources and saved games
- PS/2 and USB keyboard input
- I2S audio output

## Screenshots

![Gameplay Screenshot 1](screenshots/screen1.png)
![Gameplay Screenshot 2](screenshots/screen2.png)
![Gameplay Screenshot 3](screenshots/screen3.png)
![Gameplay Screenshot 4](screenshots/scren4.png)

## Hardware Requirements

- **Raspberry Pi Pico 2** (RP2350) or compatible board
- **8MB QSPI PSRAM** (mandatory!)
- **HDMI connector** (directly connected via resistors, no HDMI encoder needed)
- **SD card module** (SPI mode)
- **PS/2 or USB keyboard**
- **I2S DAC module** (e.g., TDA1387) for audio output

### PSRAM Options

FRANK Prince requires 8MB PSRAM to run. You can obtain PSRAM-equipped hardware in several ways:

1. **Solder a PSRAM chip** on top of the Flash chip on a Pico 2 clone (SOP-8 flash chips are only available on clones, not the original Pico 2)
2. **Build a [Nyx 2](https://rh1.tech/projects/nyx?area=nyx2)** — a DIY RP2350 board with integrated PSRAM
3. **Purchase a [Pimoroni Pico Plus 2](https://shop.pimoroni.com/products/pimoroni-pico-plus-2?variant=42092668289107)** — a ready-made Pico 2 with 8MB PSRAM

## Board Configurations

Two GPIO layouts are supported: **M1** and **M2**. The PSRAM pin is auto-detected based on chip package:
- **RP2350B**: GPIO47 (both M1 and M2)
- **RP2350A**: GPIO19 (M1) or GPIO8 (M2)

### HDMI (via 270Ω resistors)
| Signal | M1 GPIO | M2 GPIO |
|--------|---------|---------|
| CLK-   | 6       | 12      |
| CLK+   | 7       | 13      |
| D0-    | 8       | 14      |
| D0+    | 9       | 15      |
| D1-    | 10      | 16      |
| D1+    | 11      | 17      |
| D2-    | 12      | 18      |
| D2+    | 13      | 19      |

### SD Card (SPI mode)
| Signal  | M1 GPIO | M2 GPIO |
|---------|---------|---------|
| CLK     | 2       | 6       |
| CMD     | 3       | 7       |
| DAT0    | 4       | 4       |
| DAT3/CS | 5       | 5       |

### PS/2 Keyboard
| Signal | M1 GPIO | M2 GPIO |
|--------|---------|---------|
| CLK    | 0       | 2       |
| DATA   | 1       | 3       |

### I2S Audio
| Signal | M1 GPIO | M2 GPIO |
|--------|---------|---------|
| DATA   | 26      | 9       |
| BCLK   | 27      | 10      |
| LRCLK  | 28      | 11      |

## Building

### Prerequisites

1. Install the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (version 2.0+)
2. Set environment variable: `export PICO_SDK_PATH=/path/to/pico-sdk`
3. Install ARM GCC toolchain

### Build Steps

```bash
# Clone the repository with submodules
git clone --recursive https://github.com/rh1tech/frank-prince.git
cd frank-prince

# Or if already cloned, initialize submodules
git submodule update --init --recursive

# Build using the build script (recommended)
./build.sh                          # Build M1 @ 378 MHz (default)
./build.sh --board M2               # Build for M2 layout
./build.sh --board M1 --cpu 504     # Build M1 @ 504 MHz
./build.sh clean --board M2         # Clean rebuild for M2

# Or build manually with CMake
mkdir build && cd build
cmake -DBOARD_VARIANT=M1 -DCPU_SPEED=378 -DPSRAM_SPEED=133 ..
make -j$(nproc)
```

### Build Options

| Option | Description |
|--------|-------------|
| `--board M1` or `-b M1` | Use M1 GPIO layout (default) |
| `--board M2` or `-b M2` | Use M2 GPIO layout |
| `--cpu 252\|378\|504` | CPU speed in MHz (default: 378) |
| `--psram 100\|133\|166` | PSRAM speed in MHz (auto-selected) |
| `clean` | Delete build directory and rebuild |

Speed presets:
- 252 MHz CPU → 100 MHz PSRAM (no overclock)
- 378 MHz CPU → 133 MHz PSRAM (medium overclock)
- 504 MHz CPU → 166 MHz PSRAM (max overclock)

### Release Builds

To build all 6 variants (M1/M2 × 3 speeds) with version numbering:

```bash
./release.sh              # Interactive version prompt
./release.sh 1.04         # Use version 1.04 from command line
```

This creates versioned UF2 files in the `release/` directory:
- `frank-prince_m1_252_100_X_XX.uf2`
- `frank-prince_m1_378_133_X_XX.uf2`
- `frank-prince_m1_504_166_X_XX.uf2`
- `frank-prince_m2_252_100_X_XX.uf2`
- `frank-prince_m2_378_133_X_XX.uf2`
- `frank-prince_m2_504_166_X_XX.uf2`

### Flashing

```bash
# With device in BOOTSEL mode:
picotool load build-make/frank-prince.uf2

# Or with device running:
picotool load -f build-make/frank-prince.uf2

# Or use the flash script:
./flash.sh
```

## SD Card Setup

1. Format an SD card as FAT32
2. Download [prince.zip](https://github.com/rh1tech/frank-prince/raw/main/sdcard/prince.zip) and extract it
3. Copy the `prince` folder to the root of your SD card

### Upgrading from Version 1.00

When upgrading from version 1.00, copy the `prince/midi_cache` directory from the new archive to your SD card's `prince` folder.

**Note:** The MIDI cache contains pre-rendered audio for all MIDI music tracks (~62 MB). If the cache files are missing or outdated, they will be regenerated automatically during gameplay. Regeneration takes additional time during game loading.

## Controls

- **Arrow keys**: Move/Run/Climb
- **Shift**: Walk carefully / Pick up items
- **Up Arrow**: Jump (while running) / Climb up
- **Down Arrow**: Crouch / Climb down
- **Space**: Show remaining time
- **Escape**: Pause / Menu
- **Ctrl+A**: Restart level
- **Ctrl+R**: Quickload
- **Ctrl+Q**: Quit

### Title Screen

- **Alt+F1..F12**: Start at level 1-12
- **Alt+Shift+F1**: Start at level 13
- **Alt+Shift+F2**: Start at level 14

### Cheat Mode

- **Alt+Esc**: Toggle cheat mode on/off

When cheat mode is enabled:
- **Shift+L**: Skip to next level
- **K**: Kill current guard
- **R**: Resurrect kid
- **Shift+T**: Add max hit point (big potion)
- **Shift+S**: Restore hit point (small potion)
- **Shift+W**: Feather fall (slow falling)
- **Shift+I**: Flip screen upside down
- **Shift+B**: Toggle blind mode
- **+/-** (numpad): Add/subtract time
- **H/J/U/N**: View room left/right/up/down
- **Ctrl+B**: Return view to kid's room
- **C**: Show current room info
- **Shift+C**: Show adjacent room info

## License

GNU General Public License v3. See [LICENSE](LICENSE) for details.

This project is based on:
- [SDLPoP](https://github.com/NagyD/SDLPoP) by David Nagy — Open-source port of Prince of Persia

## Author

Mikhail Matveev <<xtreme@rh1.tech>>

## Acknowledgments

- Jordan Mechner for the original Prince of Persia
- David Nagy for the SDLPoP open-source port
- The Raspberry Pi foundation for the RP2350 and Pico SDK
