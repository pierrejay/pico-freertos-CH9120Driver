# UartDmaDriver Test Suite

Complete validation test suite for UartDmaDriver functionality on real hardware.

## Hardware Setup Required

**Physical loopback connections:**

### RP2040 (Pico)
- TX0 (GPIO17) ↔ RX1 (GPIO4)
- TX1 (GPIO5) ↔ RX0 (GPIO16)

### RP2350 (Pico2) 
- TX0 (GPIO1) ↔ RX1 (GPIO9)
- TX1 (GPIO8) ↔ RX0 (GPIO0)

### Pico W / Pico2 W
- TX0 (GPIO12) ↔ RX1 (GPIO9)  
- TX1 (GPIO8) ↔ RX0 (GPIO13)

## Building and Running

**Prerequisites:** 
- FreeRTOS installed and operational (see driver README for setup instructions)
- Pico SDK configured for your target board
- The `/freertos` folder here contains only required configuration & hook files

**Method 1: VSCode Extension**
- Open the `/test` folder as a Pico project
- Cmd/Ctrl+Shift+P → "Raspberry Pi Pico: Switch Board" → Select your board
- Cmd/Ctrl+Shift+P → "Compile Pico Project" to build
- Cmd/Ctrl+Shift+P → "Run Pico Project (USB)" to flash the board

**Method 2: Command Line**
```bash
cd test/
rm -rf build/                    # Clean any previous configuration
mkdir build && cd build

# Configure for your board (specify Ninja generator)
cmake .. -G Ninja -DPICO_BOARD=pico2      # For Pico2
# cmake .. -G Ninja -DPICO_BOARD=pico     # For Pico
# cmake .. -G Ninja -DPICO_BOARD=pico2_w  # For Pico2 W
# cmake .. -G Ninja -DPICO_BOARD=pico_w   # For Pico W

ninja

# Flash uart_tests.uf2 to your board
# Connect via USB serial to see results
```

**Important:** Always clean the build directory when switching boards!

## Test Coverage

- **STEP 1: Driver Validation** - Multi-baud TX/RX with overflow recovery
- **STEP 2: Robustness Tests** - 400KB data integrity + API limits  
- **STEP 3: HARDCORE Stress** - Real-time performance under 80% CPU load

Expected result: `OVERALL: 9/9 tests passed` (~23 seconds execution)