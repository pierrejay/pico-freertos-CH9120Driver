# UartDmaDriver Examples

Educational code snippets demonstrating driver usage patterns.

## Examples

### `basic_uart.cpp`
**Minimal usage example**
- Driver initialization and lifecycle
- Event-driven data processing  
- Overflow recovery with best practices
- Error handling patterns

### `advanced_uart.cpp` 
**Protocol bridge** using FreeRTOS QueueSet for efficient multi-source event handling.
- Modbus ASCII ↔ RTU translation
- Cross-UART communication
- QueueSet "superloop" pattern
- Frame completion detection using silenceFlag

## Usage

These are **educational code snippets**, not standalone projects. To use them:

1. **Copy the code** into your main.cpp
2. **Adapt pin numbers** and baud rates for your hardware
3. **Add CMake configuration**:
   ```cmake
   add_subdirectory(UartDmaDriver)
   target_link_libraries(your_app UartDmaDriver)
   ```
4. **Build and flash** to your board

All examples include detailed comments explaining the implementation choices and best practices.