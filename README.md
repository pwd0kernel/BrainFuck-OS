# BrainfuckOS 🧠

A playful Arduino experiment implementing a multi-tasking Brainfuck environment with an EEPROM-based filesystem. While not an actual operating system, it provides OS-like features for managing and running Brainfuck programs on Arduino hardware.

## Features 🚀

- **Multi-tasking Brainfuck interpreter**
  - Supports up to 2 concurrent tasks
  - Round-robin task scheduling
  - Task management commands (NEW, PS, KILL)

- **EEPROM-based "filesystem"**
  - 4 storage slots for Brainfuck programs
  - File operations: SAVE, LOAD, RUN, LS
  - Named program storage

- **Extended Brainfuck Implementation**
  - Traditional Brainfuck commands (`><+-.,[]`)
  - Hardware control extensions:
    - `^` - Toggle LED
    - `#` - Digital write
    - `~` - Digital read

- **Interactive Shell**
  - Command-line interface over serial
  - GPIO control commands (PINMODE, DWRITE, DREAD)
  - Memory and task inspection

## Hardware Requirements 💾

- Arduino Uno or compatible board
- USB connection for serial interface
- Optional: LEDs and components for GPIO experiments

## Getting Started 🏁

1. Clone this repository
2. Open `BrainfuckOS.ino` in Arduino IDE
3. Upload to your Arduino
4. Connect via serial monitor at 9600 baud
5. Type `HELP` to see available commands

## Shell Commands 💻

```
NEW <bf>       Create BF task
PS             List tasks
KILL <id>      Kill task
LS             List EEPROM slots
SAVE <s> <nm>  Save last BF code
LOAD <s>       Load BF code from slot
RUN <s>        Same as LOAD
PINMODE <p> <IN/OUT>
DWRITE <p> <0/1>
DREAD <p>
MEM            Show memory info
HELP           Print help
```

## Memory Optimizations 🔧

- Struct packing for efficient memory usage
- Smaller buffers to fit Arduino constraints
- PROGMEM string storage
- Optimized instruction set

## Example Usage 📝

```bash
# Create a new task that blinks the LED
> NEW ^^^^^^^^
Task #0 with 8 BFS instr created.

# Save it to slot 0 with name "blink"
> SAVE 0 blink
Saved code to slot 0

# List running tasks
> PS
Task 0: READY pc=3/8

# Check EEPROM storage
> LS
Slot 0 'blink' len=8
```

## License 📄

Licensed under MIT License. See LICENSE file for details.

## Contributing 🤝

Contributions are welcome! Feel free to submit issues and pull requests.

---

**Note**: This is not an actual operating system, but rather an entertaining demonstration of implementing OS-like features in an Arduino environment using the Brainfuck programming language.
