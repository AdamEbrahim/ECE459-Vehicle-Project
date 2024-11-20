# ESP32 Motor Control Code

This directory contains the ESP32 code for motor control.

## Command Protocol

Commands are sent as single bytes with the following format:

```
Byte format: SSSS CCCC
- SSSS (bits 7-4): Speed modifier
  - 0001: Slow speed
  - 0010: Normal speed
  - 0011: Custom speed (value in CCCC)
- CCCC (bits 3-0): Base command
  - 0000: STOP
  - 0001: FORWARD
  - 0010: LEFT
  - 0011: RIGHT
  - 0100: BACKWARD
```

## Example Commands
- `0x01`: Forward at normal speed
- `0x12`: Left turn at slow speed
- `0x35`: Forward at 50% speed (5/15 * 100%)
