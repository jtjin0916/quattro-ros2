# Quattro firmware modes

## Current recommended test: Teensy internal virtual motor

`teensy_quattro_can/teensy_quattro_can.ino` defaults to:

```cpp
#define QUATTRO_VIRTUAL_MOTOR_MODE 1
```

No CAN transceiver is required in this mode.

The Teensy still performs the GIM6010 MIT protocol encode/decode path:

1. Receive `q_des, qdot_des, kp, kd, tau_ff` from ROS2 over USB Serial.
2. Pack a real GIM6010 MIT `0x008` 8-byte command frame.
3. Decode that packed frame inside the virtual actuator.
4. Simulate joint position, velocity and torque.
5. Pack a GIM6010-style `0x008` feedback frame (`id, position, velocity, torque`).
6. Run the same feedback parser used by physical CAN mode.
7. Return decoded feedback to ROS2 over USB Serial.

This intentionally omits only the CAN transceiver / CANH-CANL physical layer.

## Later physical dual-CAN mode

When two CAN transceivers are available, set:

```cpp
#define QUATTRO_VIRTUAL_MOTOR_MODE 0
```

Teensy 4.0 mapping used by the firmware:

- CAN1: TX 22 / RX 23 -> motors 0..5
- CAN2: TX 1 / RX 0 -> motors 6..11
- 500 kbit/s on both buses

The ROS2 serial protocol does not change when switching modes.

## Arduino emulator

The older `arduino_virtual_gim6010` example is retained only as an optional future HIL example for use with external CAN controller/transceiver hardware. It is not required for the current internal-virtual test path.
