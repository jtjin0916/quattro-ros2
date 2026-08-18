# Dual-CAN revision

- Teensy target fixed to **Teensy 4.0**.
- CAN A = FlexCAN `CAN1`, RX23/TX22, motor IDs 0..5.
- CAN B = FlexCAN `CAN2`, RX0/TX1, motor IDs 6..11.
- Both buses = 500 kbit/s.
- Added GIM6010 startup/config command helpers based on the preserved legacy real-robot driver.
- Added per-joint feedback validity/staleness to the USB serial protocol.
- Changed timeout policy so stale host commands can never preserve `tau_ff`.
- Updated ROS serial bridge to reject stale/incomplete full-robot feedback for `/joint_states`.
- Updated Linux virtual GIM6010 node to mirror two CAN interfaces (`vcan0`/`vcan1`).
- Added Arduino + dual MCP2515 hardware-in-the-loop GIM6010 emulator.
