# Quattro ROS2 + Teensy 4.0 dual-CAN refactor

## Target architecture

```text
Joystick / keyboard
        |
      Twist
        v
   quattro_sm
(timeout / E-stop / smoothing)
        |
   QuattroCmd
        v
quattro_commander
(gait / IK / limits / calibration)
        |
   JointCommand
(q_des, qdot_des, kp, kd, torque_ff)
        v
quattro_serial_bridge
        |
 USB Serial 921600
        v
     Teensy 4.0
  +----------------------+
  | CAN1 TX22 / RX23     | -> transceiver A -> IDs 0..5
  | CAN2 TX1  / RX0      | -> transceiver B -> IDs 6..11
  | MIT pack/unpack      |
  | 100 Hz actuator loop |
  | 300 ms watchdog      |
  +----------------------+
        |
Virtual or real GIM6010 motors
        |
MIT feedback: q, qdot, torque
        v
     Teensy
        |
   USB Serial
        v
quattro_serial_bridge
        |
 /joint_states
        v
robot_state_publisher -> RViz
```

## Teensy 4.0 CAN mapping

Using FlexCAN_T4 default pins:

| Bus | Teensy peripheral | RX | TX | Motor IDs |
|---|---|---:|---:|---|
| A | CAN1 | 23 | 22 | 0..5 |
| B | CAN2 | 0 | 1 | 6..11 |

Each peripheral requires a separate external CAN transceiver. Connect CAN TX to transceiver **TXD** and CAN RX to transceiver **RXD**; do not cross them like UART.

Both buses are 500 kbit/s.

## ROS messages

`quattro_msgs/msg/JointCommand.msg`

```text
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] kp
float64[] kd
float64[] torque_ff
```

Motor feedback is published using standard `sensor_msgs/msg/JointState`:

- `position` = output joint position [rad]
- `velocity` = output joint velocity [rad/s]
- `effort` = motor torque feedback [Nm]

## Watchdog behavior

If host JointCommand frames stop for 300 ms:

1. `torque_ff` is forced to zero.
2. `qdot_des` is forced to zero.
3. With fresh feedback, Teensy holds the current position.
4. Without fresh feedback, Teensy sends zero `kp/kd/tau` so stale feedback cannot become a driven hold command.

The serial feedback protocol also carries a per-joint validity flag. ROS does not publish a complete `/joint_states` message until all 12 motors have produced valid feedback, and stops publishing if any motor feedback becomes stale.

## Virtual motor choices

### A. Pure-PC test

`quattro.virtual_gim6010` emulates two SocketCAN buses:

- `vcan0`: IDs 0..5
- `vcan1`: IDs 6..11

Create virtual buses:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link add dev vcan1 type vcan
sudo ip link set up vcan0
sudo ip link set up vcan1
```

This is useful for testing MIT packing/parsing on Linux but does **not** pass through the Teensy electrical CAN path unless a physical CAN adapter is used.

### B. Arduino hardware-in-the-loop emulator

`firmware/arduino_virtual_gim6010/` contains a sketch that emulates the 12 motors with two MCP2515 CAN modules. This is the preferred no-motor validation because the command actually traverses CANH/CANL and the feedback returns to Teensy.

With only one MCP2515 module, emulate IDs 0..5 first.

## Real motor initialization

The Teensy firmware retains the initialization sequence from the legacy real-robot driver:

- clear errors
- set velocity/current limits
- select position control + MIT input mode
- request closed-loop state

The virtual emulator simply ignores setup/configuration frames and responds to MIT `0x008` commands.

## Legacy files

Failed/experimental MPC and the original SocketCAN motor driver remain under:

```text
src/quattro/quattro/legacy/
```

They are retained for analysis and are not part of the new normal actuator path.

---

## Teensy internal virtual actuator mode (current hardware-limited test)

Because a CAN transceiver is not currently available, the Teensy firmware now defaults to an internal virtual GIM6010 path:

```cpp
#define QUATTRO_VIRTUAL_MOTOR_MODE 1
```

Data path:

```text
ROS2 JointCommand
  -> USB Serial
  -> Teensy
  -> MIT 0x008 command pack (real packet format)
  -> internal virtual motor command decode
  -> virtual actuator dynamics
  -> MIT 0x008 feedback pack (real feedback format)
  -> normal MIT feedback parser
  -> USB Serial
  -> ROS2 sensor_msgs/JointState
  -> robot_state_publisher
  -> RViz
```

This validates the ROS/Serial interface and the GIM6010 MIT command/feedback encode/decode path while deliberately excluding only the CAN physical layer.

When dual transceivers are available, change the macro to `0`. The same ROS2 bridge and `JointCommand`/`JointState` interfaces remain unchanged; the firmware then uses Teensy 4.0 CAN1 (TX22/RX23, motors 0..5) and CAN2 (TX1/RX0, motors 6..11) at 500 kbit/s.


## Maintainer

- Taejin Jo
- jtjin0916@gmail.com
