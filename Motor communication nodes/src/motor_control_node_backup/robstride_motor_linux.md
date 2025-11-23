
# RobStrideMotorLinux – Python Driver Documentation

This document describes the Python driver implemented in `robstride_motor_linux.py`, which is a translation of the original C++ `RobStrideMotor` class based on Linux raw CAN sockets (PF_CAN, CAN_RAW).

The driver is designed to control RobStride actuators via `socketcan` (e.g., `can0`), supporting multiple control modes:

- Motion control mode (position + velocity + gains in one frame)
- Speed mode (PP speed control)
- Position mode (PP)
- Current mode (Iq/Id)
- Position mode (CSP)
- Zeroing and CAN ID configuration
- Reading feedback (position, velocity, torque, temperature)

---

## 1. Data Structures

### 1.1 `DataReadWriteOne`

```python
class DataReadWriteOne:
    def __init__(self, index: int):
        self.index = index
        self.data = 0.0
```

**Purpose**  
Represents a single parameter that can be read/written via index in the motor’s internal parameter table.

**Attributes**

- `index` (`int`): The parameter index (e.g., `0x7005` for run mode).
- `data` (`float` or `int`): The last read or set value associated with this index.

---

### 1.2 `DataReadWrite`

```python
class DataReadWrite:
    def __init__(self, index_list=INDEX_LIST):
        self.run_mode      = DataReadWriteOne(index_list[0])
        self.iq_ref        = DataReadWriteOne(index_list[1])
        self.spd_ref       = DataReadWriteOne(index_list[2])
        self.imit_torque   = DataReadWriteOne(index_list[3])
        self.cur_kp        = DataReadWriteOne(index_list[4])
        self.cur_ki        = DataReadWriteOne(index_list[5])
        self.cur_filt_gain = DataReadWriteOne(index_list[6])
        self.loc_ref       = DataReadWriteOne(index_list[7])
        self.limit_spd     = DataReadWriteOne(index_list[8])
        self.limit_cur     = DataReadWriteOne(index_list[9])
        self.mechPos       = DataReadWriteOne(index_list[10])
        self.iqf           = DataReadWriteOne(index_list[11])
        self.mechVel       = DataReadWriteOne(index_list[12])
        self.VBUS          = DataReadWriteOne(index_list[13])
        self.rotation      = DataReadWriteOne(index_list[14])
```

**Purpose**  
Holds a structured collection of all relevant parameters (both writable and read-only) used by the motor for control and feedback.

**Main fields** (each is a `DataReadWriteOne`):

- `run_mode`: Control mode (0: motion, 1: PP, 2: speed, 3: current, 4: zero, 5: CSP).
- `iq_ref`: Iq reference (current command).
- `spd_ref`: Speed reference.
- `imit_torque`: Torque limit.
- `cur_kp`, `cur_ki`, `cur_filt_gain`: Current loop PID and filter parameters.
- `loc_ref`: Position reference.
- `limit_spd`: Speed limit.
- `limit_cur`: Current limit.
- `mechPos`: Mechanical position feedback.
- `iqf`: Filtered Iq feedback.
- `mechVel`: Mechanical velocity feedback.
- `VBUS`: DC bus voltage.
- `rotation`: Rotation count.

---

### 1.3 `MotorSet`

```python
class MotorSet:
    def __init__(self):
        self.set_motor_mode = 0
        self.set_current    = 0.0
        self.set_speed      = 0.0
        self.set_Torque     = 0.0
        self.set_angle      = 0.0
        self.set_limit_cur  = 0.0
        self.set_Kp         = 0.0
        self.set_Ki         = 0.0
        self.set_Kd         = 0.0
        self.set_iq         = 0.0
        self.set_id         = 0.0
        self.set_acc        = 0.0
```

**Purpose**  
Stores the most recently requested values for various setpoints (speed, acceleration, angle, currents, gains). Used mainly for consistency and internal bookkeeping.

**Typical usage**

- `set_speed`, `set_acc`, `set_angle` used in PP/CSP mode.
- `set_iq`, `set_id` used in current control mode.
- `set_motor_mode` may mirror the current control mode.

---

## 2. Main Class: `RobStrideMotorLinux`

```python
class RobStrideMotorLinux:
    def __init__(self, iface: str, master_id: int, motor_id: int, actuator_type: int):
        ...
```

### 2.1 Constructor: `__init__`

**Signature**

```python
RobStrideMotorLinux(iface: str, master_id: int, motor_id: int, actuator_type: int)
```

**Parameters**

- `iface` (`str`): Name of the CAN interface, e.g. `"can0"`.
- `master_id` (`int`): Host/master ID used in CAN ID composition (typically `0xFF`).
- `motor_id` (`int`): Motor CAN ID (e.g. `0x01`, `0xFD`).
- `actuator_type` (`int`): Actuator type index (0–6) used to pick motion limits from `ACTUATOR_OPERATION_MAPPING`.

**Behavior**

- Opens a raw CAN socket (`PF_CAN`, `SOCK_RAW`, `CAN_RAW`).
- Binds to the specified interface.
- Sets a CAN filter to only receive frames for this `motor_id` with extended IDs.
- Initializes internal feedback fields (`position`, `velocity`, `torque`, `temperature`).
- Creates `DataReadWrite` and `MotorSet` instances.

---

### 2.2 `_init_socket`

**Signature**

```python
def _init_socket(self) -> None
```

**Purpose**  
Internal helper to open the CAN socket and configure a filter identical to the C++ version.

**Behavior**

- Creates a `socket.socket(PF_CAN, SOCK_RAW, CAN_RAW)`.
- Binds to `(self.iface,)`.
- Applies a `can_filter` that matches:
  - Extended frames (`CAN_EFF_FLAG`), and
  - Bits 8–15 of CAN ID equal to `motor_id`.

**Parameters / Return**  
No external parameters; no return value.

---

### 2.3 Static Helpers: `float_to_uint`, `uint_to_float`, `byte_to_float_from_payload`

```python
@staticmethod
def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int
```

- Maps a float in `[x_min, x_max]` to an integer in `[0, 2^bits - 1]`.
- Used for encoding physical quantities (position, velocity, torque) into 16-bit integers.

```python
@staticmethod
def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float
```

- Inverse of `float_to_uint`: maps an integer back to a float in `[x_min, x_max]`.

```python
@staticmethod
def byte_to_float_from_payload(payload: bytes) -> float
```

- Interprets `payload[4..7]` as a 32-bit float (little-endian), matching the C++ `Byte_to_float` function.
- Returns `0.0` if payload length < 8.

---

### 2.4 Low-level CAN I/O: `_send_frame`, `_recv_frame`

```python
def _send_frame(self, can_id_no_flag: int, data: bytes) -> None
```

**Parameters**

- `can_id_no_flag` (`int`): 29-bit CAN ID without `CAN_EFF_FLAG` (the method adds the flag).
- `data` (`bytes`): Payload up to 8 bytes (will be padded to 8).

**Behavior**

- Packs a `struct can_frame` (`CAN_FRAME_FMT`).
- Sends it through the raw CAN socket.

---

```python
def _recv_frame(self, timeout: float = 0.1) -> Optional[Tuple[int, int, bytes]]
```

**Parameters**

- `timeout` (`float`): Receive timeout in seconds; if <= 0, blocking mode is used.

**Returns**

- `None` on timeout.
- `(can_id_no_flag, dlc, data_bytes)` if a valid extended frame is received.

**Notes**

- Only extended frames are accepted; non-extended frames are ignored.

---

### 2.5 `receive_status_frame`

```python
def receive_status_frame(self, timeout: float = 0.1) -> None
```

**Purpose**  
Reads one CAN frame, decodes it as a status or parameter frame, and updates internal state.

**Behavior**

- Calls `_recv_frame()`.
- Extracts:
  - `communication_type` (bits 24–28).
  - `extra_data`, `host_id`.
  - `error_code` (bits 16–21).
  - `pattern` (bits 22–23).
- If `communication_type == MotorRequest (0x02)`:
  - Decodes position, velocity, torque, temperature from data bytes.
  - Stores them in `self.position`, `self.velocity`, `self.torque`, `self.temperature`.
- If `communication_type == GetSingleParameter (0x11)`:
  - Uses the index in data[0..1] to update corresponding fields in `self.drw`.

**Exceptions**

- Raises `RuntimeError` if no frame is received or data length < 8.
- Raises `RuntimeError` for unsupported `communication_type`.

---

### 2.6 `set_motor_parameter`

```python
def set_motor_parameter(self, index: int, value: float, value_mode: str) -> None
```

**Purpose**  
Writes a single parameter to the motor (index-based).

**Parameters**

- `index` (`int`): Parameter index (e.g., `0x7005` for run mode, `0x700A` for speed command).
- `value` (`float`): Value to write, interpreted depending on `value_mode`.
- `value_mode` (`str`):
  - `'p'`: write `value` as a 32-bit float to data[4..7].
  - `'j'`: write `value` as a uint8 to data[4], others zero; used for mode selection.

**Behavior**

- Composes CAN ID: `SetSingleParameter << 24 | master_id << 8 | motor_id`.
- Fills payload accordingly.
- Sends frame via `_send_frame`.
- Calls `receive_status_frame()` to process the response.

---

### 2.7 `get_motor_parameter`

```python
def get_motor_parameter(self, index: int) -> None
```

**Purpose**  
Requests a single parameter from the motor.

**Parameters**

- `index` (`int`): Parameter index to query.

**Behavior**

- Sends a `GetSingleParameter` frame with the given index.
- Calls `receive_status_frame()`, which will update `self.drw` based on the response.

---

### 2.8 `enable_motor` / `disable_motor`

```python
def enable_motor(self) -> Tuple[float, float, float, float]
```

**Purpose**  
Sends the “motor enable” command and reads one status frame.

**Behavior**

- CAN ID: `MotorEnable << 24 | master_id << 8 | motor_id`.
- Payload: 8 zero bytes.
- Calls `receive_status_frame()` to refresh feedback.

**Returns**

- `(position, velocity, torque, temperature)` after the command.

---

```python
def disable_motor(self, clear_error: int = 0) -> Tuple[float, float, float, float]
```

**Purpose**  
Sends the “motor stop/disable” command and reads one status frame.

**Parameters**

- `clear_error` (`int`): Written to `data[0]`; used to clear errors if non-zero.

**Behavior**

- CAN ID: `MotorStop << 24 | master_id << 8 | motor_id`.
- Payload: `[clear_error, 0, 0, 0, 0, 0, 0, 0]`.

**Returns**

- `(position, velocity, torque, temperature)` after the command.

---

### 2.9 `ensure_mode`

```python
def ensure_mode(self, target_mode: int) -> None
```

**Purpose**  
Makes sure the motor is in a specific control mode (e.g. speed, PP, CSP, current).

**Parameters**

- `target_mode` (`int`): Desired mode (e.g. `Speed_control_mode`, `PosPP_control_mode`, etc.).

**Behavior**

- Calls `get_motor_parameter(0x7005)` to update `self.drw.run_mode`.
- If current mode != `target_mode` and `self.pattern == 2`:
  - `disable_motor(0)`
  - `set_motor_parameter(0x7005, target_mode, 'j')`
  - `get_motor_parameter(0x7005)`
  - `enable_motor()`

**Notes**

- Mirrors the mode-switching logic in the C++ code.

---

### 2.10 `send_motion_command`

```python
def send_motion_command(
    self,
    torque: float,
    position_rad: float,
    velocity_rad_s: float,
    kp: float = 0.5,
    kd: float = 0.1,
) -> Tuple[float, float, float, float]
```

**Purpose**  
Sends a motion control command where torque is encoded into the CAN ID and position/velocity/gains into payload.

**Parameters**

- `torque` (`float`): Desired torque (Nm), limited by actuator-specific `torque` range.
- `position_rad` (`float`): Target position (rad), limited by `position` range.
- `velocity_rad_s` (`float`): Target velocity (rad/s), limited by `velocity` range.
- `kp`, `kd` (`float`): Position/velocity gains, limited by corresponding `kp` and `kd` ranges.

**Behavior**

- Converts each value to 16-bit integers using `float_to_uint`.
- CAN ID: `MotionControl << 24 | torque_u16 << 8 | motor_id`.
- Payload: `[pos_u16, vel_u16, kp_u16, kd_u16]`.
- Sends frame and calls `receive_status_frame()`.

**Returns**

- `(position, velocity, torque, temperature)` feedback.

---

### 2.11 `send_velocity_mode_command`

```python
def send_velocity_mode_command(
    self,
    velocity_rad_s: float,
    acceleration_rad_s2: float = None,
) -> Tuple[float, float, float, float]
```

**Purpose**  
High-level API for speed mode, mirroring the full C++ logic for `send_velocity_mode_command`.

**Parameters**

- `velocity_rad_s` (`float`): Target speed command (rad/s).
- `acceleration_rad_s2` (`float`, optional): Acceleration setting for index `0x7026`. If `None`, uses stored `self.motor_set_all.set_acc`.

**Behavior**

1. Updates `self.motor_set_all.set_acc` if provided.
2. Calls `ensure_mode(Speed_control_mode)` to switch to speed mode.
3. Writes:
   - `0x7018` = `27.0` (speed limit).
   - `0x7026` = `set_acc` (if non-zero).
4. Writes actual speed command:
   - `0x700A` = `velocity_rad_s`.
5. Calls `receive_status_frame()` inside `set_motor_parameter()`.

**Returns**

- `(position, velocity, torque, temperature)` feedback.

---

### 2.12 `pos_pp_control` (Position Mode PP)

```python
def pos_pp_control(
    self,
    speed_rad_s: float,
    acceleration_rad_s2: float,
    angle_rad: float,
) -> Tuple[float, float, float, float]
```

**Purpose**  
Implements position control in PP mode (`PosPP_control_mode`).

**Parameters**

- `speed_rad_s` (`float`): Speed setting for PP mode (index `0x7025`).
- `acceleration_rad_s2` (`float`): Acceleration setting (index `0x7026`).
- `angle_rad` (`float`): Target position (index `0x7016`), in radians.

**Behavior**

- Stores values in `motor_set_all`.
- Calls `ensure_mode(PosPP_control_mode)`.
- Writes:
  - `0x7025` = `set_speed`.
  - `0x7026` = `set_acc`.
  - `0x7016` = `set_angle`.

**Returns**

- `(position, velocity, torque, temperature)` feedback.

---

### 2.13 `current_control` (Current Mode Iq/Id)

```python
def current_control(
    self,
    iq_command: float,
    id_command: float,
) -> Tuple[float, float, float, float]
```

**Purpose**  
Implements current control mode (`Elect_control_mode`), controlling Iq/Id.

**Parameters**

- `iq_command` (`float`): Requested Iq current (A), in `[-23, 23]` range.
- `id_command` (`float`): Requested Id current (A); typically 0 for most applications.

**Behavior**

- Stores `iq_command`, `id_command` in `motor_set_all`.
- Calls `ensure_mode(Elect_control_mode)`.
- Encodes Iq into a 16-bit integer via `float_to_uint` with range `[SCIQ_MIN, SC_MAX] = [-23, 23]`.
- Writes:
  - `0x7006` = Iq (encoded as float of that u16 value, matching the C++ behavior).
  - `0x7007` = Id (float).

**Returns**

- `(position, velocity, torque, temperature)` feedback.

**Note**  
Be very careful with the magnitude of `iq_command`, especially on first tests.

---

### 2.14 `pos_csp_control` (Position Mode CSP)

```python
def pos_csp_control(
    self,
    speed_rad_s: float,
    angle_rad: float,
) -> Tuple[float, float, float, float]
```

**Purpose**  
Implements CSP (Cyclic Synchronous Position) mode (`PosCSP_control_mode`).

**Parameters**

- `speed_rad_s` (`float`): CSP speed limit, mapped to index `0x7017`.
- `angle_rad` (`float`): CSP target position (index `0x7016`).

**Behavior**

- Stores values in `motor_set_all`.
- Calls `ensure_mode(PosCSP_control_mode)`.
- Converts `speed_rad_s` to 16-bit using velocity limits of the selected actuator.
- Writes:
  - `0x7017` = encoded speed.
  - `0x7016` = target angle.

**Returns**

- `(position, velocity, torque, temperature)` feedback.

---

### 2.15 `set_zero_mode`

```python
def set_zero_mode(self) -> None
```

**Purpose**  
Sets the control mode to “zero mode” (`Set_Zero_mode`) without sending the SetPosZero command.

**Behavior**

- Writes `Set_Zero_mode` to index `0x7005` using mode write (`'j'`).
- Typically followed by `set_zero_position()`.

---

### 2.16 `set_zero_position`

```python
def set_zero_position(self) -> None
```

**Purpose**  
Sends the special `SetPosZero` frame to store the current mechanical position as zero in the motor firmware.

**Behavior**

- `disable_motor(0)`.
- Reads current mode (`0x7005`); if not zero mode, sets it to speed mode, as in C++.
- Sends frame:
  - CAN ID: `SetPosZero << 24 | master_id << 8 | motor_id`.
  - Payload: first byte = 1, others = 0.
- Calls `enable_motor()` after sending.

---

### 2.17 `read_initial_position`

```python
def read_initial_position(self, timeout_sec: float = 10.0) -> float
```

**Purpose**  
Waits for a MotorRequest frame and decodes the position as the “initial position”.

**Parameters**

- `timeout_sec` (`float`): Maximum time to wait (seconds).

**Behavior**

- Loops until timeout:
  - Calls `_recv_frame(timeout=0.1)`.
  - Extracts `type_field`, `mid`, `eid`.
  - If `type_field == MotorRequest (0x02)` and `mid == motor_id`, decodes position from data[0..1] using a fixed range `[-4π, 4π]`.
- Prints debug information.

**Returns**

- The decoded position (rad) if successful.
- `0.0` if timeout occurs.

---

### 2.18 `set_can_id`

```python
def set_can_id(self, new_can_id: int) -> None
```

**Purpose**  
Sends a command to change the motor’s CAN ID in firmware.

**Parameters**

- `new_can_id` (`int`): New motor ID (0–255).

**Behavior**

- `disable_motor(0)`.
- Sends frame:
  - CAN ID: `Can_ID << 24 | new_can_id << 16 | master_id << 8 | motor_id`.
  - Payload: zeros.
- Does **not** update `self.motor_id` or reconfigure the socket’s CAN filter.

**Important Note**

- After this call, you must create a **new** `RobStrideMotorLinux` instance with `motor_id=new_can_id` to continue communicating with the motor.
