# ESP CAN Communication Project

Python-based CAN (Controller Area Network) communication using the python-can library. Supports multiple hardware interfaces including SocketCAN, SLCAN, PEAK, Kvaser, and virtual testing interfaces.

## Installation

Install required dependencies:
```bash
pip install python-can
```

## Features

- Cross-platform CAN bus communication
- Support for multiple CAN interfaces
- CAN message transmission and reception
- Bus monitoring and statistics collection
- Message history tracking
- Extended and standard frame formats
- CAN FD support

## Supported Interfaces

| Interface | Platform | Notes |
|-----------|----------|-------|
| `socketcan` | Linux | Requires SocketCAN kernel module |
| `slcan` | Windows/Mac/Linux | Serial CAN adapter (e.g., FTDI) |
| `peak` | Windows/Mac/Linux | PEAK PCAN USB devices |
| `kvaser` | Windows/Mac/Linux | Kvaser CAN interfaces |
| `virtual` | All | For testing/loopback (default) |

## Usage

### Basic Example

```python
from main import initialize_can, send_can_message, receive_can_message

# Initialize CAN bus
if initialize_can():
    # Send a message
    send_can_message(can_id=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF])
    
    # Receive messages
    msg = receive_can_message(timeout=0.1)
```

### Monitor CAN Bus

```bash
python main.py  # Runs main loop with send/receive cycles
```

### Using can_utils Module

```python
from can_utils import CANConfig, CANBusMonitor, CANMessage
import can

# Create bus with configuration
config = CANConfig.get_config(interface='virtual', channel='vcan0', bitrate=500000)
bus = can.Bus(**config)

# Monitor activity
monitor = CANBusMonitor()

# Send message
msg = CANMessage(can_id=0x100, data=[0x11, 0x22, 0x33, 0x44])
bus.send(msg.to_can_message())
monitor.record_tx(msg.to_can_message())

# Receive message
rx_msg = bus.recv(timeout=1.0)
if rx_msg:
    monitor.record_rx(rx_msg)

# Print statistics
monitor.print_stats()
```

## Configuration

Edit `main.py` to change CAN interface settings:

```python
CAN_CONFIG = {
    'interface': 'virtual',      # Change to 'socketcan', 'slcan', etc.
    'channel': 'vcan0',          # Change to your channel
    'bitrate': 500000,           # 500 kbps
}
```

### Interface-Specific Configuration

#### SocketCAN (Linux)
```python
CAN_CONFIG = {
    'interface': 'socketcan',
    'channel': 'can0',
    'bitrate': 500000,
}
```

#### SLCAN (Serial)
```python
CAN_CONFIG = {
    'interface': 'slcan',
    'channel': 'COM3',           # Windows: COM3, Linux: /dev/ttyUSB0
    'bitrate': 500000,
}
```

#### Virtual (Testing)
```python
CAN_CONFIG = {
    'interface': 'virtual',
    'channel': 'vcan0',
    'bitrate': 500000,
}
```

## Pin Configuration

For hardware ESP32 connections:
- CAN TX GPIO: GPIO 5
- CAN RX GPIO: GPIO 4

Refer to your specific CAN transceiver documentation for proper connection.

## Hardware Requirements

- CAN transceiver module (e.g., SN65HVD230, MCP2551)
- Proper 120Ω termination resistors at both CAN bus ends
- Pull-up/pull-down resistors on TX/RX lines (if required by transceiver)

## Files

- `main.py` - Main CAN application using python-can
- `can_utils.py` - Utility classes and functions
- `pc_client.py` - PC-side client for monitoring (with pyserial)
- `requirements.txt` - Python package dependencies
- `README.md` - This file

## Dependencies

- **python-can** - Core CAN library
- **pyserial** - For serial-based interfaces (automatic with python-can)

## Examples

### Send Multiple Messages
```python
from main import send_multiple_messages
send_multiple_messages(count=10, delay=0.1)
```

### Monitor Bus
```python
from main import monitor_can_bus
monitor_can_bus(duration=30)  # Monitor for 30 seconds
```

### Use SocketCAN on Linux
```bash
# Set up SocketCAN interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Run python with socketcan
interface='socketcan'; channel='can0'; python main.py
```

## Troubleshooting

### "No module named 'can'"
```bash
pip install python-can
```

### Interface Not Found
- Ensure your CAN hardware is connected and recognized
- Check available interfaces with: `python -c "import can; print(can.detect_available_configs())"`

### Permission Denied (Linux)
```bash
# Add user to group (SocketCAN)
sudo usermod -a -G dialout $USER
sudo usermod -a -G can $USER
```

## References

- [python-can Documentation](https://python-can.readthedocs.io/)
- [CAN Bus Protocol](https://en.wikipedia.org/wiki/CAN_bus)
- [ESP32 CAN Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html)
