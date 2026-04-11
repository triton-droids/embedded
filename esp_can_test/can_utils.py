"""
CAN communication utilities for python-can
Provides helper classes and functions for CAN bus operations
"""

import can
from datetime import datetime
from typing import Optional, List, Tuple


class CANConfig:
    """CAN bus configuration presets"""
    
    BAUDRATES = {
        '250K': 250000,
        '500K': 500000,
        '1M': 1000000,
    }
    
    INTERFACE_TYPES = {
        'socketcan': 'SocketCAN (Linux)',
        'slcan': 'Serial CAN (Windows/Mac)',
        'peak': 'PEAK PCAN',
        'kvaser': 'Kvaser',
        'virtual': 'Virtual (Testing)',
    }
    
    @staticmethod
    def get_config(interface='virtual', channel=None, bitrate=500000):
        """Get CAN bus configuration
        
        Args:
            interface: Interface type (socketcan, slcan, peak, kvaser, virtual)
            channel: Channel name (can0, COM3, etc.)
            bitrate: Bitrate in bps
        
        Returns:
            Dict with CAN bus configuration
        """
        config = {
            'interface': interface,
            'bitrate': bitrate,
        }
        
        if channel:
            config['channel'] = channel
        elif interface == 'virtual':
            config['channel'] = 'vcan0'
        elif interface == 'socketcan':
            config['channel'] = 'can0'
        
        return config


class CANMessage:
    """Enhanced CAN message wrapper"""
    
    def __init__(self, can_id, data, is_extended=False, is_fd=False):
        """Initialize CAN message
        
        Args:
            can_id: CAN message ID (0x000-0x7FF for standard, 0x0-0x1FFFFFFF for extended)
            data: Message data (bytes, bytearray, or list)
            is_extended: Whether to use extended frame format
            is_fd: Whether to use CAN FD format
        """
        self.can_id = can_id
        self.data = bytes(data) if not isinstance(data, bytes) else data
        self.is_extended = is_extended
        self.is_fd = is_fd
        self.timestamp = datetime.now()
    
    def to_can_message(self) -> can.Message:
        """Convert to python-can Message object"""
        return can.Message(
            arbitration_id=self.can_id,
            data=self.data,
            is_extended_id=self.is_extended,
            is_fd=self.is_fd
        )
    
    def __repr__(self):
        data_str = ' '.join(f'{b:02x}' for b in self.data)
        frame_type = "EXT" if self.is_extended else "STD"
        fd_str = " FD" if self.is_fd else ""
        return f"CANMessage(ID=0x{self.can_id:X}, DLC={len(self.data)}, [{data_str}], {frame_type}{fd_str})"
    
    def __str__(self):
        return self.__repr__()


class CANBusMonitor:
    """Monitor CAN bus activity and collect statistics"""
    
    def __init__(self):
        self.tx_count = 0
        self.rx_count = 0
        self.error_count = 0
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.messages = []
    
    def record_tx(self, message: can.Message):
        """Record transmitted message"""
        self.tx_count += 1
        self.tx_bytes += len(message.data)
        self.messages.append(('TX', message))
    
    def record_rx(self, message: can.Message):
        """Record received message"""
        self.rx_count += 1
        self.rx_bytes += len(message.data)
        self.messages.append(('RX', message))
    
    def record_error(self):
        """Record communication error"""
        self.error_count += 1
    
    def get_stats(self) -> dict:
        """Get current statistics
        
        Returns:
            Dictionary with statistics
        """
        return {
            'tx_count': self.tx_count,
            'rx_count': self.rx_count,
            'error_count': self.error_count,
            'tx_bytes': self.tx_bytes,
            'rx_bytes': self.rx_bytes,
            'total_messages': self.tx_count + self.rx_count,
            'total_bytes': self.tx_bytes + self.rx_bytes,
        }
    
    def print_stats(self):
        """Print statistics to console"""
        stats = self.get_stats()
        print("\n" + "="*40)
        print("CAN Bus Statistics")
        print("="*40)
        for key, value in stats.items():
            print(f"{key:20s}: {value}")
        print("="*40 + "\n")
    
    def get_message_history(self, limit: Optional[int] = None) -> List[Tuple[str, can.Message]]:
        """Get message history
        
        Args:
            limit: Maximum number of messages to return
        
        Returns:
            List of (direction, message) tuples
        """
        if limit:
            return self.messages[-limit:]
        return self.messages


class CANBusReader:
    """Helper class for reading CAN messages"""
    
    def __init__(self, bus: can.Bus, timeout: float = 0.1):
        """Initialize reader
        
        Args:
            bus: python-can Bus object
            timeout: Default read timeout in seconds
        """
        self.bus = bus
        self.timeout = timeout
    
    def read_message(self, timeout: Optional[float] = None) -> Optional[can.Message]:
        """Read a single message
        
        Args:
            timeout: Override default timeout
        
        Returns:
            Message object or None on timeout
        """
        t = timeout if timeout is not None else self.timeout
        return self.bus.recv(timeout=t)
    
    def read_messages(self, duration: float = 1.0) -> List[can.Message]:
        """Read all messages within duration
        
        Args:
            duration: Duration in seconds
        
        Returns:
            List of received messages
        """
        messages = []
        start = datetime.now()
        while (datetime.now() - start).total_seconds() < duration:
            msg = self.bus.recv(timeout=0.01)
            if msg:
                messages.append(msg)
        return messages


class CANBusWriter:
    """Helper class for writing CAN messages"""
    
    def __init__(self, bus: can.Bus, timeout: float = 0.1):
        """Initialize writer
        
        Args:
            bus: python-can Bus object
            timeout: Default write timeout in seconds
        """
        self.bus = bus
        self.timeout = timeout
    
    def send_message(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        """Send a CAN message
        
        Args:
            can_id: Message ID
            data: Message data
            is_extended: Use extended frame format
        
        Returns:
            True if successful, False otherwise
        """
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=is_extended
            )
            self.bus.send(msg, timeout=self.timeout)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def send_messages(self, messages: List[can.Message]) -> int:
        """Send multiple messages
        
        Args:
            messages: List of Message objects
        
        Returns:
            Number of successfully sent messages
        """
        sent = 0
        for msg in messages:
            try:
                self.bus.send(msg, timeout=self.timeout)
                sent += 1
            except Exception as e:
                print(f"Send error: {e}")
        return sent


def format_can_data(data: bytes, columns: int = 8) -> str:
    """Format CAN data bytes for display (hex dump style)
    
    Args:
        data: Bytes to format
        columns: Bytes per row
    
    Returns:
        Formatted string
    """
    lines = []
    for i in range(0, len(data), columns):
        chunk = data[i:i+columns]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{i:04X}: {hex_str:<{columns*3-1}} {ascii_str}")
    return '\n'.join(lines)


def create_bus(interface: str = 'virtual', channel: Optional[str] = None, 
               bitrate: int = 500000) -> Optional[can.Bus]:
    """Create a CAN bus instance
    
    Args:
        interface: Interface type
        channel: Channel name
        bitrate: Bitrate in bps
    
    Returns:
        Bus object or None on failure
    """
    try:
        config = CANConfig.get_config(interface, channel, bitrate)
        bus = can.Bus(**config)
        return bus
    except Exception as e:
        print(f"Failed to create bus: {e}")
        return None
