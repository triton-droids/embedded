"""
Example scripts demonstrating python-can usage with different interfaces
"""

import can
import time
import logging
from can_utils import CANConfig, CANBusMonitor, CANMessage, format_can_data

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def example_virtual_loopback():
    """Example: Virtual interface loopback testing"""
    print("\n=== Virtual Interface Loopback Example ===")
    
    # Create virtual bus
    bus = can.Bus(interface='virtual', channel='vcan0', bitrate=500000)
    
    try:
        # Send a message
        msg = can.Message(arbitration_id=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF])
        bus.send(msg)
        print(f"Sent: {msg}")
        
        # Receive the same message (loopback)
        rx_msg = bus.recv(timeout=1.0)
        if rx_msg:
            print(f"Received: {rx_msg}")
    finally:
        bus.shutdown()


def example_socketcan():
    """Example: SocketCAN on Linux"""
    print("\n=== SocketCAN Example (Linux) ===")
    
    try:
        # Create SocketCAN bus
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
        
        # Send messages
        for i in range(5):
            msg = can.Message(arbitration_id=0x100 + i, data=[i, 0x11, 0x22, 0x33])
            bus.send(msg)
            print(f"Sent message {i}")
            time.sleep(0.1)
        
        # Receive messages
        print("\nListening for messages (5 seconds)...")
        start = time.time()
        while (time.time() - start) < 5:
            msg = bus.recv(timeout=0.1)
            if msg:
                print(f"Received: ID=0x{msg.arbitration_id:03X} Data={msg.data.hex()}")
    
    except Exception as e:
        print(f"SocketCAN example failed: {e}")
        print("Note: Run 'sudo ip link set can0 up' first on Linux")
    finally:
        bus.shutdown()


def example_slcan_serial():
    """Example: SLCAN serial interface (Windows/Mac/Linux)"""
    print("\n=== SLCAN Serial Example ===")
    
    try:
        # Create SLCAN bus (adjust COM port as needed)
        bus = can.Bus(interface='slcan', channel='COM3', bitrate=500000)
        
        # Send a message
        msg = can.Message(arbitration_id=0x100, data=[0x11, 0x22, 0x33, 0x44])
        bus.send(msg)
        print(f"Sent: {msg}")
        
        # Wait and receive
        time.sleep(0.1)
        rx_msg = bus.recv(timeout=1.0)
        if rx_msg:
            print(f"Received: {rx_msg}")
    
    except Exception as e:
        print(f"SLCAN example failed: {e}")
        print("Make sure your SLCAN adapter is connected on COM3")
    finally:
        bus.shutdown()


def example_with_monitoring():
    """Example: Using monitoring and statistics"""
    print("\n=== Monitoring and Statistics Example ===")
    
    bus = can.Bus(interface='virtual', channel='vcan0')
    monitor = CANBusMonitor()
    
    try:
        # Send multiple messages
        for i in range(5):
            msg = can.Message(
                arbitration_id=0x100 + i,
                data=[i, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77],
                is_extended_id=False
            )
            bus.send(msg)
            monitor.record_tx(msg)
            print(f"Sent: {CANMessage(msg.arbitration_id, msg.data)}")
            time.sleep(0.05)
        
        # Receive and monitor
        print("\nReceiving messages...")
        for _ in range(5):
            rx_msg = bus.recv(timeout=0.5)
            if rx_msg:
                monitor.record_rx(rx_msg)
                print(f"Received: ID=0x{rx_msg.arbitration_id:03X} Data={rx_msg.data.hex()}")
        
        # Print statistics
        monitor.print_stats()
    
    finally:
        bus.shutdown()


def example_extended_frames():
    """Example: Extended frame format"""
    print("\n=== Extended Frame Format Example ===")
    
    bus = can.Bus(interface='virtual', channel='vcan0')
    
    try:
        # Send extended format message
        msg = can.Message(
            arbitration_id=0x18FF1234,  # Extended ID
            data=[0xAA, 0xBB, 0xCC, 0xDD],
            is_extended_id=True
        )
        bus.send(msg)
        print(f"Sent extended message: ID=0x{msg.arbitration_id:08X}")
        
        # Receive
        rx_msg = bus.recv(timeout=0.5)
        if rx_msg:
            print(f"Received: ID=0x{rx_msg.arbitration_id:08X} (Extended={rx_msg.is_extended_id})")
    
    finally:
        bus.shutdown()


def example_hex_dump():
    """Example: Displaying data with hex dump"""
    print("\n=== Hex Dump Example ===")
    
    data = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                  0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF])
    
    print(format_can_data(data))


def example_filter_messages():
    """Example: Filtering CAN messages"""
    print("\n=== Message Filtering Example ===")
    
    bus = can.Bus(interface='virtual', channel='vcan0')
    
    try:
        # Send messages with different IDs
        for i in range(10):
            msg = can.Message(arbitration_id=0x100 + i, data=[i])
            bus.send(msg)
        
        # Receive and filter
        print("Receiving messages with ID 0x100-0x105...")
        for _ in range(6):
            msg = bus.recv(timeout=0.1)
            if msg and 0x100 <= msg.arbitration_id <= 0x105:
                print(f"Passed filter: ID=0x{msg.arbitration_id:03X}")
    
    finally:
        bus.shutdown()


def main():
    """Run all examples"""
    print("python-can Examples")
    print("=" * 50)
    
    # Virtual loopback (always works)
    example_virtual_loopback()
    
    # Monitoring
    example_with_monitoring()
    
    # Extended frames
    example_extended_frames()
    
    # Hex dump
    example_hex_dump()
    
    # Platform-specific examples (may fail if hardware not available)
    try:
        example_socketcan()
    except Exception as e:
        print(f"SocketCAN example skipped: {e}")
    
    try:
        example_slcan_serial()
    except Exception as e:
        print(f"SLCAN example skipped: {e}")
    
    print("\n=== Examples Complete ===")


if __name__ == "__main__":
    main()
