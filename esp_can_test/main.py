"""
CAN Communication using python-can library
Supports multiple CAN interfaces: SocketCAN, PEAK, KVASER, ect-serial, slcan, etc.

Install: pip install python-can
"""

import can
import time
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# CAN interface configuration
# Available interfaces: 'socketcan', 'peak', 'kvaser', 'seican', 'slcan', 'virtual', 'udp_multicast'
# Example configurations:
# Linux/Mac SocketCAN: interface='socketcan', channel='can0' (requires SocketCAN kernel module)
# Windows/Serial: interface='slcan', channel='COM3', bitrate=500000
# Virtual (e.g., testing): interface='virtual', channel='vcan0'

CAN_CONFIG = {
    'interface': 'virtual',      # Change to your interface type
    'channel': 'vcan0',          # Change to your channel (e.g., 'COM3' for serial)
    'bitrate': 500000,           # 500 kbps
}

bus = None


def print_status(tag):
    """Print CAN bus status"""
    if bus is None:
        print(f"[{tag}] CAN bus not initialized")
        return
    
    try:
        state = bus.state
        print(f"[{tag}] Bus state: {state}")
    except Exception as e:
        logger.warning(f"[{tag}] Could not get bus state: {e}")


def initialize_can():
    """Initialize CAN bus"""
    global bus
    
    try:
        logger.info(f"Initializing CAN with config: {CAN_CONFIG}")
        bus = can.Bus(**CAN_CONFIG)
        logger.info("CAN bus initialized successfully")
        return True
    except Exception as e:
        logger.error(f"CAN initialization failed: {e}")
        logger.info("Trying fallback to virtual interface for demo...")
        try:
            bus = can.Bus(interface='virtual', channel='vcan0', bitrate=500000)
            logger.info("Virtual CAN bus initialized (demo mode)")
            return True
        except Exception as e2:
            logger.error(f"Fallback also failed: {e2}")
            return False


def send_can_message(can_id=0x123, data=None, is_extended=False):
    """Send a CAN message
    
    Args:
        can_id: CAN message ID (0x000-0x7FF for standard, 0x0-0x1FFFFFFF for extended)
        data: Message data (bytes or list), max 8 bytes for classic CAN, up to 64 for CAN FD
        is_extended: Whether to use extended frame format
    """
    if bus is None:
        logger.error("CAN not initialized")
        return False
    
    if data is None:
        data = [0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44]
    
    try:
        # Create CAN message
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=is_extended,
            is_fd=False  # Set True for CAN FD
        )
        
        # Send with timeout
        bus.send(msg, timeout=0.1)
        print("TX OK")
        return True
    except Exception as e:
        logger.error(f"TX FAIL: {e}")
        return False


def receive_can_message(timeout=0.1):
    """Receive a CAN message
    
    Args:
        timeout: Receive timeout in seconds
    
    Returns:
        can.Message object or None
    """
    if bus is None:
        logger.error("CAN not initialized")
        return None
    
    try:
        msg = bus.recv(timeout=timeout)
        if msg:
            data_hex = ' '.join(f'{b:02x}' for b in msg.data)
            frame_type = "EXT" if msg.is_extended_id else "STD"
            print(f"RX id=0x{msg.arbitration_id:03x} dlc={msg.dlc} data={data_hex} [{frame_type}]")
            return msg
    except can.CanOperationError:
        # Timeout - no message received
        pass
    except Exception as e:
        logger.error(f"RX error: {e}")
    
    return None


def send_multiple_messages(count=5, delay=0.2):
    """Send multiple CAN messages in sequence
    
    Args:
        count: Number of messages to send
        delay: Delay between messages in seconds
    """
    print(f"\nSending {count} CAN messages...")
    for i in range(count):
        data = [0x00 + i, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]
        send_can_message(can_id=0x100 + i, data=data)
        time.sleep(delay)


def monitor_can_bus(duration=None, print_interval=5):
    """Monitor CAN bus activity
    
    Args:
        duration: Monitor duration in seconds (None = infinite)
        print_interval: Print stats every N seconds
    """
    if bus is None:
        logger.error("CAN not initialized")
        return
    
    logger.info(f"Monitoring CAN bus (Ctrl+C to stop)...")
    print_status("after_start")
    
    msg_count = 0
    start_time = time.time()
    last_print = start_time
    
    try:
        while True:
            if duration and (time.time() - start_time) > duration:
                break
            
            msg = receive_can_message(timeout=0.1)
            if msg:
                msg_count += 1
            
            # Print stats periodically
            if time.time() - last_print >= print_interval:
                elapsed = time.time() - start_time
                rate = msg_count / elapsed if elapsed > 0 else 0
                print(f"[Stats] Received {msg_count} messages in {elapsed:.1f}s ({rate:.2f} msg/s)")
                last_print = time.time()
    except KeyboardInterrupt:
        logger.info("Monitoring stopped by user")
    finally:
        print_status("before_shutdown")


def main():
    """Main application loop"""
    # Initialize CAN bus
    if not initialize_can():
        return
    
    print_status("after_init")
    
    try:
        counter = 0
        while True:
            counter += 1
            print(f"\n--- Cycle {counter} ---")
            
            # Send a CAN message
            send_can_message()
            
            # Print status
            print_status("after_tx")
            
            # Try to receive a message
            receive_can_message(timeout=0.05)
            
            # Wait 500ms
            time.sleep(0.5)
    except KeyboardInterrupt:
        logger.info("Shutdown requested")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        if bus:
            bus.shutdown()
            logger.info("CAN bus shutdown")


if __name__ == "__main__":
    main()
