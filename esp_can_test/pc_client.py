"""
PC-side CAN client using python-can
Supports multiple CAN interfaces for monitoring and communicating with devices
"""

import can
import sys
import time
import argparse
import logging
from datetime import datetime
from can_utils import CANBusMonitor, format_can_data

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CANClient:
    """CAN bus client with monitoring and communication capabilities"""
    
    def __init__(self, interface='virtual', channel=None, bitrate=500000):
        """Initialize CAN client
        
        Args:
            interface: CAN interface type (virtual, socketcan, slcan, peak, kvaser)
            channel: Channel name (can0, COM3, etc.)
            bitrate: Communication bitrate in bps
        """
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.monitor = CANBusMonitor()
        self.connected = False
    
    def connect(self):
        """Connect to CAN bus"""
        try:
            config = {
                'interface': self.interface,
                'bitrate': self.bitrate,
            }
            if self.channel:
                config['channel'] = self.channel
            
            logger.info(f"Connecting to CAN bus: {config}")
            self.bus = can.Bus(**config)
            self.connected = True
            logger.info("Connected successfully")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.connected = False
            logger.info("Disconnected")
    
    def send_message(self, can_id, data, is_extended=False):
        """Send a CAN message
        
        Args:
            can_id: Message ID
            data: Message data (list or bytes)
            is_extended: Use extended frame format
        
        Returns:
            True if successful
        """
        if not self.connected:
            logger.error("Not connected")
            return False
        
        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=is_extended
            )
            self.bus.send(msg)
            self.monitor.record_tx(msg)
            data_hex = ' '.join(f'{b:02X}' for b in msg.data)
            logger.info(f"Sent: ID=0x{can_id:03X} Data=[{data_hex}]")
            return True
        except Exception as e:
            logger.error(f"Send failed: {e}")
            self.monitor.record_error()
            return False
    
    def receive_message(self, timeout=1.0):
        """Receive a CAN message
        
        Args:
            timeout: Receive timeout in seconds
        
        Returns:
            Message object or None
        """
        if not self.connected:
            logger.error("Not connected")
            return None
        
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg:
                self.monitor.record_rx(msg)
                data_hex = ' '.join(f'{b:02X}' for b in msg.data)
                logger.info(f"Received: ID=0x{msg.arbitration_id:03X} Data=[{data_hex}]")
                return msg
        except can.CanOperationError:
            # Timeout is normal
            pass
        except Exception as e:
            logger.error(f"Receive error: {e}")
            self.monitor.record_error()
        
        return None
    
    def monitor_bus(self, duration=None, print_stats_interval=10):
        """Monitor CAN bus activity
        
        Args:
            duration: Monitor duration in seconds (None = infinite)
            print_stats_interval: Print statistics every N seconds
        """
        if not self.connected:
            logger.error("Not connected")
            return
        
        logger.info(f"Monitoring CAN bus (Ctrl+C to stop)...")
        print("=" * 60)
        
        start_time = time.time()
        last_stats_time = start_time
        
        try:
            while True:
                if duration and (time.time() - start_time) > duration:
                    break
                
                msg = self.receive_message(timeout=0.5)
                
                # Print statistics periodically
                if time.time() - last_stats_time >= print_stats_interval:
                    self.monitor.print_stats()
                    last_stats_time = time.time()
        
        except KeyboardInterrupt:
            logger.info("Monitoring stopped by user")
        finally:
            print("=" * 60)
            self.monitor.print_stats()
    
    def send_sequence(self, messages, delay=0.1):
        """Send a sequence of messages
        
        Args:
            messages: List of (can_id, data) tuples
            delay: Delay between messages in seconds
        """
        for can_id, data in messages:
            self.send_message(can_id, data)
            time.sleep(delay)
    
    def interactive_mode(self):
        """Interactive send/receive mode"""
        print("CAN Interactive Mode (type 'help' for commands)")
        print("=" * 60)
        
        while True:
            try:
                cmd = input("> ").strip().lower()
                
                if cmd == 'help':
                    print("Commands:")
                    print("  send <id> <data>  - Send message (e.g., 'send 0x123 DE AD BE EF')")
                    print("  recv [timeout]    - Receive message")
                    print("  stats             - Show statistics")
                    print("  clear             - Clear statistics")
                    print("  exit              - Exit program")
                
                elif cmd.startswith('send'):
                    parts = cmd.split()
                    if len(parts) >= 2:
                        can_id = int(parts[1], 16)
                        data = [int(x, 16) for x in parts[2:]] if len(parts) > 2 else [0]
                        self.send_message(can_id, data)
                    else:
                        print("Usage: send <id> [data...]")
                
                elif cmd.startswith('recv'):
                    timeout = float(cmd.split()[1]) if len(cmd.split()) > 1 else 1.0
                    self.receive_message(timeout=timeout)
                
                elif cmd == 'stats':
                    self.monitor.print_stats()
                
                elif cmd == 'clear':
                    self.monitor = CANBusMonitor()
                    print("Statistics cleared")
                
                elif cmd == 'exit':
                    break
                
                else:
                    print(f"Unknown command: {cmd}")
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"Error: {e}")


def get_available_interfaces():
    """Get available CAN interfaces"""
    try:
        configs = can.detect_available_configs()
        print("Available CAN interfaces:")
        for config in configs:
            print(f"  - {config}")
    except Exception as e:
        logger.error(f"Could not detect interfaces: {e}")


def main():
    """Main CLI interface"""
    parser = argparse.ArgumentParser(
        description='CAN Bus Client using python-can',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python pc_client.py -i virtual monitor
  python pc_client.py -i socketcan -c can0 monitor
  python pc_client.py -i slcan -c COM3 -b 500000 interactive
  python pc_client.py -l
        """
    )
    
    parser.add_argument('-l', '--list', action='store_true',
                       help='List available CAN interfaces')
    parser.add_argument('-i', '--interface', default='virtual',
                       help='CAN interface (virtual, socketcan, slcan, peak, kvaser)')
    parser.add_argument('-c', '--channel', help='Channel name (can0, COM3, etc.)')
    parser.add_argument('-b', '--bitrate', type=int, default=500000,
                       help='Bitrate in bps (default: 500000)')
    
    subparsers = parser.add_subparsers(dest='command', help='Commands')
    
    # Monitor command
    monitor_parser = subparsers.add_parser('monitor', help='Monitor CAN bus')
    monitor_parser.add_argument('-d', '--duration', type=int,
                               help='Monitor duration in seconds')
    
    # Interactive command
    subparsers.add_parser('interactive', help='Interactive mode')
    
    # Send command
    send_parser = subparsers.add_parser('send', help='Send CAN message')
    send_parser.add_argument('id', type=lambda x: int(x, 0),
                            help='CAN ID (hex)')
    send_parser.add_argument('data', nargs='*', type=lambda x: int(x, 0),
                            help='Data bytes (hex)')
    
    args = parser.parse_args()
    
    if args.list:
        get_available_interfaces()
        return
    
    # Create client
    client = CANClient(interface=args.interface, channel=args.channel,
                      bitrate=args.bitrate)
    
    if not client.connect():
        sys.exit(1)
    
    try:
        if args.command == 'monitor':
            client.monitor_bus(duration=args.duration)
        elif args.command == 'interactive':
            client.interactive_mode()
        elif args.command == 'send':
            client.send_message(args.id, args.data if args.data else [0])
        else:
            parser.print_help()
    
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
