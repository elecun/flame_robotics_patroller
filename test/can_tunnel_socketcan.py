#!/usr/bin/env python3
import time
import sys
import threading
import can

# Interface configuration
# In SocketCAN, the two channels of the Kvaser card correspond to can0 and can1
CH0 = 'can0'  # ch0
CH1 = 'can1'  # ch1

# Flag to signal thread shutdown
stop_event = threading.Event()

def transmit_thread_task(bus_tx):
    """
    Asynchronous transmitter task running on a separate thread.
    Sends CAN FD messages on CH0 at 1-second intervals.
    """
    print(f"[*] Transmitter thread started on {CH0}...")
    msg_id = 0x100
    msg_count = 0
    
    while not stop_event.is_set():
        try:
            msg_count += 1
            # 64-byte payload for CAN FD
            payload = bytearray([i & 0xFF for i in range(64)])
            payload[0] = (msg_count >> 8) & 0xFF
            payload[1] = msg_count & 0xFF
            
            # Construct CAN Message
            # is_fd=True: Use CAN FD format
            # bitrate_switch=True: Allow data phase transmission at higher bitrate (dbitrate)
            msg = can.Message(
                arbitration_id=msg_id,
                data=payload,
                is_fd=True,
                bitrate_switch=True,
                check=True
            )
            
            spaced_data = " ".join(f"{b:02X}" for b in payload[:8]) + " ... " + " ".join(f"{b:02X}" for b in payload[-8:])
            print(f"[TX] Sending Msg #{msg_count} | ID: 0x{msg_id:03X} | Data: {spaced_data}")
            
            bus_tx.send(msg)
            
        except Exception as e:
            print(f"[TX Error] Exception: {e}")
            break
            
        # Sleep for 1 second, checking stop_event in smaller intervals
        for _ in range(10):
            if stop_event.is_set():
                break
            time.sleep(0.1)
            
    print("[*] Transmitter thread stopped.")

def receive_thread_task(bus_rx):
    """
    Asynchronous receiver task running on a separate thread.
    Listens on CH1 and logs received CAN FD messages.
    """
    print(f"[*] Receiver thread started on {CH1}...")
    while not stop_event.is_set():
        try:
            # Non-blocking or short-timeout read
            msg = bus_rx.recv(timeout=0.1)
            if msg is not None:
                data_hex = msg.data.hex().upper()
                spaced_data = " ".join(data_hex[i:i+2] for i in range(0, len(data_hex), 2))
                # Display message as a log
                print(f"[RX] Received -> ID: 0x{msg.arbitration_id:03X} | DLC: {msg.dlc} | FD: {msg.is_fd} | BRS: {msg.bitrate_switch} | Data: {spaced_data}")
        except Exception as e:
            print(f"[RX Error] Exception: {e}")
            break
    print("[*] Receiver thread stopped.")

def main():
    print("==================================================")
    print(" Kvaser PCIe CAN FD Loopback Test (SocketCAN Mode)")
    print("==================================================")
    print(f"[*] Connecting: TX (ch0)={CH0}, RX (ch1)={CH1} in CAN FD mode...")
    
    try:
        # Initialize the TX and RX buses in CAN FD mode (fd=True)
        # Using python-can abstraction (avoids manual socket creation)
        bus_tx = can.interface.Bus(channel=CH0, interface='socketcan', fd=True)
        bus_rx = can.interface.Bus(channel=CH1, interface='socketcan', fd=True)
    except OSError as e:
        print(f"[!] Error: Failed to open CAN interfaces. {e}")
        print("    Ensure that both network interfaces are UP and configured for CAN FD.")
        print("    Run the following commands first (requires sudo):")
        print(f"      sudo ip link set {CH0} type can bitrate 500000 dbitrate 2000000 fd on")
        print(f"      sudo ip link set {CH1} type can bitrate 500000 dbitrate 2000000 fd on")
        print(f"      sudo ip link set {CH0} up")
        print(f"      sudo ip link set {CH1} up")
        sys.exit(1)

    # Start transmitter and receiver threads
    tx_thread = threading.Thread(target=transmit_thread_task, args=(bus_tx,))
    rx_thread = threading.Thread(target=receive_thread_task, args=(bus_rx,))
    tx_thread.daemon = True
    rx_thread.daemon = True
    
    print("[*] Spawning TX and RX threads. Press Ctrl+C to stop.")
    tx_thread.start()
    rx_thread.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[*] Ctrl+C detected. Shutting down gracefully...")
    finally:
        stop_event.set()
        tx_thread.join(timeout=1.0)
        rx_thread.join(timeout=1.0)
        
        print("[*] Closing CAN connections...")
        try:
            bus_tx.shutdown()
            bus_rx.shutdown()
        except Exception as e:
            print(f"[!] Error during cleanup: {e}")

    print("[*] Loopback test finished.")

if __name__ == "__main__":
    main()
