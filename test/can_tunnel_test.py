#!/usr/bin/env python3
import time
import sys
import threading
from canlib import canlib, Frame

# Channel configuration (Modify if your card channels are ordered differently)
CH0 = 0  # Kvaser Channel 0 (ch0)
CH1 = 1  # Kvaser Channel 1 (ch1)

# Flag to signal thread shutdown
stop_event = threading.Event()

def transmit_thread_task(ch_tx):
    """
    Asynchronous transmitter task running on a separate thread.
    Sends CAN FD messages on CH0 at 1-second intervals.
    """
    print(f"[*] Transmitter thread started on Kvaser Channel {CH0}...")
    msg_id = 0x100
    msg_count = 0
    
    while not stop_event.is_set():
        try:
            msg_count += 1
            # Construct a CAN FD test payload (up to 64 bytes)
            payload = bytearray([i & 0xFF for i in range(64)])
            # Put the message count in the first two bytes for tracing
            payload[0] = (msg_count >> 8) & 0xFF
            payload[1] = msg_count & 0xFF
            
            # Construct Kvaser Frame with CAN FD flags
            # FDF: CAN FD Format
            # BRS: Bit Rate Switch (sends data phase at 2Mbps)
            frame = Frame(
                id_=msg_id,
                data=payload,
                flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS
            )
            
            spaced_data = " ".join(f"{b:02X}" for b in payload[:8]) + " ... " + " ".join(f"{b:02X}" for b in payload[-8:])
            print(f"[TX] Sending Msg #{msg_count} | ID: 0x{msg_id:03X} | Data: {spaced_data}")
            
            ch_tx.write(frame)
            
        except canlib.CanError as e:
            print(f"[TX Error] CANlib Exception: {e}")
            break
        except Exception as e:
            print(f"[TX Error] Unexpected exception: {e}")
            break
            
        # Sleep for 1 second, checking stop_event in smaller intervals
        for _ in range(10):
            if stop_event.is_set():
                break
            time.sleep(0.1)
            
    print("[*] Transmitter thread stopped.")

def receive_thread_task(ch_rx):
    """
    Asynchronous receiver task running on a separate thread.
    Listens on CH1 and logs received CAN FD messages.
    """
    print(f"[*] Receiver thread started on Kvaser Channel {CH1}...")
    while not stop_event.is_set():
        try:
            # Read from bus (timeout in milliseconds)
            frame = ch_rx.read(timeout=100)
            
            # Print the received message as a log
            data_hex = frame.data.hex().upper()
            spaced_data = " ".join(data_hex[i:i+2] for i in range(0, len(data_hex), 2))
            print(f"[RX] Received -> ID: 0x{frame.id:03X} | DLC: {len(frame.data)} | Flags: {frame.flags} | Data: {spaced_data}")
            
        except canlib.CanNoMsg:
            # No message available in this cycle, continue polling
            continue
        except canlib.CanError as e:
            print(f"[RX Error] CANlib Exception: {e}")
            break
        except Exception as e:
            print(f"[RX Error] Unexpected exception: {e}")
            break
    print("[*] Receiver thread stopped.")

def main():
    print("==================================================")
    print(" Kvaser PCIe CAN FD Loopback Test (CANlib Mode)")
    print("==================================================")
    
    num_channels = canlib.getNumberOfChannels()
    print(f"[*] Detected Kvaser Channels: {num_channels}")
    if num_channels < 2:
        print("[!] Warning: Less than 2 Kvaser channels detected by CANlib.")
        print("    Ensure the Kvaser proprietary kernel driver (kvpciefd.ko) is loaded.")
        print("    If SocketCAN driver is loaded instead, CANlib cannot detect channels.")
        # Proceed anyway as the user might run this on their target system later

    print(f"[*] Opening Channel {CH0} for TX (ch0) and Channel {CH1} for RX (ch1) in CAN FD mode...")
    
    try:
        # Open both channels with CAN FD capability
        ch_tx = canlib.openChannel(CH0, flags=canlib.Open.CAN_FD)
        ch_rx = canlib.openChannel(CH1, flags=canlib.Open.CAN_FD)
        
        # Set Arbitration Phase Bitrate (500k) and Data Phase Bitrate (2M)
        print("[*] Setting bitrates: Arbitration=500k, Data=2M...")
        ch_tx.setBusParams(canlib.Bitrate.BITRATE_500K)
        ch_tx.setBusParamsFd(canlib.BitrateFD.BITRATE_2M_80P)
        
        ch_rx.setBusParams(canlib.Bitrate.BITRATE_500K)
        ch_rx.setBusParamsFd(canlib.BitrateFD.BITRATE_2M_80P)
        
        # Go Bus On
        print("[*] Going Bus ON...")
        ch_tx.busOn()
        ch_rx.busOn()
        
    except canlib.CanError as e:
        print(f"[!] Initialization Failed: {e}")
        print("    If you are running in a SocketCAN environment, please run the socketcan alternative script.")
        sys.exit(1)
        
    # Start transmitter and receiver threads
    tx_thread = threading.Thread(target=transmit_thread_task, args=(ch_tx,))
    rx_thread = threading.Thread(target=receive_thread_task, args=(ch_rx,))
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
        # Signal threads to stop
        stop_event.set()
        tx_thread.join(timeout=1.0)
        rx_thread.join(timeout=1.0)
        
        # Go Bus Off and close channels
        print("[*] Going Bus OFF & closing channels...")
        try:
            ch_tx.busOff()
            ch_rx.busOff()
            ch_tx.close()
            ch_rx.close()
        except Exception as e:
            print(f"[!] Error during cleanup: {e}")
            
    print("[*] Loopback test finished.")


if __name__ == "__main__":
    main()
