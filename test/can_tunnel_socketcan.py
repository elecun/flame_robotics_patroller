#!/usr/bin/env python3
"""
SocketCAN Channel Detection and Device Verification Utility.
Standard CAN (non-FD) inspection tool.
"""
import os
import sys
import can

def get_available_socketcan_interfaces():
    """
    Scans system network interfaces under /sys/class/net/ to find SocketCAN devices (can*, vcan*, etc.).
    Returns a list of interface name strings.
    """
    interfaces = []
    net_path = "/sys/class/net"
    if not os.path.exists(net_path):
        return interfaces

    for iface in os.listdir(net_path):
        # Check if device type is CAN or interface name starts with can/vcan
        type_file = os.path.join(net_path, iface, "type")
        if iface.startswith(("can", "vcan")):
            interfaces.append(iface)
        elif os.path.exists(type_file):
            try:
                with open(type_file, "r") as f:
                    # ARPHRD_CAN is 280 (0x118) in Linux header
                    dev_type = f.read().strip()
                    if dev_type == "280" and iface not in interfaces:
                        interfaces.append(iface)
            except Exception:
                pass

    return sorted(interfaces)

def get_interface_operstate(iface):
    """Returns the operational state (UP, DOWN, UNKNOWN) of a network interface."""
    state_file = f"/sys/class/net/{iface}/operstate"
    if os.path.exists(state_file):
        try:
            with open(state_file, "r") as f:
                return f.read().strip().upper()
        except Exception:
            pass
    return "UNKNOWN"

def verify_socketcan_device(iface):
    """
    Attempts to initialize a Standard CAN (non-FD) Bus instance on the given interface.
    Returns (bool, str) representing (success_status, status_message).
    """
    try:
        # Standard CAN mode (fd=False)
        bus = can.interface.Bus(channel=iface, interface='socketcan', fd=False)
        
        # Test sending a dummy non-FD test frame if bus allows (or clean shutdown)
        state = bus.state if hasattr(bus, 'state') else 'ACTIVE'
        bus.shutdown()
        return True, f"OK (Bus State: {state})"
    except Exception as e:
        return False, str(e)

def main():
    print("==================================================")
    print(" SocketCAN Channel & Device Inspection Utility")
    print(" (Standard CAN Mode - Non-FD)")
    print("==================================================")

    channels = get_available_socketcan_interfaces()

    if not channels:
        print("[!] No SocketCAN interfaces (can*, vcan*) detected in /sys/class/net/.")
        print("    If you have physical CAN hardware, ensure drivers are loaded.")
        print("    You can create a virtual CAN channel for testing using:")
        print("      sudo modprobe vcan")
        print("      sudo ip link add dev vcan0 type vcan")
        print("      sudo ip link set vcan0 up")
        sys.exit(1)

    print(f"[*] Found {len(channels)} SocketCAN interface(s): {', '.join(channels)}")
    print("--------------------------------------------------")

    usable_count = 0
    for iface in channels:
        operstate = get_interface_operstate(iface)
        print(f"-> Interface: [{iface}] | Operstate: {operstate}")

        # Verify device opening
        is_ok, msg = verify_socketcan_device(iface)
        if is_ok:
            usable_count += 1
            print(f"   [Status] USABLE - {msg}")
        else:
            print(f"   [Status] UNUSABLE - Error: {msg}")
            if operstate == "DOWN":
                print(f"   [Hint] Interface is DOWN. Bring it UP with: 'sudo ip link set {iface} up type can bitrate 500000'")

        print("--------------------------------------------------")

    print(f"[*] Summary: {usable_count}/{len(channels)} SocketCAN channel(s) are correctly available for Standard CAN.")

if __name__ == "__main__":
    main()
