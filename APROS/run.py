#!/usr/bin/env python3
"""
APROS (Autonomous Patrol Robot Operating System) Entry Point.
Launches the patrol robot control system and Viser 3D visualization dashboard.
"""
import argparse
import configparser
import os
import sys
import time

def parse_args():
    parser = argparse.ArgumentParser(
        description="APROS (Autonomous Patrol Robot Operating System) Entry Point"
    )
    parser.add_argument(
        "-c", "--config",
        type=str,
        default="apros.cfg",
        help="Path to the configuration file (default: apros.cfg)"
    )
    return parser.parse_args()

def load_config(config_path):
    if not os.path.exists(config_path):
        print(f"[Error] Configuration file not found: {config_path}")
        sys.exit(1)
        
    config = configparser.ConfigParser()
    try:
        config.read(config_path, encoding='utf-8')
        print(f"[APROS] Loaded configuration file: {config_path}")
    except Exception as e:
        print(f"[Error] Failed to parse configuration file '{config_path}': {e}")
        sys.exit(1)
        
    return config

def main():
    args = parse_args()
    
    # Resolve config path relative to script directory if not absolute
    config_path = args.config
    if not os.path.isabs(config_path):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        candidate_path = os.path.join(base_dir, config_path)
        if os.path.exists(candidate_path):
            config_path = candidate_path

    config = load_config(config_path)

    print("==================================================")
    print("  APROS - Autonomous Patrol Robot Operating System")
    print("==================================================")
    
    # Print configuration overview
    for section in config.sections():
        print(f"[{section}]")
        for key, value in config.items(section):
            print(f"  {key} = {value}")
    print("==================================================")
    
    host = config.get("NETWORK", "host", fallback="0.0.0.0")
    port = config.getint("NETWORK", "port", fallback=8080)
    can_channel = config.get("CAN", "channel", fallback="can0")

    # Initialize core MobileDriveS1 CAN controller & Viser server
    from core.device.mobile_drive_s1 import MobileDriveS1
    from core.viser_server import ViserServerManager

    robot = MobileDriveS1(
        name=config.get("ROBOT", "robot_id", fallback="patrol_robot_01"),
        channel=can_channel
    )
    robot.connect()

    viser_mgr = ViserServerManager(host=host, port=port, robot=robot)
    viser_mgr.start()

    print(f"[APROS] System initialized successfully. Web UI server listening on http://{host}:{port}")
    print("[APROS] Press Ctrl+C to exit.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[APROS] Shutting down APROS system...")
        viser_mgr.stop()
        robot.disconnect()
        print("[APROS] Terminated cleanly.")

if __name__ == "__main__":
    main()
