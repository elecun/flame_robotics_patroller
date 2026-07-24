#!/usr/bin/env python3
import argparse
import configparser
import os
import sys

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
    print("[APROS] System initialized successfully and ready for control.")

if __name__ == "__main__":
    main()
