#!/usr/bin/env python3
"""
APROS (Autonomous Patrol Robot Operating System) Entry Point.
Launches the patrol robot control system and Viser 3D visualization dashboard.
Creates global ZPipe context and loads the main robot platform (IAEPatrolV1).
"""
import argparse
import configparser
import os
import sys
import time

from core.zpipe import zpipe_create_pipe, zpipe_destroy_pipe
from iae_patrol_v1 import IAEPatrolV1
from core.viser_server import ViserServerManager
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


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
        logger.error(f"Configuration file not found: {config_path}")
        sys.exit(1)
        
    config = configparser.ConfigParser()
    try:
        config.read(config_path, encoding='utf-8')
        logger.info(f"Loaded configuration file: {config_path}")
    except Exception as e:
        logger.error(f"Failed to parse configuration file '{config_path}': {e}")
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

    logger.info("==================================================")
    logger.info("  APROS - Autonomous Patrol Robot Operating System")
    logger.info("==================================================")
    
    # Print configuration overview
    for section in config.sections():
        logger.info(f"[{section}]")
        for key, value in config.items(section):
            logger.info(f"  {key} = {value}")
    logger.info("==================================================")
    
    host = config.get("NETWORK", "host", fallback="0.0.0.0")
    port = config.getint("NETWORK", "port", fallback=8080)

    # 1. Initialize ZPipe main pipe context
    zpipe_ctx = zpipe_create_pipe(io_threads=1)
    logger.info("Main ZPipe context initialized successfully.")

    # 2. Instantiate and connect main robot platform (IAEPatrolV1)
    robot = IAEPatrolV1(config=config, zpipe_ctx=zpipe_ctx)
    robot.connect()

    # 3. Start Viser 3D visualization dashboard server
    viser_mgr = ViserServerManager(host=host, port=port, robot=robot)
    viser_mgr.start()

    logger.info(f"System initialized successfully. Web UI server listening on http://{host}:{port}")
    logger.info("Press Ctrl+C to exit.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger.info("Shutting down APROS system...")
        viser_mgr.stop()
        robot.disconnect()
        zpipe_destroy_pipe()
        logger.info("Terminated cleanly.")



if __name__ == "__main__":
    main()
