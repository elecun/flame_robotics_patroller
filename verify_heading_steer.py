"""
Verification script for Heading to Steering Angle Sign Polarity in DriveExecutor & DWA Planner.
"""

import math
import numpy as np
import configparser
from iae_patrol_v1 import IAEPatrolV1

def test_heading_steer_sign():
    cfg = configparser.ConfigParser()
    cfg.read('APROS/apros.cfg')
    robot = IAEPatrolV1(config=cfg)
    drive_exec = robot.drive_executor
    
    # Create a straight North global path (x increases from 0 to 10m, y=0, heading=0.0)
    straight_north_path = []
    for i in range(100):
        straight_north_path.append({
            "x": i * 0.1,
            "y": 0.0,
            "heading": 0.0,
            "curvature": 0.0,
            "v_ref": 0.278,
            "corridor_boundary": 3.0
        })
    drive_exec.global_path = straight_north_path

    # Case A: RTK Heading = +45 deg (North-East, +45° East from True North)
    # Robot is facing East (+45°). Path is Straight North (0°).
    # Robot needs to turn Clockwise/Right -> Dispatched Steer Angle MUST be Negative (-)
    rtk_dev = robot.devices['synerex_rtk']
    rtk_dev.heading = 45.0
    rtk_dev.latitude = drive_exec.origin_lat = 37.14681795
    rtk_dev.longitude = drive_exec.origin_lon = 127.41444111
    
    pose_45, _ = drive_exec._get_current_robot_pose()
    window_45 = drive_exec._get_local_path_window(pose_45, drive_exec.global_path)
    v_ms, target_delta_rad_45 = drive_exec.local_planner.compute_velocity_commands(pose_45, 0.0, window_45, [])
    cmd_delta_deg_45 = math.degrees(target_delta_rad_45)

    # Case B: RTK Heading = 315 deg (-45° North-West, -45° West from True North)
    # Robot is facing West (-45°). Path is Straight North (0°).
    # Robot needs to turn Counter-Clockwise/Left -> Dispatched Steer Angle MUST be Positive (+)
    rtk_dev.heading = 315.0
    pose_315, _ = drive_exec._get_current_robot_pose()
    window_315 = drive_exec._get_local_path_window(pose_315, drive_exec.global_path)
    v_ms, target_delta_rad_315 = drive_exec.local_planner.compute_velocity_commands(pose_315, 0.0, window_315, [])
    cmd_delta_deg_315 = math.degrees(target_delta_rad_315)

    print(f"Case A (RTK Heading +45° East): rheading={pose_45['heading']:.4f}rad -> Dispatched Steer Angle={cmd_delta_deg_45:.2f}°")
    print(f"Case B (RTK Heading 315° / -45° West): rheading={pose_315['heading']:.4f}rad -> Dispatched Steer Angle={cmd_delta_deg_315:.2f}°")

    assert cmd_delta_deg_45 < 0, f"Expected negative steer angle for +45° heading, got {cmd_delta_deg_45:.2f}°"
    assert cmd_delta_deg_315 > 0, f"Expected positive steer angle for 315° (-45°) heading, got {cmd_delta_deg_315:.2f}°"
    print("\n🎉 HEADING & STEERING SIGN POLARITY VERIFICATION SUCCESSFUL!")

if __name__ == "__main__":
    test_heading_steer_sign()
