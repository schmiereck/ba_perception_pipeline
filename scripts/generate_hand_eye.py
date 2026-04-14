#!/usr/bin/env python3
"""
Helper script to generate a hand-eye transformation matrix.
Calculates the rotation based on a 'Look-At' target point.

Usage:
    python3 scripts/generate_hand_eye.py --cam 0.4 0.2 0.6 --target 0.3 0.0 0.0
"""

import argparse
import numpy as np
import yaml

def generate_look_at(cam_pos, target_pos, world_up=np.array([0, 0, 1])):
    """
    Generates a 4x4 transformation matrix (Camera to Robot Base).
    The camera is at cam_pos and looks at target_pos.
    """
    # 1. Calculate the 'Look' direction (Z-axis of the camera)
    # In OpenCV/ROS camera convention: 
    # Z points forward, X points right, Y points down.
    
    forward = target_pos - cam_pos
    forward = forward / np.linalg.norm(forward)
    
    # 2. Calculate Right axis (X)
    # We want X to be horizontal (perpendicular to world_up and forward)
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        # Fallback if looking straight up/down
        right = np.array([1, 0, 0])
    right = right / np.linalg.norm(right)
    
    # 3. Calculate Down axis (Y)
    down = np.cross(right, forward)
    down = down / np.linalg.norm(down)
    
    # 4. Construct Rotation Matrix (Columns are X, Y, Z axes)
    R = np.zeros((3, 3))
    R[:, 0] = right
    R[:, 1] = down
    R[:, 2] = forward
    
    # 5. Construct 4x4 Matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = cam_pos
    
    return T

def main():
    parser = argparse.ArgumentParser(description='Generate Hand-Eye Matrix via Look-At target.')
    parser.add_argument('--cam', nargs=3, type=float, required=True,
                        help='Camera position X Y Z in meters relative to robot base')
    parser.add_argument('--target', nargs=3, type=float, default=[0.3, 0.0, 0.0],
                        help='Target point X Y Z the camera is looking at (default: 0.3 0 0)')
    
    args = parser.parse_args()
    
    cam_pos = np.array(args.cam)
    target_pos = np.array(args.target)
    
    T = generate_look_at(cam_pos, target_pos)
    
    # Flatten for YAML (Row-Major)
    T_flat = T.flatten().tolist()
    
    print("\n" + "="*60)
    print("HAND-EYE CONFIGURATION GENERATOR")
    print("="*60)
    print(f"Camera Position: {cam_pos}")
    print(f"Looking at:      {target_pos}")
    print("-" * 60)
    print("Copy this into your config/perception_pipeline.yaml:")
    print("\n    hand_eye_transform: [")
    for i in range(0, 16, 4):
        line = ", ".join(f"{x:8.5f}" for x in T_flat[i:i+4])
        print(f"      {line},")
    print("    ]")
    print("-" * 60)
    print("Note: The values are stored in Row-Major order.")
    print("="*60 + "\n")

if __name__ == "__main__":
    main()
