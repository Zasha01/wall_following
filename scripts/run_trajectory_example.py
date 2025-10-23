#!/usr/bin/env python3
"""
Example script demonstrating trajectory tracking usage
"""

import subprocess
import time
import os
import json
import matplotlib.pyplot as plt


def run_trajectory_example():
    """Run a complete trajectory tracking example"""
    
    print("="*60)
    print("ROBOT TRAJECTORY TRACKING EXAMPLE")
    print("="*60)
    
    print("\n1. Building the package...")
    try:
        subprocess.run(["colcon", "build", "--packages-select", "wall_following"], 
                      cwd="/home/zaka/ros2_ws", check=True)
        print("✓ Package built successfully")
    except subprocess.CalledProcessError as e:
        print(f"✗ Build failed: {e}")
        return False
    
    print("\n2. Starting the multi-robot system...")
    print("   This will run for 30 seconds to collect trajectory data")
    print("   Press Ctrl+C to stop early")
    
    try:
        # Start the system
        process = subprocess.Popen([
            "ros2", "launch", "wall_following", "multi_robot_launch.py"
        ], cwd="/home/zaka/ros2_ws")
        
        # Wait for 30 seconds
        time.sleep(30)
        
        # Stop the system
        process.terminate()
        process.wait()
        print("✓ System stopped")
        
    except KeyboardInterrupt:
        print("\n✓ System stopped by user")
        if 'process' in locals():
            process.terminate()
            process.wait()
    
    print("\n3. Analyzing trajectory data...")
    
    # Find trajectory files
    trajectory_files = []
    for file in os.listdir("/tmp"):
        if file.startswith("robot_trajectory_") and file.endswith(".json"):
            trajectory_files.append(os.path.join("/tmp", file))
    
    if not trajectory_files:
        print("✗ No trajectory files found")
        return False
    
    print(f"✓ Found {len(trajectory_files)} trajectory files")
    
    # Analyze each trajectory
    for i, file_path in enumerate(trajectory_files):
        print(f"\n   Analyzing {os.path.basename(file_path)}...")
        
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            if not data:
                print(f"   ✗ No data in {file_path}")
                continue
            
            # Calculate statistics
            duration = data[-1]['timestamp'] - data[0]['timestamp']
            path_length = 0.0
            
            for j in range(1, len(data)):
                dx = data[j]['x'] - data[j-1]['x']
                dy = data[j]['y'] - data[j-1]['y']
                path_length += (dx*dx + dy*dy)**0.5
            
            avg_speed = path_length / duration if duration > 0 else 0
            
            print(f"   ✓ Duration: {duration:.1f}s")
            print(f"   ✓ Path length: {path_length:.2f}m")
            print(f"   ✓ Average speed: {avg_speed:.3f}m/s")
            print(f"   ✓ Points: {len(data)}")
            
        except Exception as e:
            print(f"   ✗ Error analyzing {file_path}: {e}")
    
    print("\n4. Generating visualization...")
    
    try:
        # Run the visualization script
        result = subprocess.run([
            "python3", "scripts/visualize_trajectories.py", "--auto", 
            "--save", "/tmp/example_trajectory_plot.png", "--no-show"
        ], cwd="/home/zaka/ros2_ws/src/new/wall_following", 
        capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ Visualization generated: /tmp/example_trajectory_plot.png")
        else:
            print(f"✗ Visualization failed: {result.stderr}")
            
    except Exception as e:
        print(f"✗ Error generating visualization: {e}")
    
    print("\n" + "="*60)
    print("EXAMPLE COMPLETED")
    print("="*60)
    print("\nNext steps:")
    print("1. View the trajectory plot: /tmp/example_trajectory_plot.png")
    print("2. Check trajectory data files in /tmp/")
    print("3. Use RViz for real-time visualization:")
    print("   rviz2 -d /home/zaka/ros2_ws/src/new/wall_following/rviz/trajectory_visualization.rviz")
    print("4. Run detailed analysis:")
    print("   python3 scripts/visualize_trajectories.py --auto --analysis")
    
    return True


if __name__ == "__main__":
    run_trajectory_example()
