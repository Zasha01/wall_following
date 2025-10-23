#!/usr/bin/env python3
"""
Trajectory Visualization Tool
Plots robot trajectories from logged JSON data
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
import glob
from datetime import datetime


class TrajectoryVisualizer:
    def __init__(self):
        self.trajectories = []
        self.colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray']

    def load_trajectory(self, filename, robot_name=None):
        """Load trajectory data from JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            if not data:
                print(f"Warning: No data found in {filename}")
                return False

            # Extract x, y coordinates
            x_coords = [point['x'] for point in data]
            y_coords = [point['y'] for point in data]
            timestamps = [point['timestamp'] for point in data]
            
            trajectory = {
                'name': robot_name or os.path.basename(filename),
                'x': x_coords,
                'y': y_coords,
                'timestamps': timestamps,
                'filename': filename
            }
            
            self.trajectories.append(trajectory)
            print(f"Loaded trajectory: {trajectory['name']} ({len(data)} points)")
            return True
            
        except Exception as e:
            print(f"Error loading {filename}: {e}")
            return False

    def plot_trajectories(self, save_path=None, show_plot=True):
        """Plot all loaded trajectories"""
        if not self.trajectories:
            print("No trajectories to plot!")
            return

        plt.figure(figsize=(10, 8))
        
        for i, traj in enumerate(self.trajectories):
            color = self.colors[i % len(self.colors)]
            
            # Plot trajectory line
            plt.plot(traj['x'], traj['y'], color=color, linewidth=2, 
                    label=f"{traj['name']} ({len(traj['x'])} points)", alpha=0.7)
            
            # Mark start and end points
            plt.scatter(traj['x'][0], traj['y'][0], color=color, s=100, marker='o', 
                       label=f"{traj['name']} Start", zorder=5)
            plt.scatter(traj['x'][-1], traj['y'][-1], color=color, s=100, marker='s', 
                       label=f"{traj['name']} End", zorder=5)

        plt.xlabel('X Position (meters)', fontsize=12)
        plt.ylabel('Y Position (meters)', fontsize=12)
        plt.title('Robot Trajectories', fontsize=16)
        plt.grid(True, alpha=0.3)
        plt.legend(loc='upper right', fontsize=10)
        plt.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")
        
        if show_plot:
            plt.show()

    def plot_velocity_analysis(self, save_path=None, show_plot=True):
        """Plot velocity analysis for trajectories"""
        if not self.trajectories:
            print("No trajectories to analyze!")
            return

        fig, axes = plt.subplots(2, 1, figsize=(12, 10))
        fig.suptitle('Trajectory Analysis', fontsize=16)

        for i, traj in enumerate(self.trajectories):
            color = self.colors[i % len(self.colors)]
            
            # Calculate velocities
            x_vel = np.gradient(traj['x'], traj['timestamps'])
            y_vel = np.gradient(traj['y'], traj['timestamps'])
            speed = np.sqrt(x_vel**2 + y_vel**2)
            
            # Plot speed over time
            axes[0].plot(traj['timestamps'], speed, color=color, 
                        label=traj['name'], alpha=0.7, linewidth=2)
            
            # Plot trajectory with speed color coding
            scatter = axes[1].scatter(traj['x'], traj['y'], c=speed, 
                                    cmap='viridis', s=20, alpha=0.7)

        # Speed over time plot
        axes[0].set_title('Speed Over Time', fontsize=14)
        axes[0].set_xlabel('Time (seconds)', fontsize=12)
        axes[0].set_ylabel('Speed (m/s)', fontsize=12)
        axes[0].legend(loc='upper right', fontsize=10)
        axes[0].grid(True, alpha=0.3)

        # Speed color map plot
        axes[1].set_title('Trajectory Speed Map', fontsize=14)
        axes[1].set_xlabel('X Position (meters)', fontsize=12)
        axes[1].set_ylabel('Y Position (meters)', fontsize=12)
        axes[1].grid(True, alpha=0.3)
        axes[1].axis('equal')
        
        # Add colorbar for speed map
        cbar = plt.colorbar(scatter, ax=axes[1], shrink=0.8)
        cbar.set_label('Speed (m/s)', fontsize=10)

        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Analysis plot saved to: {save_path}")
        
        if show_plot:
            plt.show()

    def print_statistics(self):
        """Print trajectory statistics"""
        print("\n" + "="*60)
        print("TRAJECTORY STATISTICS")
        print("="*60)
        
        for traj in self.trajectories:
            print(f"\n{traj['name']}:")
            print(f"  Points: {len(traj['x'])}")
            print(f"  Duration: {traj['timestamps'][-1] - traj['timestamps'][0]:.2f} seconds")
            
            # Calculate path length
            path_length = 0.0
            for i in range(1, len(traj['x'])):
                dx = traj['x'][i] - traj['x'][i-1]
                dy = traj['y'][i] - traj['y'][i-1]
                path_length += np.sqrt(dx*dx + dy*dy)
            
            print(f"  Path Length: {path_length:.2f} meters")
            
            # Calculate average speed
            duration = traj['timestamps'][-1] - traj['timestamps'][0]
            avg_speed = path_length / duration if duration > 0 else 0
            print(f"  Average Speed: {avg_speed:.3f} m/s")
            
            # Calculate bounding box
            x_min, x_max = min(traj['x']), max(traj['x'])
            y_min, y_max = min(traj['y']), max(traj['y'])
            print(f"  Bounding Box: ({x_min:.2f}, {y_min:.2f}) to ({x_max:.2f}, {y_max:.2f})")
            print(f"  Area Covered: {(x_max-x_min)*(y_max-y_min):.2f} m²")


def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectories')
    parser.add_argument('files', nargs='*', help='Trajectory JSON files to load')
    parser.add_argument('--auto', action='store_true', 
                       help='Automatically load all trajectory files from /tmp/')
    parser.add_argument('--save', type=str, help='Save plot to file')
    parser.add_argument('--analysis', action='store_true', 
                       help='Show velocity analysis plots')
    parser.add_argument('--no-show', action='store_true', 
                       help='Don\'t display plots (useful for saving only)')
    
    args = parser.parse_args()
    
    visualizer = TrajectoryVisualizer()
    
    # Load trajectories
    if args.auto:
        # Auto-load from /tmp/
        pattern = '/tmp/robot_trajectory_*.json'
        files = glob.glob(pattern)
        if files:
            print(f"Found {len(files)} trajectory files:")
            for file in files:
                robot_name = os.path.basename(file).replace('robot_trajectory_', '').replace('.json', '')
                visualizer.load_trajectory(file, robot_name)
        else:
            print("No trajectory files found in /tmp/")
            return
    elif args.files:
        for file in args.files:
            visualizer.load_trajectory(file)
    else:
        print("No files specified. Use --auto or provide file paths.")
        return
    
    if not visualizer.trajectories:
        print("No trajectories loaded!")
        return
    
    # Print statistics
    visualizer.print_statistics()
    
    # Generate plots
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    if args.analysis:
        analysis_path = args.save or f'/tmp/trajectory_analysis_{timestamp}.png'
        visualizer.plot_velocity_analysis(analysis_path, not args.no_show)
    else:
        plot_path = args.save or f'/tmp/trajectory_plot_{timestamp}.png'
        visualizer.plot_trajectories(plot_path, not args.no_show)


if __name__ == '__main__':
    main()
