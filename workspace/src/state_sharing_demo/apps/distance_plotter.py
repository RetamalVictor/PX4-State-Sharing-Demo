#!/usr/bin/env python3

"""
Real-time distance plotter for multi-drone system.

This script subscribes to StateSharingMsg from multiple PX4 drones and plots
the real-time distances between them using matplotlib in the main thread.

Author: Generated for gnc_px4_demo
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Handle numpy/matplotlib compatibility issue
import warnings
warnings.filterwarnings("ignore", message=".*numpy.*")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import defaultdict, deque
import numpy as np
import math
import threading
import time
from datetime import datetime

# Import PX4 messages
from px4_msgs.msg import StateSharingMsg


class DroneDistanceMonitor(Node):
    """
    ROS2 Node that subscribes to state sharing messages and provides data for plotting.
    """
    
    def __init__(self):
        super().__init__('drone_distance_monitor')
        
        # Parameters
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('history_length', 100)
        
        self.num_drones = self.get_parameter('num_drones').value
        self.history_length = self.get_parameter('history_length').value
        
        # Data storage
        self.drone_positions = {}  # {agent_id: {lat, lon, alt, timestamp}}
        self.distance_history = defaultdict(lambda: deque(maxlen=self.history_length))
        self.time_history = deque(maxlen=self.history_length)
        self.data_lock = threading.Lock()
        
        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to state sharing messages from all drones
        self.subscribers = []
        for i in range(self.num_drones):
            topic_name = f'/px4_{i}/fmu/out/incoming_state_sharing'
            subscriber = self.create_subscription(
                StateSharingMsg,
                topic_name,
                lambda msg, drone_id=i: self.state_sharing_callback(msg, drone_id),
                qos_profile
            )
            self.subscribers.append(subscriber)
            self.get_logger().info(f'Subscribed to {topic_name}')
        
        self.get_logger().info(f'Distance monitor initialized for {self.num_drones} drones')
    
    def state_sharing_callback(self, msg: StateSharingMsg, drone_id: int):
        """
        Callback for state sharing messages.
        
        Args:
            msg: StateSharingMsg containing drone position
            drone_id: ID of the drone (0-indexed)
        """
        with self.data_lock:
            # Store drone position (using agent_id from message if available, otherwise use drone_id)
            agent_id = getattr(msg, 'agent_id', drone_id + 1)  # PX4 uses 1-indexed agent_id
            
            self.drone_positions[agent_id] = {
                'lat': msg.global_position_lat,
                'lon': msg.global_position_lon,
                'alt': msg.global_position_alt,
                'timestamp': msg.timestamp,
                'q': msg.q
            }
            
            # Calculate distances if we have multiple drones
            if len(self.drone_positions) >= 2:
                self.calculate_distances()
    
    def calculate_distances(self):
        """Calculate distances between all pairs of drones."""
        current_time = time.time()
        
        # Get list of active drone IDs
        drone_ids = list(self.drone_positions.keys())
        
        # Calculate pairwise distances
        distances = {}
        for i, drone1_id in enumerate(drone_ids):
            for j, drone2_id in enumerate(drone_ids[i+1:], i+1):
                pos1 = self.drone_positions[drone1_id]
                pos2 = self.drone_positions[drone2_id]
                
                # Calculate distance using haversine formula for lat/lon and altitude difference
                distance = self.calculate_3d_distance(
                    pos1['lat'], pos1['lon'], pos1['alt'],
                    pos2['lat'], pos2['lon'], pos2['alt']
                )
                
                pair_key = f"Drone_{drone1_id}-Drone_{drone2_id}"
                distances[pair_key] = distance
        
        # Store in history
        self.time_history.append(current_time)
        for pair_key, distance in distances.items():
            self.distance_history[pair_key].append(distance)
        
        # Ensure all pairs have the same length by padding with NaN if needed
        for pair_key in self.distance_history:
            if len(self.distance_history[pair_key]) < len(self.time_history):
                self.distance_history[pair_key].append(np.nan)
    
    def calculate_3d_distance(self, lat1, lon1, alt1, lat2, lon2, alt2):
        """
        Calculate 3D distance between two GPS coordinates with altitude.
        
        Args:
            lat1, lon1, alt1: First position (degrees, degrees, meters)
            lat2, lon2, alt2: Second position (degrees, degrees, meters)
            
        Returns:
            Distance in meters
        """
        # Earth radius in meters
        R = 6371000.0
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula for horizontal distance
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        horizontal_distance = R * c
        
        # Altitude difference
        altitude_diff = alt2 - alt1
        
        # 3D distance
        distance_3d = math.sqrt(horizontal_distance**2 + altitude_diff**2)
        
        return distance_3d
    
    # Getter methods for plotting
    def get_plot_data(self):
        """
        Get current plotting data in a thread-safe manner.
        
        Returns:
            tuple: (time_array, distance_data, drone_count)
        """
        with self.data_lock:
            if len(self.time_history) == 0:
                return None, None, 0
            
            # Convert time to relative seconds from start
            start_time = self.time_history[0]
            time_array = np.array([(t - start_time) for t in self.time_history])
            
            # Prepare distance data
            distance_data = {}
            for pair_key, distances in self.distance_history.items():
                if len(distances) > 0:
                    dist_array = np.array(list(distances))
                    distance_data[pair_key] = dist_array
            
            drone_count = len(self.drone_positions)
            
            return time_array, distance_data, drone_count
    
    def get_active_drones(self):
        """Get list of active drone IDs."""
        with self.data_lock:
            return list(self.drone_positions.keys())


class DistancePlotter:
    """
    Matplotlib-based plotter that runs in the main thread.
    """
    
    def __init__(self, monitor_node, update_rate_hz=10.0):
        self.monitor = monitor_node
        self.update_rate = update_rate_hz
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title('Real-time Distances Between Drones', fontsize=16)
        self.ax.set_xlabel('Time (seconds)', fontsize=12)
        self.ax.set_ylabel('Distance (meters)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        # Color map for different drone pairs
        self.colors = plt.cm.tab10(np.linspace(0, 1, 10))
        self.line_styles = ['-', '--', '-.', ':']
        
        print("Matplotlib plotter initialized")
        print("Close the plot window or press Ctrl+C to stop")
    
    def update_plot(self, frame=None):
        """Update the matplotlib plot with current data."""
        time_array, distance_data, drone_count = self.monitor.get_plot_data()
        
        if time_array is None or len(distance_data) == 0:
            return
        
        # Clear previous lines
        self.ax.clear()
        self.ax.set_title('Real-time Distances Between Drones', fontsize=16)
        self.ax.set_xlabel('Time (seconds)', fontsize=12)
        self.ax.set_ylabel('Distance (meters)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        
        # Plot each drone pair
        color_idx = 0
        for pair_key, dist_array in distance_data.items():
            # Only plot if we have valid data
            valid_mask = ~np.isnan(dist_array)
            if np.any(valid_mask):
                color = self.colors[color_idx % len(self.colors)]
                style = self.line_styles[color_idx % len(self.line_styles)]
                
                self.ax.plot(
                    time_array[valid_mask], 
                    dist_array[valid_mask],
                    color=color,
                    linestyle=style,
                    linewidth=2,
                    marker='o',
                    markersize=3,
                    label=pair_key
                )
                color_idx += 1
        
        # Add legend and formatting
        if color_idx > 0:
            self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            
            # Set reasonable axis limits
            if len(time_array) > 0:
                self.ax.set_xlim(max(0, time_array[-1] - 60), time_array[-1] + 1)  # Show last 60 seconds
            
            # Set y-axis limits with minimum range to avoid small variations looking like big jumps
            all_valid_distances = []
            for dist_array in distance_data.values():
                valid_distances = dist_array[~np.isnan(dist_array)]
                if len(valid_distances) > 0:
                    all_valid_distances.extend(valid_distances)
            
            if len(all_valid_distances) > 0:
                min_dist = np.min(all_valid_distances)
                max_dist = np.max(all_valid_distances)
                
                # Ensure minimum range of 2 meters
                min_range = 2.0
                current_range = max_dist - min_dist
                
                if current_range < min_range:
                    # Expand range symmetrically around the center
                    center = (min_dist + max_dist) / 2
                    half_range = min_range / 2
                    y_min = max(0, center - half_range)  # Don't go below 0
                    y_max = center + half_range
                else:
                    # Add 10% padding to existing range
                    padding = current_range * 0.1
                    y_min = max(0, min_dist - padding)
                    y_max = max_dist + padding
                
                self.ax.set_ylim(y_min, y_max)
            
            # Add current drone count info
            info_text = f"Active Drones: {drone_count}"
            self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
    
    def run(self):
        """Run the plotting loop in the main thread."""
        try:
            print("Starting real-time plotting... Close window or Ctrl+C to stop")
            
            # Show the initial plot
            plt.show(block=False)
            
            # Manual update loop - more reliable than FuncAnimation
            while plt.get_fignums():  # While figure windows are open
                try:
                    self.update_plot()
                    plt.pause(1.0 / self.update_rate)  # Update at specified rate
                except Exception as update_error:
                    print(f"Update error: {update_error}")
                    time.sleep(0.1)
            
            print("Plot window closed")
            
        except KeyboardInterrupt:
            print("\nPlot interrupted by user")
        except Exception as e:
            print(f"Plot error: {e}")
            import traceback
            traceback.print_exc()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        # Create the ROS2 node
        monitor = DroneDistanceMonitor()
        
        # Start ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
        ros_thread.start()
        
        print("ROS2 node started in background thread")
        print("Starting matplotlib in main thread...")
        
        # Give ROS2 a moment to start up
        time.sleep(1.0)
        
        # Create and run the plotter in the main thread
        plotter = DistancePlotter(monitor, update_rate_hz=30.0)
        plotter.run()
        
    except KeyboardInterrupt:
        print("\nShutting down distance plotter...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
