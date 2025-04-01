# Educational Robotics Simulation
# Author: Ian Brown
# Purpose: Sample project demonstrating robotics teaching concepts for middle/high school students

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import math

class RobotSimulation:
    """
    A simple 2D robot simulation for teaching basic robotics concepts:
    - Movement and navigation
    - Sensor data processing
    - Obstacle avoidance
    - Path planning
    """
    
    def __init__(self):
        # Robot position and orientation
        self.x = 1
        self.y = 1
        self.theta = 0  # heading in radians
        
        # Robot parameters
        self.wheel_radius = 0.1
        self.wheel_base = 0.5
        self.max_speed = 0.2
        
        # Environment setup
        self.arena_size = 10
        self.obstacles = [
            {"x": 3, "y": 3, "radius": 0.5},
            {"x": 7, "y": 2, "radius": 0.7},
            {"x": 5, "y": 6, "radius": 0.8},
            {"x": 2, "y": 7, "radius": 0.4}
        ]
        
        # Sensor data (simulated distances in 8 directions)
        self.sensor_angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
        self.sensor_max_range = 3.0
        self.sensor_readings = np.zeros(len(self.sensor_angles))
        
        # Target position
        self.target_x = 8
        self.target_y = 8
        
        # Path history for visualization
        self.path_x = [self.x]
        self.path_y = [self.y]
        
        # Setup visualization
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(0, self.arena_size)
        self.ax.set_ylim(0, self.arena_size)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('Educational Robot Simulation')
        
        # Plot elements
        self.robot_plot = self.ax.plot([], [], 'bo', markersize=15)[0]
        self.direction_plot = self.ax.plot([], [], 'r-', linewidth=2)[0]
        self.path_plot = self.ax.plot([], [], 'g-', linewidth=1)[0]
        self.target_plot = self.ax.plot(self.target_x, self.target_y, 'r*', markersize=15)[0]
        self.sensor_lines = [self.ax.plot([], [], 'y-', alpha=0.3)[0] for _ in range(len(self.sensor_angles))]
        
        # Information text
        self.info_text = self.ax.text(0.5, 0.95, '', transform=self.ax.transAxes, 
                                     horizontalalignment='center', verticalalignment='top',
                                     bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="black", alpha=0.8))
        
        # Draw obstacles
        for obs in self.obstacles:
            circle = plt.Circle((obs["x"], obs["y"]), obs["radius"], color='gray', alpha=0.7)
            self.ax.add_patch(circle)
    
    def update_sensor_readings(self):
        """Update simulated sensor readings based on obstacles"""
        for i, angle in enumerate(self.sensor_angles):
            # Direction in world coordinates
            sensor_angle = self.theta + angle
            dx = math.cos(sensor_angle)
            dy = math.sin(sensor_angle)
            
            # Start with maximum range
            min_distance = self.sensor_max_range
            
            # Check distance to each obstacle
            for obs in self.obstacles:
                # Vector from robot to obstacle center
                to_obstacle_x = obs["x"] - self.x
                to_obstacle_y = obs["y"] - self.y
                
                # Project onto sensor direction
                t = to_obstacle_x * dx + to_obstacle_y * dy
                
                # Skip if obstacle is behind sensor
                if t < 0:
                    continue
                
                # Distance from projected point to obstacle center
                proj_x = self.x + t * dx
                proj_y = self.y + t * dy
                
                closest_dist = math.sqrt((proj_x - obs["x"])**2 + (proj_y - obs["y"])**2)
                
                # If projection point is within obstacle radius, calculate intersection
                if closest_dist < obs["radius"]:
                    # Calculate intersection distance (using Pythagorean theorem)
                    intersection_dist = t - math.sqrt(obs["radius"]**2 - closest_dist**2)
                    
                    # Update if this is the closest obstacle
                    if 0 <= intersection_dist < min_distance:
                        min_distance = intersection_dist
            
            # Check for arena boundaries
            if dx != 0:
                t_x1 = (0 - self.x) / dx
                t_x2 = (self.arena_size - self.x) / dx
                if t_x1 > 0 and t_x1 < min_distance:
                    min_distance = t_x1
                if t_x2 > 0 and t_x2 < min_distance:
                    min_distance = t_x2
            
            if dy != 0:
                t_y1 = (0 - self.y) / dy
                t_y2 = (self.arena_size - self.y) / dy
                if t_y1 > 0 and t_y1 < min_distance:
                    min_distance = t_y1
                if t_y2 > 0 and t_y2 < min_distance:
                    min_distance = t_y2
            
            self.sensor_readings[i] = min_distance
    
    def simple_obstacle_avoidance(self):
        """
        A basic obstacle avoidance algorithm for educational purposes
        Returns motor speeds for left and right wheels
        """
        # Calculate vector to target
        to_target_x = self.target_x - self.x
        to_target_y = self.target_y - self.y
        
        # Calculate desired heading angle
        desired_theta = math.atan2(to_target_y, to_target_x)
        
        # Calculate heading error
        theta_error = self.normalize_angle(desired_theta - self.theta)
        
        # Default speeds (straight to target)
        left_speed = self.max_speed
        right_speed = self.max_speed
        
        # Adjust for heading error
        if theta_error > 0:
            # Turn right
            right_speed = self.max_speed * (1 - min(abs(theta_error), 0.5) / 0.5)
        else:
            # Turn left
            left_speed = self.max_speed * (1 - min(abs(theta_error), 0.5) / 0.5)
        
        # Check front sensors for obstacles
        front_sensor_idx = 0  # Front sensor index
        if self.sensor_readings[front_sensor_idx] < 1.0:
            # Obstacle detected in front
            left_side_idx = 2  # Left side sensor index
            right_side_idx = 6  # Right side sensor index
            
            # Decide which way to turn based on side clearance
            if self.sensor_readings[left_side_idx] > self.sensor_readings[right_side_idx]:
                # More space on left, turn left
                right_speed = self.max_speed
                left_speed = -self.max_speed / 2
            else:
                # More space on right, turn right
                left_speed = self.max_speed
                right_speed = -self.max_speed / 2
        
        return left_speed, right_speed
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def move_robot(self, left_speed, right_speed, dt=0.1):
        """Move robot based on differential drive model"""
        # Calculate linear and angular velocity from wheel speeds
        linear_velocity = (right_speed + left_speed) / 2
        angular_velocity = (right_speed - left_speed) / self.wheel_base
        
        # Update position and orientation
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt
        self.theta = self.normalize_angle(self.theta)
        
        # Keep robot in arena bounds
        self.x = max(0, min(self.x, self.arena_size))
        self.y = max(0, min(self.y, self.arena_size))
        
        # Record path
        self.path_x.append(self.x)
        self.path_y.append(self.y)
    
    def update_plot(self, frame):
        """Update visualization for animation"""
        # Update sensor readings
        self.update_sensor_readings()
        
        # Get motor commands from control algorithm
        left_speed, right_speed = self.simple_obstacle_avoidance()
        
        # Move robot
        self.move_robot(left_speed, right_speed)
        
        # Update robot position in plot
        self.robot_plot.set_data(self.x, self.y)
        
        # Update direction indicator
        direction_x = [self.x, self.x + 0.5 * math.cos(self.theta)]
        direction_y = [self.y, self.y + 0.5 * math.sin(self.theta)]
        self.direction_plot.set_data(direction_x, direction_y)
        
        # Update path
        self.path_plot.set_data(self.path_x, self.path_y)
        
        # Update sensor lines
        for i, line in enumerate(self.sensor_lines):
            angle = self.theta + self.sensor_angles[i]
            end_x = self.x + self.sensor_readings[i] * math.cos(angle)
            end_y = self.y + self.sensor_readings[i] * math.sin(angle)
            line.set_data([self.x, end_x], [self.y, end_y])
        
        # Update info text
        distance_to_target = math.sqrt((self.x - self.target_x)**2 + (self.y - self.target_y)**2)
        info = f"Position: ({self.x:.2f}, {self.y:.2f})\n"
        info += f"Heading: {math.degrees(self.theta):.1f}°\n"
        info += f"Distance to Target: {distance_to_target:.2f}\n"
        info += f"Left Motor: {left_speed:.2f}, Right Motor: {right_speed:.2f}"
        self.info_text.set_text(info)
        
        return [self.robot_plot, self.direction_plot, self.path_plot, 
                self.info_text, *self.sensor_lines]
    
    def run_simulation(self, frames=200):
        """Run the animation"""
        ani = FuncAnimation(self.fig, self.update_plot, frames=frames, 
                            interval=50, blit=True)
        plt.show()


# Add educational lesson structure
def robotics_lesson():
    print("========== EDUCATIONAL ROBOTICS SIMULATION ==========")
    print("Concepts demonstrated:")
    print("1. Differential drive kinematics")
    print("2. Sensor-based obstacle avoidance")
    print("3. Simple navigation algorithms")
    print("4. Robot localization and path planning")
    print("\nThis simulation can be used to teach:")
    print("- Basic physics of robot movement")
    print("- Programming logic for autonomous navigation")
    print("- Sensor data processing")
    print("- Mathematical concepts (vectors, trigonometry)")
    print("\nStarting simulation...\n")
    
    # Create and run simulation
    sim = RobotSimulation()
    sim.run_simulation()


# Example lesson extensions (for classroom use)
class LessonPlans:
    """
    Sample lesson plans demonstrating how this simulation could be extended
    for classroom teaching at different grade levels
    """
    
    @staticmethod
    def middle_school_intro():
        """Introduction to robotics for middle school students"""
        print("""
        MIDDLE SCHOOL ROBOTICS INTRODUCTION (Grades 6-8)
        
        Learning Objectives:
        - Understand basic robot components and movement
        - Learn about sensors and how robots "see" their environment
        - Apply simple logic to robot navigation
        
        Classroom Activities:
        1. Interactive demonstration of simulation
        2. Student worksheet on predicting robot behavior
        3. Group activity: design an obstacle course on graph paper
        4. Extension: modify code to change robot speed or sensor range
        """)
    
    @staticmethod
    def high_school_programming():
        """Programming concepts for high school students"""
        print("""
        HIGH SCHOOL ROBOTICS PROGRAMMING (Grades 9-12)
        
        Learning Objectives:
        - Understand differential drive kinematics
        - Apply trigonometry to robot navigation
        - Implement and test different obstacle avoidance algorithms
        - Analyze efficiency of different path planning approaches
        
        Coding Challenges:
        1. Modify the obstacle avoidance algorithm to handle complex environments
        2. Implement a wall-following behavior
        3. Create a more efficient path planning algorithm
        4. Add simulated sensor noise and develop filtering techniques
        
        Assessment:
        - Code review of student modifications
        - Analysis report comparing algorithm performance
        - Team competition: fastest completion of custom obstacle course
        """)
    
    @staticmethod
    def competitive_robotics_team():
        """Extension for robotics competition preparation"""
        print("""
        COMPETITIVE ROBOTICS TEAM PREPARATION
        
        Applications:
        - Use simulation to prototype navigation strategies before hardware implementation
        - Test algorithms in various virtual environments
        - Collect performance data to optimize real-world robot behavior
        
        Competition Preparation:
        1. Simulate competition field layouts
        2. Time trials for different navigation approaches
        3. Analyze tradeoffs between speed and accuracy
        4. Implement debugging tools to diagnose issues in virtual environment
        """)


if __name__ == "__main__":
    # Uncomment to run the simulation
    # robotics_lesson()
    
    # Display sample lesson plans
    print("Sample Educational Resources Included:")
    print("1. Interactive Robot Simulation")
    print("2. Lesson plans for different grade levels")
    print("3. Coding challenges and extensions")
    print("\nTo run simulation, uncomment the robotics_lesson() line\n")
    
    # Preview lesson plans
    LessonPlans.middle_school_intro()
    print("\n" + "-"*50 + "\n")
    LessonPlans.high_school_programming()
    print("\n" + "-"*50 + "\n")
    LessonPlans.competitive_robotics_team()
