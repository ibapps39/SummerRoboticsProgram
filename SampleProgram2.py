#!/usr/bin/env python3
"""
RoboTrack: A Python-based Robotics Curriculum Tool

This project demonstrates a modular approach to teaching robotics concepts to middle and high
school students. It simulates robot movement and sensors while providing visualization tools
that help students understand programming concepts through interactive exercises.

Author: Ian Brown
Date: March 2025
"""

import time
import math
import random
import argparse
from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Optional, Dict, Any


# ---------- Robot Simulation Classes ----------

@dataclass
class Position:
    """Represents a 2D position with x, y coordinates."""
    x: float = 0.0
    y: float = 0.0

    def distance_to(self, other: 'Position') -> float:
        """Calculate Euclidean distance to another position."""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)


class Sensor:
    """Base class for all robot sensors."""
    
    def __init__(self, name: str):
        self.name = name
        self._value = 0
        
    def read(self) -> float:
        """Read current sensor value."""
        return self._value
    
    def update(self, environment: Dict[str, Any], robot_position: Position) -> None:
        """Update sensor based on environment and robot position."""
        pass


class DistanceSensor(Sensor):
    """Simulates an ultrasonic or IR distance sensor."""
    
    def __init__(self, name: str, max_range: float = 100.0, noise_factor: float = 0.05):
        super().__init__(name)
        self.max_range = max_range
        self.noise_factor = noise_factor
        
    def update(self, environment: Dict[str, Any], robot_position: Position) -> None:
        """Update distance reading based on obstacles in environment."""
        obstacles = environment.get('obstacles', [])
        min_distance = self.max_range
        
        for obstacle in obstacles:
            distance = obstacle.distance_to(robot_position)
            if distance < min_distance:
                min_distance = distance
        
        # Add some noise to simulate real-world sensor behavior
        noise = random.uniform(-self.noise_factor, self.noise_factor) * min_distance
        self._value = max(0, min(self.max_range, min_distance + noise))


class LightSensor(Sensor):
    """Simulates a light sensor that detects brightness levels."""
    
    def __init__(self, name: str, max_value: float = 100.0):
        super().__init__(name)
        self.max_value = max_value
        
    def update(self, environment: Dict[str, Any], robot_position: Position) -> None:
        """Update light reading based on light sources in environment."""
        light_sources = environment.get('light_sources', [])
        light_level = environment.get('ambient_light', 10.0)
        
        for source in light_sources:
            distance = source['position'].distance_to(robot_position)
            intensity = source['intensity']
            
            # Light diminishes with square of distance
            if distance > 0:
                light_level += intensity / (distance ** 2)
        
        self._value = min(self.max_value, light_level)


class LineSensor(Sensor):
    """Simulates a line-following sensor."""
    
    def update(self, environment: Dict[str, Any], robot_position: Position) -> None:
        """Update line sensor reading based on lines in environment."""
        lines = environment.get('lines', [])
        self._value = 0  # Default: no line detected
        
        for line in lines:
            start, end = line['start'], line['end']
            
            # Calculate closest point on line segment to robot position
            line_vec = Position(end.x - start.x, end.y - start.y)
            line_length_squared = line_vec.x**2 + line_vec.y**2
            
            if line_length_squared == 0:
                # Line segment is actually a point
                closest_point = start
            else:
                # Calculate projection of robot position onto line segment
                t = max(0, min(1, ((robot_position.x - start.x) * line_vec.x + 
                                  (robot_position.y - start.y) * line_vec.y) / 
                                 line_length_squared))
                closest_point = Position(
                    start.x + t * line_vec.x,
                    start.y + t * line_vec.y
                )
            
            # Check if robot is close enough to line
            distance = robot_position.distance_to(closest_point)
            if distance < line['width'] / 2:
                # Stronger reading the closer to center
                self._value = 100 * (1 - distance / (line['width'] / 2))
                break


class Motor:
    """Simulates a DC motor with speed control."""
    
    def __init__(self, name: str, max_speed: float = 100.0):
        self.name = name
        self.max_speed = max_speed
        self._speed = 0
        self._direction = 1  # 1 for forward, -1 for reverse
        
    def set_speed(self, speed: float) -> None:
        """Set motor speed (-max_speed to max_speed)."""
        if speed < 0:
            self._direction = -1
            self._speed = min(abs(speed), self.max_speed)
        else:
            self._direction = 1
            self._speed = min(speed, self.max_speed)
    
    def get_speed(self) -> float:
        """Get current motor speed with direction."""
        return self._speed * self._direction


class Robot:
    """Simulates a basic educational robot with sensors and motors."""
    
    def __init__(self, name: str = "RoboStudent"):
        self.name = name
        self.position = Position(0, 0)
        self.heading = 0  # degrees, 0 is East, 90 is North
        self.sensors = {}
        self.motors = {}
        self.trail = []  # Tracks robot movement for visualization
        
    def add_sensor(self, sensor: Sensor) -> None:
        """Add a sensor to the robot."""
        self.sensors[sensor.name] = sensor
        
    def add_motor(self, motor: Motor) -> None:
        """Add a motor to the robot."""
        self.motors[motor.name] = motor
        
    def get_sensor_reading(self, sensor_name: str) -> float:
        """Get reading from a specific sensor."""
        sensor = self.sensors.get(sensor_name)
        if sensor:
            return sensor.read()
        else:
            raise ValueError(f"Sensor '{sensor_name}' not found on robot {self.name}")
    
    def set_motor_speed(self, motor_name: str, speed: float) -> None:
        """Set speed for a specific motor."""
        motor = self.motors.get(motor_name)
        if motor:
            motor.set_speed(speed)
        else:
            raise ValueError(f"Motor '{motor_name}' not found on robot {self.name}")
    
    def update(self, environment: Dict[str, Any], time_step: float = 0.1) -> None:
        """Update robot state based on motor settings and environment."""
        # Update sensors
        for sensor in self.sensors.values():
            sensor.update(environment, self.position)
        
        # Calculate movement based on motor speeds
        # This is a simple differential drive model
        if 'left_motor' in self.motors and 'right_motor' in self.motors:
            left_speed = self.motors['left_motor'].get_speed()
            right_speed = self.motors['right_motor'].get_speed()
            
            # Average speed determines forward movement
            avg_speed = (left_speed + right_speed) / 2
            
            # Difference in speeds determines rotation
            rotation_speed = (right_speed - left_speed) / 2
            
            # Update heading (in radians for calculation)
            heading_rad = math.radians(self.heading)
            heading_rad += rotation_speed * time_step * 0.05  # Rotation factor
            self.heading = math.degrees(heading_rad) % 360
            
            # Calculate new position
            self.position.x += avg_speed * time_step * math.cos(heading_rad) * 0.1  # Movement factor
            self.position.y += avg_speed * time_step * math.sin(heading_rad) * 0.1
            
            # Record position for trail visualization
            self.trail.append((self.position.x, self.position.y))


# ---------- Visualization and Simulation Environment ----------

class RobotSimulation:
    """Provides a simulation environment for robots with visualization."""
    
    def __init__(self, width: float = 100.0, height: float = 100.0):
        self.width = width
        self.height = height
        self.robots = []
        self.environment = {
            'obstacles': [],
            'light_sources': [],
            'lines': [],
            'ambient_light': 10.0
        }
        self.fig = None
        self.ax = None
        
    def add_robot(self, robot: Robot) -> None:
        """Add a robot to the simulation."""
        self.robots.append(robot)
        
    def add_obstacle(self, position: Position, radius: float = 5.0) -> None:
        """Add a circular obstacle to the environment."""
        self.environment['obstacles'].append(position)
        
    def add_light_source(self, position: Position, intensity: float = 100.0) -> None:
        """Add a light source to the environment."""
        self.environment['light_sources'].append({
            'position': position,
            'intensity': intensity
        })
        
    def add_line(self, start: Position, end: Position, width: float = 2.0) -> None:
        """Add a line (for line following) to the environment."""
        self.environment['lines'].append({
            'start': start,
            'end': end,
            'width': width
        })
        
    def setup_visualization(self) -> None:
        """Set up matplotlib visualization."""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title('Robot Simulation')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        
    def update_visualization(self) -> None:
        """Update the visualization with current robot and environment state."""
        if not self.fig or not self.ax:
            self.setup_visualization()
            
        self.ax.clear()
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.grid(True)
        self.ax.set_title('Robot Simulation')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        
        # Draw obstacles
        for obstacle in self.environment['obstacles']:
            circle = plt.Circle((obstacle.x, obstacle.y), 5, color='red', alpha=0.7)
            self.ax.add_patch(circle)
            
        # Draw light sources
        for source in self.environment['light_sources']:
            circle = plt.Circle((source['position'].x, source['position'].y), 
                               source['intensity'] / 20, color='yellow', alpha=0.5)
            self.ax.add_patch(circle)
            
        # Draw lines
        for line in self.environment['lines']:
            self.ax.plot([line['start'].x, line['end'].x], 
                        [line['start'].y, line['end'].y], 
                        'k-', linewidth=line['width'])
            
        # Draw robots
        for robot in self.robots:
            # Draw robot body
            circle = plt.Circle((robot.position.x, robot.position.y), 3, color='blue', alpha=0.8)
            self.ax.add_patch(circle)
            
            # Draw heading line
            heading_rad = math.radians(robot.heading)
            end_x = robot.position.x + 8 * math.cos(heading_rad)
            end_y = robot.position.y + 8 * math.sin(heading_rad)
            self.ax.plot([robot.position.x, end_x], [robot.position.y, end_y], 'g-', linewidth=2)
            
            # Draw trail
            if robot.trail:
                trail_x, trail_y = zip(*robot.trail)
                self.ax.plot(trail_x, trail_y, 'b-', alpha=0.3)
                
            # Display robot name
            self.ax.text(robot.position.x, robot.position.y + 10, robot.name, 
                        ha='center', va='center', color='black')
        
        plt.pause(0.01)
        
    def run_simulation(self, time_steps: int = 100, step_duration: float = 0.1) -> None:
        """Run the simulation for a specified number of time steps."""
        for _ in range(time_steps):
            # Update robot states
            for robot in self.robots:
                robot.update(self.environment, step_duration)
            
            # Update visualization
            self.update_visualization()
            
            # Small delay to make visualization smoother
            time.sleep(0.05)


# ---------- Example Curriculum Missions ----------

class RoboticsMission:
    """Base class for robotics curriculum missions."""
    
    def __init__(self, name: str, difficulty: int = 1, description: str = ""):
        self.name = name
        self.difficulty = difficulty
        self.description = description
        self.simulation = None
        
    def setup(self) -> None:
        """Set up the mission environment and robots."""
        pass
        
    def run(self) -> None:
        """Run the mission simulation."""
        pass
        
    def evaluate(self) -> Tuple[bool, str]:
        """Evaluate mission success and provide feedback."""
        return False, "Mission not implemented"


class LineFollowingMission(RoboticsMission):
    """A line following mission for teaching basic sensor-based navigation."""
    
    def __init__(self):
        super().__init__(
            name="Line Following Challenge",
            difficulty=2,
            description="Program the robot to follow a curved line path from start to finish."
        )
        self.robot = None
        self.start_position = Position(10, 50)
        self.end_position = Position(90, 50)
        self.completion_time = 0
        
    def setup(self) -> None:
        """Set up the line following mission."""
        self.simulation = RobotSimulation(100, 100)
        
        # Create curved line path
        points = []
        points.append(self.start_position)
        
        # Middle points for curved line
        points.append(Position(30, 60))
        points.append(Position(50, 40))
        points.append(Position(70, 60))
        
        points.append(self.end_position)
        
        # Add line segments to simulation
        for i in range(len(points) - 1):
            self.simulation.add_line(points[i], points[i+1], width=3.0)
        
        # Add target marker
        self.simulation.add_light_source(self.end_position, intensity=50)
        
        # Create and configure robot
        self.robot = Robot("LineBot")
        self.robot.position = Position(self.start_position.x, self.start_position.y)
        
        # Add sensors
        left_line_sensor = LineSensor("left_line")
        center_line_sensor = LineSensor("center_line")
        right_line_sensor = LineSensor("right_line")
        self.robot.add_sensor(left_line_sensor)
        self.robot.add_sensor(center_line_sensor)
        self.robot.add_sensor(right_line_sensor)
        
        # Add motors
        left_motor = Motor("left_motor")
        right_motor = Motor("right_motor")
        self.robot.add_motor(left_motor)
        self.robot.add_motor(right_motor)
        
        # Add robot to simulation
        self.simulation.add_robot(self.robot)
        
    def run(self) -> None:
        """Run the line following mission with PID control."""
        self.simulation.setup_visualization()
        
        # Simple PID controller constants
        kp = 0.5  # Proportional gain
        ki = 0.1  # Integral gain
        kd = 0.2  # Derivative gain
        
        base_speed = 30
        integral = 0
        last_error = 0
        
        # Run simulation loop
        start_time = time.time()
        for step in range(300):  # Max 300 steps
            # Read line sensors
            left = self.robot.get_sensor_reading("left_line")
            center = self.robot.get_sensor_reading("center_line")
            right = self.robot.get_sensor_reading("right_line")
            
            # Calculate error (negative: line is to the left, positive: line is to the right)
            error = (right - left)
            
            # PID calculation
            integral += error
            integral = max(-100, min(100, integral))  # Anti-windup
            derivative = error - last_error
            last_error = error
            
            # Calculate motor adjustment
            adjustment = kp * error + ki * integral + kd * derivative
            
            # Set motor speeds
            left_speed = base_speed + adjustment
            right_speed = base_speed - adjustment
            
            self.robot.set_motor_speed("left_motor", left_speed)
            self.robot.set_motor_speed("right_motor", right_speed)
            
            # Update simulation
            self.simulation.update_visualization()
            self.robot.update(self.simulation.environment, 0.1)
            
            # Check if we've reached the end
            distance_to_end = self.robot.position.distance_to(self.end_position)
            if distance_to_end < 5:
                print(f"Mission completed in {step} steps!")
                self.completion_time = time.time() - start_time
                break
                
            time.sleep(0.05)
        
        plt.show(block=True)
        
    def evaluate(self) -> Tuple[bool, str]:
        """Evaluate mission success based on completion and time."""
        if self.completion_time > 0:
            return True, f"Mission accomplished! Completion time: {self.completion_time:.2f} seconds"
        else:
            return False, "Mission failed: Robot did not reach the target in time"


class ObstacleAvoidanceMission(RoboticsMission):
    """An obstacle avoidance mission for teaching sensor-based decision making."""
    
    def __init__(self):
        super().__init__(
            name="Obstacle Avoidance Challenge",
            difficulty=3,
            description="Navigate through a field of obstacles to reach the target."
        )
        self.robot = None
        self.start_position = Position(10, 50)
        self.end_position = Position(90, 50)
        
    def setup(self) -> None:
        """Set up the obstacle avoidance mission."""
        self.simulation = RobotSimulation(100, 100)
        
        # Add random obstacles
        random.seed(42)  # For reproducibility
        for _ in range(10):
            x = random.uniform(20, 80)
            y = random.uniform(20, 80)
            # Ensure obstacle doesn't block start or end
            if (Position(x, y).distance_to(self.start_position) > 15 and 
                Position(x, y).distance_to(self.end_position) > 15):
                self.simulation.add_obstacle(Position(x, y))
        
        # Add target marker
        self.simulation.add_light_source(self.end_position, intensity=100)
        
        # Create and configure robot
        self.robot = Robot("SafeBot")
        self.robot.position = Position(self.start_position.x, self.start_position.y)
        self.robot.heading = 0  # Start facing east (toward target)
        
        # Add sensors
        front_sensor = DistanceSensor("front_distance", max_range=30)
        left_sensor = DistanceSensor("left_distance", max_range=20)
        right_sensor = DistanceSensor("right_distance", max_range=20)
        target_sensor = LightSensor("target_sensor")
        self.robot.add_sensor(front_sensor)
        self.robot.add_sensor(left_sensor)
        self.robot.add_sensor(right_sensor)
        self.robot.add_sensor(target_sensor)
        
        # Add motors
        left_motor = Motor("left_motor")
        right_motor = Motor("right_motor")
        self.robot.add_motor(left_motor)
        self.robot.add_motor(right_motor)
        
        # Add robot to simulation
        self.simulation.add_robot(self.robot)
        
    def run(self) -> None:
        """Run the obstacle avoidance mission."""
        self.simulation.setup_visualization()
        
        # State machine for navigation
        states = ["MOVE_TO_TARGET", "AVOID_OBSTACLE"]
        current_state = "MOVE_TO_TARGET"
        
        # Run simulation loop
        for step in range(500):  # Max 500 steps
            # Read sensors
            front_distance = self.robot.get_sensor_reading("front_distance")
            left_distance = self.robot.get_sensor_reading("left_distance")
            right_distance = self.robot.get_sensor_reading("right_distance")
            target_light = self.robot.get_sensor_reading("target_sensor")
            
            # State machine logic
            if current_state == "MOVE_TO_TARGET":
                if front_distance < 15:  # Obstacle ahead
                    current_state = "AVOID_OBSTACLE"
                else:
                    # Calculate direction to target based on light reading
                    # This is a simplified approach - in reality would need more sensors
                    dx = self.end_position.x - self.robot.position.x
                    dy = self.end_position.y - self.robot.position.y
                    target_heading = math.degrees(math.atan2(dy, dx)) % 360
                    
                    # Calculate heading difference (-180 to 180)
                    heading_diff = ((target_heading - self.robot.heading + 180) % 360) - 180
                    
                    # Adjust motors based on heading difference
                    base_speed = 40
                    turn_factor = 0.5
                    
                    left_speed = base_speed - heading_diff * turn_factor
                    right_speed = base_speed + heading_diff * turn_factor
                    
                    self.robot.set_motor_speed("left_motor", left_speed)
                    self.robot.set_motor_speed("right_motor", right_speed)
            
            elif current_state == "AVOID_OBSTACLE":
                if front_distance > 20:  # Clear ahead
                    current_state = "MOVE_TO_TARGET"
                else:
                    # Find clearest direction
                    if left_distance > right_distance:
                        # Turn left
                        self.robot.set_motor_speed("left_motor", 10)
                        self.robot.set_motor_speed("right_motor", 40)
                    else:
                        # Turn right
                        self.robot.set_motor_speed("left_motor", 40)
                        self.robot.set_motor_speed("right_motor", 10)
            
            # Update simulation
            self.simulation.update_visualization()
            self.robot.update(self.simulation.environment, 0.1)
            
            # Check if we've reached the end
            distance_to_end = self.robot.position.distance_to(self.end_position)
            if distance_to_end < 5:
                print(f"Mission completed in {step} steps!")
                break
                
            time.sleep(0.05)
        
        plt.show(block=True)
        
    def evaluate(self) -> Tuple[bool, str]:
        """Evaluate mission success."""
        distance_to_end = self.robot.position.distance_to(self.end_position)
        if distance_to_end < 5:
            collision_count = 0
            for obstacle in self.simulation.environment['obstacles']:
                if obstacle.distance_to(self.robot.position) < 5:
                    collision_count += 1
            
            if collision_count == 0:
                return True, "Perfect navigation! Robot reached target without collisions."
            else:
                return True, f"Robot reached target but had {collision_count} collisions."
        else:
            return False, f"Mission failed: Robot did not reach the target. Distance: {distance_to_end:.2f}"


# ---------- Curriculum Module Example ----------

class RoboticsCurriculum:
    """Organizes robotics missions into a structured curriculum."""
    
    def __init__(self):
        self.missions = []
        
    def add_mission(self, mission: RoboticsMission) -> None:
        """Add a mission to the curriculum."""
        self.missions.append(mission)
        
    def list_missions(self) -> None:
        """Display all available missions."""
        print("\n=== Available Robotics Missions ===")
        for i, mission in enumerate(self.missions, 1):
            print(f"{i}. {mission.name} (Difficulty: {mission.difficulty}/5)")
            print(f"   {mission.description}")
        print("=================================\n")
        
    def run_mission(self, mission_index: int) -> None:
        """Run a specific mission by index."""
        if 0 <= mission_index < len(self.missions):
            mission = self.missions[mission_index]
            print(f"\nStarting mission: {mission.name}")
            print(f"Description: {mission.description}")
            print("Setting up mission environment...")
            
            mission.setup()
            print("Mission setup complete. Running simulation...")
            
            mission.run()
            success, feedback = mission.evaluate()
            
            print(f"Mission {'completed successfully' if success else 'failed'}!")
            print(f"Feedback: {feedback}")
        else:
            print(f"Invalid mission index. Please select a mission between 1 and {len(self.missions)}.")


# ---------- Main Application ----------

def main():
    """Main application entry point."""
    parser = argparse.ArgumentParser(description='RoboTrack: Python-based Robotics Curriculum Tool')
    parser.add_argument('--mission', type=int, help='Mission index to run (1-based)')
    parser.add_argument('--list', action='store_true', help='List all available missions')
    args = parser.parse_args()

    # Create curriculum
    curriculum = RoboticsCurriculum()
    
    # Add available missions
    curriculum.add_mission(LineFollowingMission())
    curriculum.add_mission(ObstacleAvoidanceMission())
    
    # Process command line arguments
    if args.list:
        curriculum.list_missions()
    elif args.mission is not None:
        curriculum.run_mission(args.mission - 1)  # Convert to 0-based index
    else:
        print("RoboTrack: Python-based Robotics Curriculum Tool")
        print("Use --list to see available missions or --mission N to run a specific mission.")
        print("Running demo mission (Line Following)...")
        curriculum.run_mission(0)

if __name__ == "__main__":
    main()
