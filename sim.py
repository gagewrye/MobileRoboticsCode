import numpy as np
import matplotlib.pyplot as plt
from map import OccupancyMap as Map
from typing import Tuple

# Simulation parameters
track_width = 0.1  # meters
dt = 0.1  # time step
timeout = 30  # seconds
goal = (6, 4)
goals = [(2, 4), (5, 5), (7, 3), (5, 9), (10, 1)]
noise_level = 0.01

edmunds_start = (15, 2)
edmunds_goal = (3, 3)

# Simulation Type
compass_simulation = False
noise_simulation = False
multi_goal_simulation = False
edmunds_simulation = True

class Pose:
    def __init__(self, x, y, theta, compass_angle=None, alpha=0.5):
        self.x = x
        self.y = y
        self.theta = theta
        self.compass_angle = compass_angle
        self.alpha = alpha

def forward_kinematics(pose: Pose, v_left, v_right, dt, track_width):
    v = (v_left + v_right) / 2
    theta_dot = (v_right - v_left) / track_width

    pose.x += v * np.cos(pose.theta) * dt
    pose.y += v * np.sin(pose.theta) * dt
    
    pose.theta += theta_dot * dt
    if pose.compass_angle is not None:
        pose.theta = pose.alpha * pose.theta + (1 - pose.alpha) * pose.compass_angle

def position_control(pose: Pose, goal, k_position, k_orientation, max_linear_velocity, max_angular_velocity, track_width, goal_threshold=0.01):
    d = np.hypot(goal[0] - pose.x, goal[1] - pose.y)
    if d < goal_threshold:
        return 0, 0

    v = k_position * d
    v = min(v, max_linear_velocity)
    v = max(v, -max_linear_velocity)

    angle_to_goal = np.arctan2(goal[1] - pose.y, goal[0] - pose.x)
    theta_error = np.arctan2(np.sin(angle_to_goal - pose.theta), np.cos(angle_to_goal - pose.theta))
    theta_dot = k_orientation * theta_error
    theta_dot = min(theta_dot, max_angular_velocity)

    v_left = v - theta_dot * track_width / 2
    v_right = v + theta_dot * track_width / 2

    return v_left, v_right

def simulate(goal, all_trajectories: list, k_position = 0.5, k_orientation = 0.5, max_linear_velocity = 0.5, max_angular_velocity = 0.5, left_noise=0.0, right_noise=0.0, alpha=0.5, compass_noise=None):
    t = 0
    v_left, v_right = 0, 0
    trajectory = []
    if compass_noise is not None:
        compass_angle = 0
    else:
        compass_angle = None
    
    percieved_pose = Pose(0, 0, 0, compass_angle=compass_angle, alpha=alpha)
    actual_pose = Pose(0, 0, 0)
    
    while t < timeout:
        forward_kinematics(actual_pose, v_left, v_right, dt, track_width)
        
        if compass_angle is not None:
            # Calculate Compass Angle
            compass_angle = actual_pose.theta
            if compass_noise != 0:
                compass_angle += np.random.normal(0, compass_noise)
            percieved_pose.compass_angle = compass_angle
        
        # Add noise
        noisy_v_left = v_left + np.random.normal(0, left_noise)
        noisy_v_right = v_right + np.random.normal(0, right_noise)
        
        forward_kinematics(percieved_pose, noisy_v_left, noisy_v_right, dt, track_width)
        v_left, v_right = position_control(
            percieved_pose, goal, k_position, k_orientation, max_linear_velocity, max_angular_velocity, track_width
        )
        
        trajectory.append((actual_pose.x, actual_pose.y))
        t += dt
    
    if compass_noise is None:
        compass_noise = "No Compass"
    all_trajectories.append({
                        'trajectory': trajectory,
                        'goal': goal,
                        'k_position': k_position,
                        'k_orientation': k_orientation,
                        'max_linear_velocity': max_linear_velocity,
                        'max_angular_velocity': max_angular_velocity,
                        'left_noise': left_noise,
                        'right_noise': right_noise,
                        'compass_noise': compass_noise,
                        'alpha': alpha
                    })
def simulate_sequence(goals, all_trajectories: list, k_position=0.5, k_orientation=0.5,
                      max_linear_velocity=0.5, max_angular_velocity=0.5, left_noise=0.0,
                      right_noise=0.0, alpha=0.5, compass_noise=None):
    v_left, v_right = 0, 0
    goal_threshold = 0.05
    if compass_noise is not None:
        compass_angle = 0
    else:
        compass_angle = None

    percieved_pose = Pose(0, 0, 0, compass_angle=compass_angle, alpha=alpha)
    actual_pose = Pose(0, 0, 0)

    for goal in goals:
        t = 0
        trajectory = []
        
        # Reset the perceived pose to match the actual pose for each goal
        percieved_pose.x, percieved_pose.y, percieved_pose.theta = actual_pose.x, actual_pose.y, actual_pose.theta

        while (np.hypot(goal[0] - percieved_pose.x, goal[1] - percieved_pose.y) > goal_threshold) and t < timeout:
            # Update actual robot's motion
            forward_kinematics(actual_pose, v_left, v_right, dt, track_width)
            
            if compass_angle is not None:
                # Simulate and smooth compass readings
                compass_angle = actual_pose.theta
                if compass_noise:
                    compass_angle += np.random.normal(0, compass_noise)
                percieved_pose.compass_angle = compass_angle

            # Add noise to wheel velocities
            noisy_v_left = v_left + np.random.normal(0, left_noise)
            noisy_v_right = v_right + np.random.normal(0, right_noise)

            # Update perceived robot's motion
            forward_kinematics(percieved_pose, noisy_v_left, noisy_v_right, dt, track_width)

            # Update control commands
            v_left, v_right = position_control(
                percieved_pose, goal, k_position, k_orientation,
                max_linear_velocity, max_angular_velocity, track_width
            )

            # Record the actual trajectory
            trajectory.append((actual_pose.x, actual_pose.y))
            t += dt

        print(f"Goal: {goal}, Time: {t}")
        # Log trajectory information for each goal
        if compass_noise is None:
            compass_noise = "No Compass"
        all_trajectories.append({
            'trajectory': trajectory,
            'goal': goal,
            'k_position': k_position,
            'k_orientation': k_orientation,
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
            'left_noise': left_noise,
            'right_noise': right_noise,
            'compass_noise': compass_noise,
            'alpha': alpha
        })
def simulate_edmunds(start: Tuple, goal: Tuple, all_trajectories: list, k_position = 0.5, k_orientation = 0.5, max_linear_velocity = 0.5, max_angular_velocity = 0.5, left_noise=0.0, right_noise=0.0, alpha=0.5, compass_noise=None):
    map = Map()
    map.build_edmunds105(start)
    paths = map.bfs(start, goal)
    paths = paths[1:]  # Remove the start position
    t = 0
    v_left, v_right = 0, 0
    trajectory = []
    goal_threshold = 0.5
    
    # Initialize the robot's pose
    compass_angle = 0 if compass_noise is not None else None
    percieved_pose = Pose(start[0], start[1], 0, compass_angle=compass_angle, alpha=alpha)
    actual_pose = Pose(start[0], start[1], 0)
    
    if paths:  # Ensure paths exist
        first_waypoint = paths[0]
        dx, dy = first_waypoint[0] - start[0], first_waypoint[1] - start[1]
        initial_theta = np.arctan2(dy, dx)
        percieved_pose.theta = initial_theta
        actual_pose.theta = initial_theta
    else:
        return

    for path in paths:
        print(f"Goal: {path}")
        print(f"Percieved Loc: {percieved_pose.x, percieved_pose.y}")
        print(f"Actual Loc: {actual_pose.x, actual_pose.y}")
        t = 0
        trajectory = []
        
        while (np.hypot(path[0] - percieved_pose.x, path[1] - percieved_pose.y) > goal_threshold) and t < timeout:
            # Update actual robot's motion
            forward_kinematics(actual_pose, v_left, v_right, dt, track_width)
            if compass_angle is not None:
                # Simulate and smooth compass readings
                compass_angle = actual_pose.theta
                if compass_noise:
                    compass_angle += np.random.normal(0, compass_noise)
                percieved_pose.compass_angle = compass_angle

            # Add noise to wheel velocities
            noisy_v_left = v_left + np.random.normal(0, left_noise)
            noisy_v_right = v_right + np.random.normal(0, right_noise)

            # Update perceived robot's motion
            forward_kinematics(percieved_pose, noisy_v_left, noisy_v_right, dt, track_width)

            # Update control commands
            v_left, v_right = position_control(
                percieved_pose, path, k_position, k_orientation,
                max_linear_velocity, max_angular_velocity, track_width
            )

            # Record the actual trajectory
            trajectory.append((actual_pose.x, actual_pose.y))
            t += dt

        # Log trajectory information for each goal
        if compass_noise is None:
            compass_noise = "No Compass"
        all_trajectories.append({
            'trajectory': trajectory,
            'goal': path,
            'k_position': k_position,
            'k_orientation': k_orientation,
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
            'left_noise': left_noise,
            'right_noise': right_noise,
            'compass_noise': compass_noise,
            'alpha': alpha
        })
        map.travel_to(round(actual_pose.x), round(actual_pose.y))
    map.show()

all_trajectories = []

# Noise Simulation
if noise_simulation:
    simulate(goal, all_trajectories)
    simulate(goal, all_trajectories, left_noise=noise_level)
    simulate(goal, all_trajectories, right_noise=noise_level)
    simulate(goal, all_trajectories, left_noise=noise_level, right_noise=noise_level)

# Compass Noise Simulation
elif compass_simulation:
    simulate(goal, all_trajectories)
    motor_noise = 0.05
    simulate(goal, all_trajectories, left_noise=motor_noise, right_noise=motor_noise)
    alpha = 0
    compass_noise = 0.1
    simulate(goal, all_trajectories, left_noise=motor_noise, right_noise=motor_noise, alpha=alpha, compass_noise=compass_noise)
    alpha = 0.1
    simulate(goal, all_trajectories, left_noise=motor_noise, right_noise=motor_noise, alpha=alpha, compass_noise=compass_noise)
    alpha = 0.5
    simulate(goal, all_trajectories, left_noise=motor_noise, right_noise=motor_noise, alpha=alpha, compass_noise=compass_noise)
    alpha = 0.0
    compass_noise = 0.5
    simulate(goal, all_trajectories, left_noise=motor_noise, right_noise=motor_noise, alpha=alpha, compass_noise=compass_noise)
    alpha = 0.1
    simulate(goal, all_trajectories, left_noise=motor_noise, right_noise=motor_noise, alpha=alpha, compass_noise=compass_noise)

elif edmunds_simulation:
    multi_goal_simulation = True
    simulate_edmunds(edmunds_start, edmunds_goal, all_trajectories)

# Multi-Goal Simulation
elif multi_goal_simulation:
    simulate_sequence(goals, all_trajectories, left_noise=noise_level, right_noise=noise_level, compass_noise=noise_level)

# Ideal Simulation Tests
else:
    simulate(goal, all_trajectories)
    simulate(goal, all_trajectories, k_position=2.0, k_orientation=2.0)
    simulate(goal, all_trajectories, max_linear_velocity=2.0, max_angular_velocity=2.0)
    simulate(goal, all_trajectories, max_linear_velocity=5.0, max_angular_velocity=5.0)
    simulate(goal, all_trajectories, max_linear_velocity=5.0)

    goal = (10, 1)
    simulate(goal, all_trajectories)
    simulate(goal, all_trajectories, max_linear_velocity=2.0, max_angular_velocity=2.0)
    simulate(goal, all_trajectories, max_linear_velocity=5.0)

    goal = (1, 5)
    simulate(goal, all_trajectories)
    simulate(goal, all_trajectories, k_position=2.0, k_orientation=2.0)
    simulate(goal, all_trajectories, max_linear_velocity=2.0, max_angular_velocity=2.0)

# Plot all trajectories
plt.figure(figsize=(10, 10))
goal = None
first = True

for traj_info in all_trajectories:
    traj = np.array(traj_info['trajectory'])
    # Add Goals to the legend
    if traj_info['goal'] != goal and not multi_goal_simulation:
        goal = traj_info['goal']
        plt.plot([], [], ' ', label=f"$\\bf{{Goal: {goal}}}$")  
    if first and noise_simulation or compass_simulation:
        plt.plot(traj[:, 0], traj[:, 1], label=f"Ideal Trajectory")
        first = False
    elif noise_simulation or compass_simulation:
        if compass_simulation:
            plt.title('Robot Trajectories with Compass')
            if traj_info['compass_noise'] == "No Compass":
                plt.plot(traj[:, 0], traj[:, 1], label=f"No Compass, Motor Noise: {traj_info['left_noise']}")
            else:
                plt.plot(traj[:, 0], traj[:, 1], label=f"Compass Noise: {traj_info['compass_noise']}, Alpha: {traj_info['alpha']}, Motor Noise: {traj_info['left_noise']}")
        else:
            plt.title('Robot Trajectories with Noise')
            plt.plot(traj[:, 0], traj[:, 1], label=f"Goal: {traj_info['goal']}, L_Noise: {traj_info['left_noise']}, R_Noise: {traj_info['right_noise']}")
    elif multi_goal_simulation:
        plt.title('Robot Trajectory with Multiple Goals')
        plt.plot(traj[:, 0], traj[:, 1], label=f"$\\bf{{Goal: {traj_info['goal']}}}$")
    else:
        plt.title('Robot Trajectories')
        plt.plot(traj[:, 0], traj[:, 1], label=f"Goal: {traj_info['goal']}, Kp: {traj_info['k_position']}, Ko: {traj_info['k_orientation']}, Max V: {traj_info['max_linear_velocity']}, Max ω: {traj_info['max_angular_velocity']}")

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid(True)
plt.tight_layout()
plt.show()