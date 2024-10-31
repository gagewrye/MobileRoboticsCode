import numpy as np
import matplotlib.pyplot as plt

# TODO: Add Compass Angle to the Pose class
# TODO: Implement weighted_sensor_fusion function to correct the pose angle

class Pose:
    def __init__(self, x, y, theta, compass_angle=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.compass_angle = compass_angle

def forward_kinematics(pose, v_left, v_right, dt, track_width):
    v = (v_left + v_right) / 2
    theta_dot = (v_right - v_left) / track_width

    pose.x += v * np.cos(pose.theta) * dt
    pose.y += v * np.sin(pose.theta) * dt
    pose.theta += theta_dot * dt

    return pose

def position_control(pose, goal, k_position, k_orientation, max_linear_velocity, max_angular_velocity, track_width, goal_threshold=0.01):
    d = np.hypot(goal[0] - pose.x, goal[1] - pose.y)

    if d < goal_threshold:
        return 0, 0

    v = k_position * d
    v = min(v, max_linear_velocity)

    angle_to_goal = np.arctan2(goal[1] - pose.y, goal[0] - pose.x)
    theta_error = angle_to_goal - pose.theta
    theta_dot = k_orientation * theta_error
    theta_dot = min(theta_dot, max_angular_velocity)

    v_left = v - theta_dot * track_width / 2
    v_right = v + theta_dot * track_width / 2

    return v_left, v_right

# Simulation parameters
track_width = 0.1  # meters
dt = 0.1  # time step
time_end = 20  # seconds

def simulate(goal, all_trajectories: list, k_position = 0.5, k_orientation = 0.5, max_linear_velocity = 0.5, max_angular_velocity = 0.5, left_noise=0.0, right_noise=0.0):
    pose = Pose(0, 0, 0)
    t = 0
    trajectory = []

    v_left, v_right = 0, 0
    while t < time_end:
        v_left += np.random.normal(-left_noise, left_noise)
        v_right += np.random.normal(-right_noise, right_noise)
        pose = forward_kinematics(pose, v_left, v_right, dt, track_width)
        v_left, v_right = position_control(
            pose, goal, k_position, k_orientation, max_linear_velocity, max_angular_velocity, track_width
        )
        trajectory.append((pose.x, pose.y))
        t += dt
    
    all_trajectories.append({
                        'trajectory': trajectory,
                        'goal': goal,
                        'k_position': k_position,
                        'k_orientation': k_orientation,
                        'max_linear_velocity': max_linear_velocity,
                        'max_angular_velocity': max_angular_velocity,
                        'left_noise': left_noise,
                        'right_noise': right_noise
                    })


goal = (6, 4)
all_trajectories = []
simulate(goal, all_trajectories)

"""
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
"""

# Noisy Simulation
noise_simulation = True
noise_level = 0.01
simulate(goal, all_trajectories, left_noise=noise_level)
simulate(goal, all_trajectories, right_noise=noise_level)
simulate(goal, all_trajectories, left_noise=noise_level, right_noise=noise_level)


# Plot all trajectories
plt.figure(figsize=(10, 10))
for traj_info in all_trajectories:
    traj = np.array(traj_info['trajectory'])
    if noise_simulation:
        plt.plot(traj[:, 0], traj[:, 1], label=f"Goal: {traj_info['goal']}, Left Noise: {traj_info['left_noise']}, Right Noise: {traj_info['right_noise']}")
    else:
        plt.plot(traj[:, 0], traj[:, 1], label=f"Goal: {traj_info['goal']}, Kp: {traj_info['k_position']}, Ko: {traj_info['k_orientation']}, Max V: {traj_info['max_linear_velocity']}, Max ω: {traj_info['max_angular_velocity']}")

plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
if noise_simulation:
    plt.title('Robot Trajectories for Different Noise Levels')
else:
    plt.title('Robot Trajectories for Different Control Parameters')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid(True)
plt.tight_layout()

plt.show()