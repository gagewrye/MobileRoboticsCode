def weighted_sensor_fusion(kinematics_angle, compass_angle, alpha):
    """
    Perform weighted sensor fusion to estimate the corrected pose angle.

    Parameters:
    kinematics_angle (float): The angle calculated from kinematics.
    compass_angle (float): The angle measured from the compass.
    alpha (float): The weight for the kinematics angle. (0 <= alpha <= 1)

    Returns:
    float: The corrected pose angle.
    """
    if not (0 <= alpha <= 1):
        raise ValueError("Alpha must be between 0 and 1")

    corrected_angle = alpha * kinematics_angle + (1 - alpha) * compass_angle
    return corrected_angle

# Example usage
kinematics_angle = 30.0  # Example kinematics angle in degrees
compass_angle = 35.0     # Example compass angle in degrees
alpha = 0.7              # Weight for kinematics angle

corrected_angle = weighted_sensor_fusion(kinematics_angle, compass_angle, alpha)
print(f"Corrected Pose Angle: {corrected_angle} degrees")