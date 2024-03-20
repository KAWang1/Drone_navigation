import math

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return yaw  # return only yaw angle (in radians)

def calculate_angle_to_destination(start_pos, destination_pos):
    """
    Calculate the angle the car needs to face to move straight towards the destination coordinates.
    """
    dx = destination_pos[0] - start_pos[0]
    dy = destination_pos[1] - start_pos[1]
    angle_to_destination = math.atan2(dy, dx)
    return angle_to_destination

def calculate_turn_angle(current_angle, angle_to_destination):
    """
    Calculate the angle the car needs to turn to align itself with the desired direction.
    """
    turn_angle = angle_to_destination - current_angle
    while turn_angle > math.pi:
        turn_angle -= 2 * math.pi
    while turn_angle < -math.pi:
        turn_angle += 2 * math.pi
    return turn_angle

# Example usage:
# Assuming 'start_pos' and 'destination_pos' are tuples of (x, y, z) coordinates,
# and 'current_orientation' is a tuple of quaternion (x, y, z, w) representing the current orientation.

start_pos = (0, 0, 0)
destination_pos = (5, 5, 0)
current_orientation = (0, 0, 0, 1)  # Example quaternion representing no rotation (identity quaternion)

current_angle = quaternion_to_euler(*current_orientation)
angle_to_destination = calculate_angle_to_destination(start_pos, destination_pos)
turn_angle = calculate_turn_angle(current_angle, angle_to_destination)

print("Current angle:", math.degrees(current_angle), "degrees")
print("Angle to destination:", math.degrees(angle_to_destination), "degrees")
print("Turn angle:", math.degrees(turn_angle), "degrees")


def euler_to_quaternion(yaw):
    """
    Convert Euler angles (yaw) to quaternion (x, y, z, w).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0, 0, sy, cy)  # Assuming no roll or pitch, so x, y = 0

# Example usage:
yaw_angle = math.radians(45)  # Example yaw angle in degrees
quaternion = euler_to_quaternion(yaw_angle)
print("Quaternion (x, y, z, w):", quaternion)
