#Where current_angle is the current orientation
def calculate_turn_angle(current_position, current_angle, destination):
   
    # Calculate direction vector to destination
    direction = (destination[0] - current_position[0], destination[1] - current_position[1])
   
    # Normalize direction to a unit vector
    direction_length = math.sqrt(direction[0]**2 + direction[1]**2)
    direction = (direction[0] / direction_length, direction[1] / direction_length)
   
    # Calculate desired orientation
    desired_orientation = math.atan2(direction[1], direction[0])
   
    # Calculate turn angle to face the correct orientation
    turn_angle = desired_orientation - current_angle
    while turn_angle > math.pi:
        turn_angle -= 2 * math.pi
    while turn_angle < -math.pi:
        turn_angle += 2 * math.pi
   
    return turn_angle

