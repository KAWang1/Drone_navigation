import math

def calculate_angle_to_destination(start_pos, destination_pos):
    """
    Calculate the angle the car needs to face to move straight towards the destination coordinates.
    """
    dx = destination_pos[0] - start_pos[0]
    dy = destination_pos[1] - start_pos[1]
    angle_to_destination = math.atan2(dy, dx)
    return angle_to_destination

#SOMEWHERE IN MATHEWMOTOR (Before movement method)
start_pos = (self.latest_pose.pose.position.x, self.latest_pose.pose.position.y)
destination_pos = (5,5) #Can be anything

#Destination angle to be used in car orientation method
angle_to_destination = calculate_angle_to_destination(start_pos, destination_pos)

#FOR BUFFER CODE
cur_xyz = start_pos #Initialize cur_xyz

