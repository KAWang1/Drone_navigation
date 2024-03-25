#creates the boundary for the box and the buffer for the car to stop within



buffer = 0.25
dest_buffer = 0.15

# Coordinates of the box
LL = (-1.59, -4.13)
UL = (-2.71, -3.64)
LR = (-1.5, -2.4)
UR = (-2.65, -2.33)



# create a buffer around the boundary for the car to stop (Minus buffer from box)
def create_boundary_buffer():

    boundary_buffer = {
        "LL": (LL[0] - buffer, LL[1] - buffer),
        "UL": (UL[0] - buffer, UL[1] + buffer),
        "LR": (LR[0] + buffer, LR[1] - buffer),
        "UR": (UR[0] + buffer, UR[1] + buffer)
    }
    return boundary_buffer




#stop the car if in either buffer zone by returning true if it needs to stop and false if it can continue
def should_stop(curr_xyz, buffer, dest_buffer):
    x, y = curr_xyz


    # Check if the car is within the boundary buffer zone
    
    buffer_x_min, buffer_y_min = buffer["LL"]
    buffer_x_max, buffer_y_max = buffer["UR"]
    
    within_buffer = (
    
        x >= buffer_x_min and x <= buffer_x_max and
        y >= buffer_y_min and y <= buffer_y_max
        
    )

    # Check if the car is within the destination buffer zone
    dest_x_min, dest_y_min = dest_buffer["x"]
    dest_x_max, dest_y_max = dest_buffer["y"]
    within_dest_buffer = (
    
        x >= dest_x_min and x <= dest_x_max and
        y >= dest_y_min and y <= dest_y_max
        
    )

    # Return True if the car is within either buffer zone, False otherwise
    return within_buffer or within_dest_buffer





#  create a destination buffer for the car to stop within at its destination or close to the destination
def create_stop_buffer(curr_xyz, dest_xyz, orientation, dest_buffer):
    stop_buffer = {
    
        "x": (dest_xyz[0] - dest_buffer, dest_xyz[0] + dest_buffer),
        "y": (dest_xyz[1] - dest_buffer, dest_xyz[1] + dest_buffer)
        
    }

    if orientation == "right":
        stop_buffer["z"] = (dest_xyz[0] - dest_buffer, dest_xyz[0] + dest_buffer)
        stop_buffer["w"] = (dest_xyz[1] - dest_buffer, dest_xyz[1] + dest_buffer)
    elif orientation == "left":
        stop_buffer["z"] = (dest_xyz[0] - dest_buffer, dest_xyz[0] + dest_buffer)
        stop_buffer["w"] = (dest_xyz[1] - dest_buffer, dest_xyz[1] + dest_buffer)
    elif orientation == "forward":
        stop_buffer["-z"] = (dest_xyz[0] - dest_buffer, dest_xyz[0] + dest_buffer)
        stop_buffer["-w"] = (dest_xyz[1] - dest_buffer, dest_xyz[1] + dest_buffer)
    elif orientation == "backward":
        stop_buffer["+z"] = (dest_xyz[0] - dest_buffer, dest_xyz[0] + dest_buffer)
        stop_buffer["-w"] = (dest_xyz[1] - dest_buffer, dest_xyz[1] + dest_buffer)


    return stop_buffer




'''
# Example usage
curr_xyz = (0, 0)
dest_xyz = (1, 2)  
orientation = "forward"

boundary_buffer = create_boundary_buffer()
stop_buffer = create_stop_buffer(curr_xyz, dest_xyz, orientation, dest_buffer)

print("Boundary Buffer:", boundary_buffer)
print("Stop Buffer:", stop_buffer)



def update_position():
    # Update curr_xyz with new position data
    curr_xyz = (new_x, new_y)

    # Call should_stop function with updated curr_xyz
    stop = should_stop(curr_xyz, buffer, dest_buffer)

    if stop:
        print("Stop the car!")
    else:
        print("Continue moving.")

# Example usage
update_position()
'''