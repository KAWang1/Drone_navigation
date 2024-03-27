!/bin/bash

# Ensure tmux is installed
if ! command -v tmux &> /dev/null
then
    echo "tmux could not be found, please install it first"
    exit
fi

# Start a new tmux session and detach from it
tmux new-session -d -s my_session

# Create a window for the Mocap_Optitrack
tmux new-window -t my_session:1 -n 'Mocap_Optitrack'
tmux send-keys -t my_session:1 'cd ~/ros_ws2' C-m
tmux send-keys -t my_session:1 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t my_session:1 'source install/setup.bash' C-m
tmux send-keys -t my_session:1 'ros2 launch drone.launch.py' C-m

# Create a window for the Mocap_data
tmux new-window -t my_session:2 -n 'Display_data'
tmux send-keys -t my_session:2 'cd ~/ros_ws2' C-m
tmux send-keys -t my_session:2 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t my_session:2 'source install/setup.bash' C-m
tmux send-keys -t my_session:2 'ros2 topic list' C-m
tmux send-keys -t my_session:2 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t my_session:2 'source install/setup.bash' C-m
tmux send-keys -t my_session:2 'ros2 topic list' C-m
tmux send-keys -t my_session:2 'ros2 topic echo /AimsCar/pose' C-m

# Create a window for the Build/Run
tmux new-window -t my_session:3 -n 'Build/Run'
tmux send-keys -t my_session:3 'cd ~/ros_ws2' C-m
tmux send-keys -t my_session:3 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t my_session:3 'source install/setup.bash' C-m
tmux send-keys -t my_session:3 'colcon build --packages-select py_motor' C-m
tmux send-keys -t my_session:3 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t my_session:3 'source install/setup.bash' C-m
tmux send-keys -t my_session:3 'ros2 run py_motor motor' C-m

# Attach to the session
tmux attach -t my_session
