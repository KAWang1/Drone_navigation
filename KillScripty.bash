!/bin/bash

# Terminate ROS2 processes
pkill -f 'ros2'

# Kill any Python processes related to controlling the RC car
pkill -f 'python MatthewMotor.py'

sleep 2

# Optionally, you can also kill any remaining tmux sessions
tmux list-sessions -F "#S" | while read -r session; do
  tmux kill-session -t "$session"
done

echo "All processes related to controlling the RC car have been terminated."
#!/bin/bash

# Check if tmux is installed
if ! command -v tmux &> /dev/null
then
    echo "tmux could not be found"
    exit
fi

# Get a list of all tmux sessions and kill them
tmux list-sessions -F "#S" | while read -r session; do
  tmux kill-session -t "$session"
done

echo "All tmux sessions have been killed."
