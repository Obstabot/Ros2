#!/bin/bash

echo "🚀 Launching all turtlebot_controller nodes..."

trap 'echo "🛑 Ctrl+C detected. Killing all..."; pkill -f turtlebot_controller; exit' INT

for i in {1..5}; do
    echo "▶️ Starting turtlebot_$i..."
    ros2 run my_jackal_world turtlebot_controller turtlebot_$i &
    sleep 1
done

wait
