#!/usr/bin/env bash
# Simulated drone for bench-testing the ground station without hardware.
# Runs inside the airside image: rosbridge, the sim camera + capture service,
# and fake MAVROS topics (position, altitude, armed state, battery, IMU).
#
# From the monorepo root:
#   docker run --rm -d --name drone-sim -p 9090:9090 -v "$PWD/ims/test_scripts:/t" warg/airside:latest bash /t/sim_drone.sh
# Simulate losing the drone's data:   docker exec drone-sim pkill -f "topic pub"
# Stop:                               docker rm -f drone-sim

ros2 run rosbridge_server rosbridge_websocket --ros-args -p max_message_size:=10000000 &
ros2 run camera camera --ros-args -p camera_type:=sim &
ros2 run camera triggered_image_publisher &

ros2 topic pub -r 5 /mavros/global_position/global sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: gps}, latitude: 43.4339, longitude: -80.5785, altitude: 330.0}" \
  --qos-reliability best_effort > /dev/null &
ros2 topic pub -r 5 /mavros/global_position/rel_alt std_msgs/msg/Float64 "{data: 30.0}" > /dev/null &
ros2 topic pub -r 1 /mavros/state mavros_msgs/msg/State "{connected: true, armed: true, mode: GUIDED}" > /dev/null &
ros2 topic pub -r 1 /mavros/battery sensor_msgs/msg/BatteryState "{percentage: 0.87}" > /dev/null &
ros2 topic pub -r 10 /mavros/imu/data sensor_msgs/msg/Imu "{header: {frame_id: imu}, orientation: {w: 1.0}}" \
  --qos-reliability best_effort > /dev/null &

wait
