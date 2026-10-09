/**
 * Every ROS topic the dashboard reads, with the message type rosbridge needs to
 * subscribe. Widgets import names from here rather than repeating string
 * literals, so a typo cannot silently subscribe to a topic that does not exist.
 */
export const TOPICS = {
  heartbeat: { name: '/heartbeat', type: 'std_msgs/Empty' },
  mavrosState: { name: '/mavros/state', type: 'mavros_msgs/State' },
  localPose: { name: '/mavros/local_position/pose', type: 'geometry_msgs/PoseStamped' },
  globalPosition: { name: '/mavros/global_position/global', type: 'sensor_msgs/NavSatFix' },
  // Read only by the session recorder (#189); no widget displays these yet.
  relAlt: { name: '/mavros/global_position/rel_alt', type: 'std_msgs/Float64' },
  imu: { name: '/mavros/imu/data', type: 'sensor_msgs/Imu' },
  camera: { name: '/camera/image_raw', type: 'sensor_msgs/Image' },
  target: { name: '/capture/target_location', type: 'airside_interfaces/Target' },
} as const;
