std_msgs/Header header

# Force/Torque sensor data in sensor frame
geometry_msgs/Vector3 force   # N
geometry_msgs/Vector3 torque  # Nm

# Status
bool sensor_error
bool zeroed
