std_msgs/Header header

# Gripper identification
uint16 product_code
string serial_number

# Status flags
bool busy
bool grip_detected
bool s1_pushed
bool s1_triggered
bool s2_pushed
bool s2_triggered
bool safety_error

# Position and force
float32 actual_width          # mm
float32 actual_width_offset   # mm (with fingertip offset)
float32 actual_depth          # mm
float32 actual_relative_depth # mm
float32 fingertip_offset      # mm

# Configuration
float32 target_width  # mm
float32 target_force  # N
float32 min_width     # mm
float32 max_width     # mm
