std_msgs/Header header

# Status flags
bool busy
bool grip_detected
bool force_grip_detected
bool calibrated

# Position and force
float32 raw_diameter              # mm
float32 diameter_with_offset      # mm
float32 force_applied             # percentage (0-100)
float32 actual_width_with_offset  # mm

# Finger configuration
float32 finger_length    # mm
uint8 finger_position    # 1, 2, or 3
float32 fingertip_offset # mm

# Diameter range
float32 min_diameter  # mm
float32 max_diameter  # mm
