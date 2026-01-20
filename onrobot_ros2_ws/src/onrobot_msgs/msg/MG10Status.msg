std_msgs/Header header

# Status flags
bool part_gripped
bool near_part
bool busy
bool magnet_strength_not_reached
bool smart_grip_available
bool smart_grip_failed
bool part_dropped
bool internal_temperature_warning

# Error flags
bool over_heating
bool sensor_target_mismatch
bool no_motor_calibration
bool no_magnet_calibration
bool no_hall_calibration
bool over_current
bool position_error

# Current magnet strength (percentage)
float32 magnet_strength  # 0-100%

# Finger configuration
float32 finger_height  # mm
uint8 finger_type      # 1=None, 2=Protective, 3=Custom
