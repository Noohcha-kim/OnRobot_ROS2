std_msgs/Header header

# Status flags
bool busy                      # True when motion is ongoing
bool grip_detected             # True when grip is detected
bool force_grip_detected       # True when force grip is detected (3FG specific)

# Current measurements
float32 raw_diameter           # Current raw diameter in mm
float32 diameter_with_offset   # Diameter with fingertip offset in mm
float32 force_applied          # Current force in N

# Limits
float32 min_diameter           # Minimum diameter in mm
float32 max_diameter           # Maximum diameter in mm

# Additional info
uint16 finger_length           # Finger length in 0.1mm
uint16 finger_position         # Finger position
uint16 fingertip_offset        # Fingertip offset in 0.01mm
