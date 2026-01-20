# Command types
uint8 GRIP = 1
uint8 MOVE = 2
uint8 STOP = 3
uint8 FLEXIBLE_GRIP = 5

uint8 command

# Parameters
float32 target_diameter  # mm
float32 target_force     # percentage (0-100)

# Grip type
uint8 GRIP_EXTERNAL = 0
uint8 GRIP_INTERNAL = 1
uint8 grip_type
