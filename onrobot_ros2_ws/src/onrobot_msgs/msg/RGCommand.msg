# Command type
uint8 GRIP = 1
uint8 GRIP_WITH_OFFSET = 2
uint8 STOP = 3

uint8 command

# Parameters
float32 target_width  # mm
float32 target_force  # N (0-400 for RG2, 0-1200 for RG6)
