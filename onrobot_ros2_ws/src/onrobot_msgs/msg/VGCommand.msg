# Control mode per channel
uint8 MODE_RELEASE = 0
uint8 MODE_GRIP = 1
uint8 MODE_IDLE = 2

# Channel commands (array length depends on gripper model)
uint8[] control_mode      # One per channel
float32[] target_vacuum   # Percentage (0-80%), one per channel
bool[] require_grip       # If true, must reach target within 3 seconds
bool[] ignore_channel     # If true, command ignored for this channel
