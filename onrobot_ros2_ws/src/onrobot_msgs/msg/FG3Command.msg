# 3FG15/3FG25 Command Message

# Command types
uint8 CMD_GRIP = 1           # Grip with force
uint8 CMD_MOVE = 2           # Move without force
uint8 CMD_STOP = 4           # Stop motion
uint8 CMD_FLEXIBLE_GRIP = 5  # Flexible grip (max 140N, 8kg payload)

# Grip types
uint8 GRIP_EXTERNAL = 0      # External grip (default)
uint8 GRIP_INTERNAL = 1      # Internal grip

# Command
uint8 command                # Command type (1/2/4/5)

# Target values
float32 target_diameter      # Target diameter in mm (0-150 for 3FG15, 0-250 for 3FG25)
float32 target_force         # Target force in % (0-100%, NOT Newtons!)
uint8 grip_type              # Grip type (0=external, 1=internal)
