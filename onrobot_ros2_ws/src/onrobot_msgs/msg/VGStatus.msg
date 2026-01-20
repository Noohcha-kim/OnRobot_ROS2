std_msgs/Header header

# Channel status (A, B, C, D depending on model)
uint8 num_channels  # 2 for VG10/VGC10, 4 for VGP20, 2 for VGP30

# Per-channel grip status - constants
uint8 GRIP_NOT_GRIPPED = 0
uint8 GRIP_DETECTED = 1
uint8 GRIP_TIMEOUT = 2
uint8 GRIP_LOST = 3

uint8[] grip_status      # One per channel

# Per-channel release status
uint8 RELEASE_NOT_RELEASED = 0
uint8 RELEASE_OK = 1
uint8 RELEASE_FAIL = 2

uint8[] release_status   # One per channel

# Vacuum levels (percentage)
float32[] vacuum_level   # One per channel (0-100%)

# Overall status
bool busy
bool psu_error
bool qc_error
