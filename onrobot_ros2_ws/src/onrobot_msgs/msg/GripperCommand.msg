# Generic gripper command types
uint8 INITIALIZE = 0
uint8 GRIP = 1
uint8 RELEASE = 2
uint8 STOP = 3
uint8 CALIBRATE = 4

uint8 command

# Generic parameters (interpretation depends on gripper type)
float32 param1
float32 param2
float32 param3
