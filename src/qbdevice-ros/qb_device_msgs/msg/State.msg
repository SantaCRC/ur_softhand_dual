# State message valid for either qbhand or qbmove

# either qbhand or qbmove:
#  - motors: position, command in [ticks], velocity in [ticks/s], effort in [mA]
qb_device_msgs/ResourceData[] actuators

# qbhand:
#  - closure: position, command in [0,1], velocity in [percent/s],  effort in [A].
# qbmove:
#  - shaft: position, command in [radians], velocity in [radians/s], effort in [A];
#  - preset: position, command in [0,1], velocity in [percent/s], effort is not used.
qb_device_msgs/ResourceData[] joints

# Reliability of the retrieved measurements
bool is_reliable
int32 consecutive_failures