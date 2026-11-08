# Enumerated mission states. Numeric values match MissionStateCode in Python.
uint8 IDLE=0
uint8 MOVING_TO_WAYPOINT=1
uint8 INSPECTING=2
uint8 HAZARD_DETECTED=3
uint8 RETREATING=4
uint8 MISSION_COMPLETE=5
uint8 ABORTED=6

# Simulation or ROS timestamp at which this status was published.
builtin_interfaces/Time stamp

# Machine-readable state code and human-readable equivalent.
uint8 state
string state_name

# Index of the waypoint currently being processed.
uint32 waypoint_index

# Short factual explanation of the latest accepted transition.
string detail
