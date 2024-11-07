uint8 DJI_WAYPOINT_V2_ACTION_INTERVAL_TYPE_TIME     = 1  # The action will be repeated after a particular period of time.
uint8 DJI_WAYPOINT_V2_ACTION_INTERVAL_TYPE_DISTANCE = 2  # The action will be repeated after a particular distance.
uint8 DJI_WAYPOINT_V2_ACTION_INTERVAL_TYPE_UNKNOWN  = 255 # Unknown action trigger type.

uint16 start_index # It determines the index of the waypoint at which the trigger starts.
uint16 interval 
uint8 action_interval_type # The type of interval trigger.
