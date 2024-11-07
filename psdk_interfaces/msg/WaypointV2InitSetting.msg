
uint8 DJI_WAYPOINT_V2_MISSION_FINISHED_NO_ACTION = 0  # No further action will be taken.
uint8 DJI_WAYPOINT_V2_MISSION_FINISHED_GO_HOME = 1    # Goes home when the mission is finished. The aircraft will
uint8 DJI_WAYPOINT_V2_MISSION_FINISHED_AUTO_LANDING = 2  # The aircraft will land automatically at the last waypoint.
uint8 DJI_WAYPOINT_V2_MISSION_FINISHED_GO_TO_FIRST_WAYPOINT = 3  # The aircraft will go back to the first waypoint and hover.
uint8 DJI_WAYPOINT_V2_MISSION_FINISHED_CONTINUE_UNTIL_STOP = 4  #  When the aircraft reaches its final waypoint, it will hover without ending the

uint8 DJI_WAYPOINT_V2_MISSION_GOTO_FIRST_WAYPOINT_MODE_SAFELY = 0 # Go to the waypoint safely. The aircraft will rise to the same altitude of the
uint8 DJI_WAYPOINT_V2_MISSION_GOTO_FIRST_WAYPOINT_MODE_POINT_TO_POINT = 1 # Go to the waypoint from the current aircraft point to the waypoint directly.


uint32 mission_id # The Mission ID. Use to distinguish　different mission
uint16 miss_total_len # The Mission waypoint total length, could not exceed 65535
uint8 repeat_times # Mission execution can be repeated more than once. A value of 0 means the mission
uint8 finished_action # Action the aircraft will take when the waypoint mission is complete.

float32 max_flight_speed 

float32 auto_flight_speed 

uint8 exit_mission_on_signal_lost # Determines whether the mission should stop when connection between the aircraft

uint8 goto_first_waypoint_mode # Defines how the aircraft will go to the first waypoint from its current

WaypointV2[] mission 
