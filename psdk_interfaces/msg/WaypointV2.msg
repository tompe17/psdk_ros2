
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE = 0  #In the mission, the aircraft will go to the waypoint along a curve and fly past the waypoint.
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_A_CURVE_AND_STOP = 1 #In the mission, the aircraft will go to the waypoint along a curve and stop at the waypoint.
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_A_STRAIGHT_LINE_AND_STOP = 2  #In the mission, the aircraft will go to the waypoint along a straight line and stop at the waypoint.
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_COORDINATE_TURN = 3   #In the mission, the aircraft will fly from the previous waypoint to the next waypoint along a smooth curve without stopping at this waypoint.
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_FIRST_POINT_ALONG_A_STRAIGHT_LINE = 4  # In the mission, the aircraft will go to the first waypoint along a straight line.
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_STRAIGHT_OUT = 5   # Straight exit the Last waypoint, Only valid for last waypoint.
uint8 DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_UNKNOWN = 255    # Unknown

uint8 DJI_WAYPOINT_V2_HEADING_MODE_AUTO = 0    # Aircraft's heading will always be in the direction of flight.
uint8 DJI_WAYPOINT_V2_HEADING_FIXED    = 1    # Aircraft's heading will be set to the heading when reaching the first waypoint.
uint8 DJI_WAYPOINT_V2_HEADING_MANUAL   = 2    # The aircraft's heading in the mission can be controlled by the remote controller.
uint8 DJI_WAYPOINT_V2_HEADING_WAYPOINT_CUSTOM = 3  # In the mission, the aircraft's heading will change dynamically and adapt to the heading set at the next waypoint.
uint8 DJI_WAYPOINT_V2_HEADING_TOWARD_POINT_OF_INTEREST = 4 # Aircraft's heading will always toward point of interest.
uint8 DJI_WAYPOINT_V2_HEADING_GIMBAL_YAW_FOLLOW  = 5   # The aircraft's heading rotate simultaneously with its gimbal's yaw.
uint8 DJI_WAYPOINT_V2_HEADING_UNKNOWN = 255         # Unknown.

uint8 DJI_WAYPOINT_V2_TURN_MODE_CLOCKWISE = 0  # The aircraft's heading rotates clockwise.
uint8 DJI_WAYPOINT_V2_TURN_MODE_COUNTER_CLOCKWISE = 1   # The aircraft's heading rotates counterclockwise.
uint8 DJI_WAYPOINT_V2_TURN_MODE_UNKNOWN = 255    # Changes the heading of the aircraft by rotating the aircraft anti-clockwise.



float64 longitude # waypoint position relative to WayPointV2InitSettings's reference point.unit: m
float64 latitude 
float32 relative_height # relative to takeoff height
uint8 waypoint_type # Waypoint flight path mode
uint8 heading_mode # Represents current aircraft's heading mode on current waypoint.
WaypointV2Config config # Represents current waypoint's speed config.
uint16 damping_distance 
float32 heading # The heading to which the aircraft will rotate by the time it reaches the
uint8 turn_mode # Determines whether the aircraft will turn clockwise or anticlockwise when
float32 position_x # X distance to reference point, North is positive
float32 position_y # Y distance to reference point, East is positive
float32 position_z # Z distance to reference point, UP is positive

float32 max_flight_speed 

float32 auto_flight_speed 
