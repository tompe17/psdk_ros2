uint16 DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_TAKE_PHOTO = 1
uint16 DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_START_RECORD_VIDEO = 2
uint16 DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_STOP_RECORD_VIDEO = 3
uint16 DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_FOCUS = 4
uint16 DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_FOCAL_LENGTH = 5

uint16 actuator_index # The index of actuator. It is valid when the diagnostics is related

uint16 dji_waypoint_v2_action_actuator_camera_operation_type 
WaypointV2CameraActuatorFocusParam focus_param # The parameters for camera focus operation. It is valid only when
WaypointV2CameraActuatorFocalLengthParam zoom_param # The parameters for camera focus length operation. It is valid only when
