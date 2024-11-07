uint8 DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_SIMULTANEOUSLY = 1  # The trigger starts simultaneously with the trigger that is associated.
uint8 DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_AFTER_FINISED = 2    # The trigger starts after the trigger associated has finished.
uint8 DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_UNKNOWN = 255       # Unkown timing type.

uint8 action_associated_type # The type of assciate trigger.
uint8 waiting_time # Waiting time in seconds after ActionTrigger starts.
uint16 action_id_associated # Associated action ID.
