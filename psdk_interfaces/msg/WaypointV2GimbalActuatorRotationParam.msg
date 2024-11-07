int16 x # gimbal roll angle, unit: 0.1 deg,range:[-3600, 3600]*/
int16 y # gimbal pitch angle, unit: 0.1 deg,range:[-3600, 3600]*/
int16 z # gimbal yaw angle, unit: 0.1 deg,range:[-3600, 3600]*/
uint8 ctrl_mode # 0: absolute position control, 1:relative position control*/
uint8 roll_cmd_ignore # 0: roll command normal, 1: roll command ignore*/
uint8 pitch_cmd_ignore # 0: pitch command normal, 1: pitch command ignore*/
uint8 yaw_cmd_ignore # 0: yaw command normal, 1: yaw command ignore*/
uint8 abs_yaw_mode_ref # 0: absoluate rotate yaw relative to aircraft,
uint8 duration_time # 0: rotate time,unit:0.1s, range[1,255]*/
