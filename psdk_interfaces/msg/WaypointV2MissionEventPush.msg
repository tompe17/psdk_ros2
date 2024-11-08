uint8 event 
uint32 f_c_timestamp 
uint8 interrupt_reason #0x00:rc triggered interrupt

uint8 recover_process #0x00:finished pause recover

uint8 finish_reason #0x00:finished normally; 0x10:External user trigger ended successfully

uint16 waypoint_index 

uint8 current_mission_exec_num 
uint8 finished_all_exec_num #0:not finished; 1:finished all exec num
