#int32 htc_time #hhmmss
string htc_time_hh
string htc_time_mm
string htc_time_ss

string team_id

int8 active_entrance_gate
int8 active_exit_gate

bool light_buoy_active
bool ACTIVE = True
bool NEGATIVE = False

#ex: RGG , light_pattern[0] = 0,light_pattern[1] = 2, light_pattern[2] = 2
int8[3] light_pattern
int8 RED = 0
int8 BLUE = 1
int8 GREEN = 2