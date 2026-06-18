# Unified chassis command used by the bridge and the temporary master package.

string command_type
string robot_id

string operation_mode

int32 distance_mm
int32 speed_mm_s
bool has_time_sec
int32 time_sec

int32 angle_deg
int32 speed_deg_s
bool has_angle_acc_deg_s2
int32 angle_acc_deg_s2

int32 x_mm
int32 y_mm
int32 heading_deg

string goal
string from_goal
string to_goal
bool has_wait_at_goal_sec
int32 wait_at_goal_sec
bool has_goto_time_sec
int32 goto_time_sec

string mission_name
string mission_id
string description

string report_type
bool has_report_duration_sec
int32 report_duration_sec

string mission_type
