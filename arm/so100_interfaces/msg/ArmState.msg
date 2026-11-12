# Published on /arm/state at 2 Hz.
std_msgs/Header header
string state            # "idle" | "resting" | "busy" | "holding" | "fault" | "torque_off" | "no_arm"
bool holding            # an object is in the jaws (after a successful Pick)
bool torque_on
string fault            # "" when fine, otherwise why the arm stopped (e.g. overheating)
float32 voltage_min     # V
float32 temp_max        # deg C
string task             # running task: "pick" "place" "pick_place" "home" "rest" or ""
string step             # current step of that task
string last_error_code  # result of the last task (SUCCESS, UNREACHABLE, ...)
string last_message
