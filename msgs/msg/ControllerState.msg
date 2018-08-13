# Distance to goal (joint space)
float64[] joint_distance
# Distance to goal (cartesian space)
float64[] cartesian_distance

# Status code containing information why robot cannot move further
# (e.g. because the IK-solution is not finite/valid or a self-collision has been detected)
int64 OK = 1
int64 INVALID_IK = -1
int64 SELF_COLLISION = -2
int64 SCENE_COLLISION = -3
int64 FRAME_TRANSFORM_FAILURE = -4
int64 IK_JUMP_DETECTED = -5
int64 CLOSE_TO_SINGULARITY = -6
int64 JOINT_LIMITS_VIOLATED = -7
int64 INVALID_LINK_NAME = -8
int64 TASK_SPACE_JUMP_DETECTED = -9
int64 error_code

# Flag indicating whether the last set-point was reached (controller converged)
bool converged

# Flag indicating whether model-based 'self collision' detection and prevention is enabled
bool self_collision_check_enabled

# Flag indicating whether moveit planning scene 'scene collision' detection and prevention is enabled
bool scene_collision_check_enabled

# Flag indicating whether joint-limits are enabled
# Use caes: The joint limit check has to be disabled when the robot-state
# is initially outside the limits to move it back into the limits.
bool joint_limits_check_enabled
