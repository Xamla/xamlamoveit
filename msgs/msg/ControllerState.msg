# Distance to goal (joint space)
float64[] joint_distance
# Distance to goal (cartesian space)
float64[] cartesian_distance

# Status code containing information why robot cannot move further
# (e.g. because the IK-solution is not finite/valid or a self-collision has been detected)
int64 OK = 1
int64 INVALID_IK = -1
int64 SELFCOLLISION = -2
int64 CLOSE2SINGULARITY = -3
int64 error_code

# Flag indicating whether the last set-point was reached (controller converged)
bool converged
