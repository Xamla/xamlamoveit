# Distance to goal (joint space)
float64[] joint_distance
# Distance to goal (cartesian space)
float64[] cartesian_distance

#Information darüber das der Roboter nicht weiterfahren kann (z.B. weil die IK-Lösung nicht endlich ist, oder weil eine Selbstkollision erkannt wurde)
int64 OK = 1
int64 INVALID_IK = -1
int64 SELFCOLLISION = -2
int64 CLOSE2SINGULARITY = -3
int64 error_code

#Information darüber ob das letzte Ziel erreicht wurde (converged)
bool converged