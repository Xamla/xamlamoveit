# Distance to goal (joint space)
float64[] joint_distance
# Distance to goal (cartesian space)
float64[] cartesian_distance

#Information darüber das der Roboter nicht weiterfahren kann (z.B. weil die IK-Lösung nicht endlich ist, oder weil eine Selbstkollision erkannt wurde)
uint64 OK = 1
uint64 INVALID_IK = -1
uint64 SELFCOLLISION = -2
uint64 CLOSE2SINGULARITY = -3
uint64 error_code

#Information darüber ob das letzte Ziel erreicht wurde (converged)
bool converged