Provides specialized planning functions and sensore interfaces.

## CreateJointSpacePlan

- planner (e.g. optim path)
- plannerParameters
  - max Acceleration, Velocity, Jerk
  - maxDeviation
  - dt (samplingResolution)
- waypoints

## MoveJ commands

MoveJ Kommandos sollen immer von einem definierten Startpunkt zu einem definierten Endpunkt führen.
Hierbei sollen bestimmte Garantien erfüllt werden. Genauigkeit bei der Trajectory-Verfolgung, sowohl in Räumlichen als auch Zeitlichen Rahmen, müssen in gewissen Tolleranzen bleiben.

### Definition Universal Robots

`moveJ` will make movements that are calculated in the joint space of the robot arm. Each joint is controlled to reach the desired end location at the same time. This movement type results in a curved path for the tool. The shared parameters that apply to this movement type are the maximum joint speed and joint acceleration to use for the movement calculations, specified in `deg/s` and `deg/s^2` , respectively. If it is desired to have the robot arm move fast between waypoints, disregarding the path of the tool between those waypoints, this movement type is the favorable choice.

### global parameters:

- `path_execution_tolerance` and `max_start_point_distance_tolerance` (gegebenenfals gleich?): Dies ist die Fehlertoleranz wärend der Ausführung der Trajectorie, in radians.
- `duration_tolerance`: Diese Property soll die Tolereanz in sek. angeben, die der Roboter hat, um den letzten Punkt der Trajektorie zu erreichen der Kommandiert wurde.
- `goal_tolerance`: Toleranz für jedes Joint um den Zielzustand zu erreichen. Wenn die Zielposition mit +/- `goal_tolerance` erreicht wird, war die Ausführung erfolgreich.

### meta parameter für jeden individuellen movej call

- `double[] max Velocity,Acceleration` <- pro joint
- `bool collisionCheck`

### optim path planner

- `double[] Velocity_constraint,Acceleration_constraint` <- pro joint
- `double maxDeviation`: Gibt an wie genau/smooth die Wegpunkte angefahren werden sollen, unter Berücksichtigung von max Vel,Acc.

### moveit ompl path planner

### moveit chomp path planner