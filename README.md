# XamlaMoveit

This package containts extentions to the ros/moveit framework.


## start xamlamoveit

### prerequirements

Ros-Moveit setup needs to be started.

### Launch-File

start all xamla services/actions with:

```
<?xml version="1.0" encoding="utf-8" standalone="yes"?>
<launch>
  <group>
    <node name="xamlaMoveActions" pkg="xamlamoveit_server" type="moveJActionServer.lua" output="screen" />
    <node name="xamlaMoveGroupServices" pkg="xamlamoveit_server" type="moveGroupServices.lua" output="screen" />
    <node name="xamlaPlanningServices" pkg="xamlamoveit_server" type="planningServices.lua" output="screen" />
    <node name="xamlaResourceLockService" pkg="xamlamoveit_server" type="resourceLockService.lua" output="screen" />
  </group>
</launch>
```

## Heartbeats

If all components are healthy can be monitored by `xamla_sysmon`.
Configure the xamla_sysmon node to listen to these topics:

```
/xamlaJointMonitor/heartbeat
/xamlaMoveActions/heartbeat
/xamlaMoveGroupServices/heartbeat
/xamlaPlanningServices/heartbeat
/xamlaResourceLockService/heartbeat
```

## Services

- moveGroupServices
  - `/xamlaMoveGroupServices/query_fk`
  - `/xamlaMoveGroupServices/query_ik`
  -
  - `/xamlaMoveGroupServices/query_joint_position_collision_check`
  - `/xamlaMoveGroupServices/query_move_group_current_position`
  - `/xamlaMoveGroupServices/query_move_group_interface`

- xamlaPlanningServices
  - `/xamlaPlanningServices/query_joint_path`
  - `/xamlaPlanningServices/query_joint_trajectory`
- xamlaResourceLockServices
  -  `/xamlaResourceLockService/query_resource_lock`

## Actions

- `/moveJ_action`
- `/moveJ_step_action`

## Todo

### Execution Monitoring

- was wurde als command geschickt?
- was wurde gefahren
- detection von der initialen Roboterbewegung zur t0 Festlegung
- Dirigent sammelt die Aufmerksamkeit des Orchesters. Im Falle von Verzögerungen im Start prozess der einzelnen roboter (siehe start up time ur5), wird ein sog. Dirigent verwendet, der für die Koordinerung der generierten Pläne zuständig ist. Besonders wichtig wird diese Funktion bei sog. `hand over tasks`.

### NodeServer

Plänne und Jogging Kommandos sollen hier durchgehen:

- jogging wird nur activiert wenn alle actions aus sind
- moveJ nur mit start und goal 0 velocity am ende und start
- Collision Checks mit distanz informationen


### Robot emulation

Node: [tvpSimulationNode](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/tvpSimulationNode.lua)

Dieser Node ist aktuell nur auf den sda10d eingestellt. Dies kann aber leicht noch erweitert werden auf beliebige roboter typen.
Der Node startet für jede controller gruppe, die in config spezfiziert ist, einen tvp-controler.
Die config tabele muss mit der controller.yaml übereinstimmen damit moveit trajektorien abspielen kann.
Die Joint states aktuallisieren sich mit einem feedback delay (parameters können noch nicht von aussen gestzt werden):

- delay: 0.150 sec
- cycleTime: 0.008 sec

## Based on torch-moveit

This project uses fragments of [torch-moveit](https://github.com/xamla/torch-moveit).
