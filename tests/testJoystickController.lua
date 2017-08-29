local Controller = require 'xamlamoveit.controller'
local ros = require 'ros'
tf = ros.tf
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
local printf = xutils.printf

local function initializeMoveIt(groupName, velocityScaling)
  local velocityScaling = velocityScaling or 0.5

  local planningSceneInterface = moveit.PlanningSceneInterface()
  local manipulator = moveit.MoveGroupInterface(groupName or 'manipulator')

  manipulator:setMaxVelocityScalingFactor(velocityScaling)
  manipulator:setGoalTolerance(1E-5)
  manipulator:setPlanningTime(10.0)

  -- ask move group for current state
  manipulator:getCurrentState()
  manipulator:setStartStateToCurrentState()
  local currentPose = manipulator:getCurrentPose():toTensor()
  ros.INFO('Current robot pose:')
  print(currentPose)

  ros.INFO(string.format('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink()))
  return manipulator, planningSceneInterface
end

function main(params)
  ros.init(params["__name"] or 'JoystickTest')
  local nodehandle = ros.NodeHandle("~")
  local sp = ros.AsyncSpinner()  -- background job
  sp:start()

  local moveGroup, psi = initializeMoveIt(params.groupName)
  local joyCtr = Controller.JoystickController(nodehandle, moveGroup, params.controller_name, params.frequency, false)
  joyCtr:connect(params.topic)
  while ros.ok() do
    joyCtr:update()
    ros.spinOnce()
  end


  sp:stop()
  sys.sleep(1.0)

  ros.shutdown()
end

local cmd = torch.CmdLine()
cmd:option('-topic', "/joy", 'Topic to expect joystick messages.')
cmd:option('-groupName', "arm_left", "Move Group Id prepared by moveit")
cmd:option('-controller_name', "", "basicly the namespace where we find the joint_command topic")
cmd:option('-frequency',1/124 , "Controller update frequence.")
local params = xutils.parseRosParametersFromCommandLine(arg,cmd)
main(params)
