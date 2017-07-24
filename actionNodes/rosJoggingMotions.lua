#!/usr/bin/env th
local ros = require 'ros'
local tf = ros.tf
require 'ros.actionlib.SimpleActionServer'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local controller =require 'xamlamoveit.controller'
local actionlib = ros.actionlib

local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local planning = xamlamoveit.planning

local xutils = xamlamoveit.xutils
local printf = xutils.printf

local nodehandle, sp

local cntr
local run = false

all_EE_parent_group_names, all_EE_parent_link_names  = {},{}
all_group_joint_names = {}

local function initSetup()
  ros.init('MoveActions')
  nodehandle = ros.NodeHandle()
  service_queue = ros.CallbackQueue()

  sp = ros.AsyncSpinner()  -- background job
  sp:start()
end


local function shutdownSetup()
  sp:stop()
  ros.shutdown()
end

srv_spec = ros.SrvSpec('roscpp/GetLoggers')
get_string_spec = ros.SrvSpec('std_srvs/Trigger')
set_string_spec = ros.SrvSpec('xamlamoveit_msgs/SetString')
get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')
set_bool_spec = ros.SrvSpec('std_srvs/SetBool')

print(srv_spec)

function getMoveGroupHandler(request, response, header)

  return true
end

local function findString(my_string, collection)
  for i,v in ipairs(collection) do
    if v == my_string then
      return true, i
    end
  end
  return false
end

function setMoveGroupHandler(request, response, header)
  local new_move_group_name = request.data
  run = false
  if cntr.move_group:getName() == new_move_group_name then
    response.success = true
    response.message = "Set move_group successfuly"
  else
    if findString(new_move_group_name, all_group_joint_names) then
      local succ, msg = cntr:setMoveGroupInterface(new_move_group_name)
      response.success = succ
      response.message = msg
    else
      local response_message = "Unknown group name! Choose from: "
      for i,v in ipairs (all_group_joint_names) do
        response_message = string.format("%s %s;",response_message, v)
      end

      response.success = false
      response.message = response_message
    end
  end
  return true
end

function getMoveGroupHandler(request, response, header)
    response.success = true
    response.message = cntr.move_group:getName()
    return true
end

function setControllerNameHandler(request, response, header)
  local new_controller_name = request.data
  if new_controller_name == nil or  new_controller_name == "" then
      response.success = false
      response.message = "string is empty"
  end
  run = false
  response.success = true
  response.message = "Success"
  return true
end

function getControllerNameHandler(request, response, header)
  local name = cntr.controller_name
  if name then
    response.success = true
    response.message = name
  else
    response.success = false
    response.message = ""
  end
  return true
end


function startStopHandler(request, response, header)
  local startStop = request.data
  if startStop == nil  then
      response.success = false
      response.message = "bool is nil"
  end
  run = startStop
  response.success = true
  response.in_topic = "Success"
  return true
end


function getVelocityLimitsHandler(request, response, header)

  return true
end

function setVelocityLimitsHandler(request, response, header)

  return true
end

function getStatusHandler(request, response, header)
  response.is_running = run
  response.move_group_name = cntr.move_group:getName()
  response.joint_names = cntr.joint_monitor:getJointNames()
  response.out_topic = cntr:getOutTopic()
  response.in_topic = cntr:getInTopic()
  return true
end

local function joggingJoystickServer(namespace)
  local ns = namespace or "jogging_joystick_server"
  initSetup()
  local nh = ros.NodeHandle()
  local psi = moveit.PlanningSceneInterface()
  local dt = ros.Duration(1/125)

  local robot_model_loader = moveit.RobotModelLoader("robot_description")
  local robot_model = robot_model_loader:getModel()

  all_EE_parent_group_names, all_EE_parent_link_names  = robot_model:getEndEffectorParentGroups()
  all_group_joint_names = robot_model:getJointModelGroupNames()

  ---Services
  --set_limits
  set_limits_server = nh:advertiseService(string.format('/%s/set_velocity_limits',ns), srv_spec, myServiceHandler)
  get_limits_server = nh:advertiseService(string.format('/%s/get_velocity_limits',ns), srv_spec, myServiceHandler)
  --set_movegroup
  set_movegroup_server = nh:advertiseService(string.format('/%s/set_movegroup_name',ns), set_string_spec, setMoveGroupHandler)
  get_movegroup_server = nh:advertiseService(string.format('/%s/get_movegroup_name',ns), get_string_spec, getMoveGroupHandler)

  set_controller_name_server = nh:advertiseService(string.format('/%s/set_controller_name',ns), set_string_spec, setControllerNameHandler)
  get_controller_name_server = nh:advertiseService(string.format('/%s/get_controller_name',ns), get_string_spec, getControllerNameHandler)

  start_stop_server = nh:advertiseService(string.format('/%s/start_stop_tracking',ns), set_bool_spec, startStopHandler)
  --status
  status_server = nh:advertiseService(string.format('/%s/status',ns), get_status_spec, getStatusHandler)

  cntr = controller.JointJoggingController(nh, moveit.MoveGroupInterface(all_group_joint_names[1]), nil, dt)
  cntr:connect(string.format('/%s/jogging_command',ns))


  while ros.ok() do
    if run then
      cntr:update()
    end
    ros.spinOnce()
    cntr.dt:sleep()
    collectgarbage()
  end
  shutdownSetup()
end


joggingJoystickServer()
