#!/usr/bin/env th
local Controller = require "xamlamoveit.controller"
local ros = require "ros"
tf = ros.tf
local moveit = require "moveit"
local xutils = require "xamlamoveit.xutils"
local printf = xutils.printf

local function shutdownSetup()
  sp:stop()
  ros.shutdown()
end
local joyCtr, all_group_joint_names
local limit_vel, limit_acc
local run = true
local srv_spec = ros.SrvSpec("roscpp/GetLoggers")
local get_string_spec = ros.SrvSpec("xamlamoveit_msgs/GetSelected")
local set_string_spec = ros.SrvSpec("xamlamoveit_msgs/SetString")
local get_status_spec = ros.SrvSpec("xamlamoveit_msgs/StatusController")
local set_bool_spec = ros.SrvSpec("std_srvs/SetBool")

local function query_joint_limits(joint_names, node_handle)
  local max_vel = torch.zeros(#joint_names)
  local max_acc = torch.zeros(#joint_names)
  local nh = node_handle
  local root_path = 'robot_description_planning/joint_limits'
  for i, name in ipairs(joint_names) do
      local has_vel_param = string.format('/%s/%s/has_velocity_limits', root_path, name)
      local get_vel_param = string.format('/%s/%s/max_velocity', root_path, name)
      local has_acc_param = string.format('/%s/%s/has_acceleration_limits', root_path, name)
      local get_acc_param = string.format('/%s/%s/max_acceleration', root_path, name)
      if nh:getParamVariable(has_vel_param) then
          max_vel[i] = nh:getParamVariable(get_vel_param)
      else
          ros.WARN('Joint: %s has no velocity limit', name)
      end
      if nh:getParamVariable(has_acc_param) then
          max_acc[i] = nh:getParamVariable(get_acc_param)
      else
          max_acc[i] = max_vel[i] * 0.5
          ros.WARN('Joint: %s has no acceleration limit. Will be set to %f', name, max_acc[i])
      end
  end
  return max_vel, max_acc
end

local function findString(my_string, collection)
  local index = -1
  if torch.type(collection) == "std.StringVector" then
    collection = collection:totable()
  end
  if torch.type(collection) == "table" then
      index = table.indexof(collection, my_string)
  end
  if index > -1 then
      return true, index
  else
      return false, index
  end
end

function setMoveGroupHandler(request, response, header)
  local new_move_group_name = request.data
  run = false
  if joyCtr.move_group:getName() == new_move_group_name then
      response.success = true
      response.message = "Set move_group successfuly"
  else
      if findString(new_move_group_name, all_group_joint_names) then
          local succ, msg = joyCtr:setMoveGroupInterface(new_move_group_name)
          response.success = succ
          response.message = msg
      else
          local response_message = string.format("Unknown group name %s ! Choose from: ",new_move_group_name)
          for i, v in ipairs(all_group_joint_names) do
              response_message = string.format("%s %s;", response_message, v)
          end

          response.success = false
          response.message = response_message
      end
  end
  return true
end

function getMoveGroupHandler(request, response, header)
  response.success = true
  response.selected = joyCtr.move_group:getName()
  response.collection = all_group_joint_names
  return true
end

function setControllerNameHandler(request, response, header)
  local new_controller_name = request.data
  if new_controller_name == nil or new_controller_name == "" then
      response.success = false
      response.message = "string is empty"
  end
  run = false
  response.success = true
  response.message = "Success"
  return true
end

function getControllerNameHandler(request, response, header)
  local name = joyCtr.controller_name
  if name then
      response.selected = name
  else
      response.selected = ""
  end
  return true
end

function startStopHandler(request, response, header)
  local startStop = request.data
  if startStop == nil then
      response.success = false
      response.message = "bool is nil"
      return true
  end
  run = startStop
  response.success = true
  response.message = "Success"
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
  response.move_group_name = joyCtr.move_group:getName()
  response.joint_names = joyCtr.joint_monitor:getJointNames()
  response.out_topic = joyCtr:getOutTopic()
  response.in_topic = joyCtr:getInTopic()
  response.status_message_tracking = last_status_message_tracking or 'OK'
  return true
end

local function initializeMoveIt(groupName, velocityScaling)
    local velocityScaling = velocityScaling or 0.5

    local planningSceneInterface = moveit.PlanningSceneInterface()
    local manipulator = moveit.MoveGroupInterface(groupName or "manipulator")

    manipulator:setMaxVelocityScalingFactor(velocityScaling)
    manipulator:setGoalTolerance(1E-5)
    manipulator:setPlanningTime(10.0)

    -- ask move group for current state
    manipulator:getCurrentState()
    manipulator:setStartStateToCurrentState()
    local currentPose = manipulator:getCurrentPose():toTensor()
    ros.INFO("Current robot pose:")
    print(currentPose)

    ros.INFO(
        string.format('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink())
    )
    return manipulator, planningSceneInterface
end

function main(params)
    ros.init(params["__name"] or "xamlajoystickservice")
    local nh = ros.NodeHandle("~")
    local sp = ros.AsyncSpinner() -- background job
    sp:start()

    local controller_name, succ = nh:getParamString("controller_name")
    if not succ then
        controller_name = params.controller_name
        if controller_name == "" then
          controller_name = "sda10d"
        end
    end
    local robot_model_loader = moveit.RobotModelLoader("robot_description")
    local robot_model = robot_model_loader:getModel()

    all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
    all_group_joint_names = robot_model:getJointModelGroupNames()
    local planningGroup, succ = nh:getParamString("move_group")
    if not succ or (table.indexof(all_group_joint_names, planningGroup or "") < 0) then
        planningGroup = params.groupName or all_group_joint_names[1]
    end
    local state = moveit.RobotState.createFromModel(robot_model)
    local limit_vel, limit_acc = query_joint_limits(state:getVariableNames():totable(), nh)
    state = nil
    ---Services
    --set_limits
    set_limits_server = nh:advertiseService("set_velocity_limits", srv_spec, myServiceHandler)
    get_limits_server = nh:advertiseService("get_velocity_limits", srv_spec, myServiceHandler)
    --set_movegroup
    set_movegroup_server = nh:advertiseService("set_movegroup_name", set_string_spec, setMoveGroupHandler)
    get_movegroup_server = nh:advertiseService("get_movegroup_name", get_string_spec, getMoveGroupHandler)

    set_controller_name_server = nh:advertiseService("set_controller_name", set_string_spec, setControllerNameHandler)
    get_controller_name_server = nh:advertiseService("get_controller_name", get_string_spec, getControllerNameHandler)

    start_stop_server = nh:advertiseService("start_stop_tracking", set_bool_spec, startStopHandler)
    --status
    status_server = nh:advertiseService("status", get_status_spec, getStatusHandler)

    local moveGroup, psi = initializeMoveIt(planningGroup)
    joyCtr = Controller.JoystickControllerOpenLoop(nh, moveGroup, controller_name, params.frequency, false)

    local dt = ros.Rate(1/ params.frequency)
    if joyCtr:connect(params.topic) then
      dt:reset()
        while ros.ok() do
          if run then
            joyCtr:update()
          end
            ros.spinOnce()
            dt:sleep()
        end
    end

    sp:stop()
    sys.sleep(1.0)

    ros.shutdown()
end

local cmd = torch.CmdLine()
cmd:option("-topic", "/joy", "Topic to expect joystick messages.")
cmd:option("-groupName", "arm_left", "Move Group Id prepared by moveit")
cmd:option("-controller_name", "", "The namespace where we find the joint_command topic. (controller_list param)")
cmd:option("-frequency", 1 / 124, "Controller update frequence.")
local params = xutils.parseRosParametersFromCommandLine(arg, cmd)
main(params)
