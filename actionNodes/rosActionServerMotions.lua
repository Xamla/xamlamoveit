local ros = require 'ros'
local tf = ros.tf
require 'ros.actionlib.SimpleActionServer'
local GoalStatus = require 'ros.actionlib.GoalStatus'
require 'Worker'
local actionlib = ros.actionlib

local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local environmentSetup = xamlamoveit.EnvironmentSetup
local planning = xamlamoveit.Planning
local plannter= {}

local xutils = xamlamoveit.Xutils
local printf = xutils.printf
local nodehandle, sp, worker

local function initSetup()
  ros.init('MoveActions')
  nodehandle = ros.NodeHandle()
  sp = ros.AsyncSpinner()  -- background job
  worker = Worker(nodehandle)
  sp:start()
end


local function shutdownSetup()
  sp:stop()
  ros.shutdown()
end


local function moveP_ActionServer_Goal(goalHandle)
  ros.INFO("moveP_ActionServer_Goal")
  
  local g = goalHandle:acceptNewGoal()
  local suc = true
  local traj = {
      starttime = ros.Time.now(), duration = t1,
      goalHandle = goalHandle, goal = g,
      accept = function()  
        --goalHandle:setAccepted('Starting trajectory execution')
        return true
      end,
      proceed = function()
        --if goalHandle:getGoalStatus().status == GoalStatus.ACTIVE then
          return true
        --else
        --  ros.WARN('Goal status of current trajectory no longer ACTIVE (actual: %d).', goalHandle:getGoalStatus().status)
        --  return false
        --end
      end,
      abort = function(self, msg)
        goalHandle:setAborted(nil, msg or 'Error')
      end,
      completed = function()
        local r = goalHandle:createResult()
        r.result = worker.errorCodes.SUCCESSFUL
        goalHandle:setSucceeded(r, 'Completed')
      end
    }

    if suc then
      worker:doTrajectoryAsync(traj)    -- queue for processing
    else
      -- trajectory is not valid, immediately abort it
      ros.WARN('Aborting trajectory processing: ' .. reason)
      local r = goalHandle:createResult()
      r.result = Worker.INVALID_GOAL
      goalHandle:setRejected(r, 'Validation of trajectory failed')
    end
end


local function moveJ_ActionServer_Cancel(goal_handle)
  ros.INFO("moveJ_ActionServer_Cancel")
  --goal_handle:setCanceled(nil, 'blub')
end


local function moveJ_ActionServer_Goal(goalHandle)
  ros.INFO("moveJ_ActionServer_Goal")
  local feedback = goalHandle:createFeeback()
  local g = goalHandle:acceptNewGoal()
  local traj = {
    starttime = ros.Time.now(), duration = t1,
    goalHandle = goalHandle, goal = g,
    accept = function()
      print("in accept")
      return true
      --[[
      if goalHandle:getGoalStatus().status == GoalStatus.PENDING then
        goalHandle:setAccepted('Starting trajectory execution')
        return true
      else
        ros.WARN('Status of queued trajectory is not pending but %d.', goalHandle:getGoalStatus().status)
        return false
      end
      ]]
    end,
    proceed = function()
      --[[
        if goalHandle:getGoalStatus().status == GoalStatus.ACTIVE then
        return true
      else
        ros.WARN('Goal status of current trajectory no longer ACTIVE (actual: %d).', goalHandle:getGoalStatus().status)
        return false
      end
      ]]
      return true
    end,
    abort = function(self, msg)
      goalHandle:setAborted(nil, msg or 'Error')
    end,
    completed = function()
      local r = goalHandle:createResult()
      r.result = worker.errorCodes.SUCCESSFUL
      goalHandle:setSucceeded(r, 'Completed')
    end
  }
   worker:doTrajectoryAsync(traj)    -- queue for processing
end


local function moveP_ActionServer_Cancel(goal_handle)
  ros.INFO("moveP_ActionServer_Cancel")
  --goal_handle:setCanceled(nil, 'NOT IMPLEMENTED YET')
end


local function moveActionServer()
  initSetup()
  
  --moveGroup = initializeMoveGroup()
  local psi = moveit.PlanningSceneInterface()
  environmentSetup.labRoboteur(psi)
  --local dp = moveGroup:getCurrentPose()
  local dt = 1/125
  --planner = planning.moveitPlanning.new(nodeHandle,MoveGroup,dt)
  ros.console.setLoggerLevel('actionlib', ros.console.Level.Warn)

  local mj = actionlib.SimpleActionServer(nodehandle, 'moveJ_action', 'roboteur_msgs/moveJ')
  local mp = actionlib.SimpleActionServer(nodehandle, 'moveP_action', 'roboteur_msgs/moveP')
  --local ml = actionlib.ActionServer(nodehandle, 'test_action', 'actionlib/Test')

  mj:registerGoalCallback(moveJ_ActionServer_Goal)
  mj:registerPreemptCallback(moveJ_ActionServer_Cancel)
  mp:registerGoalCallback(moveP_ActionServer_Goal)
  mp:registerPreemptCallback(moveP_ActionServer_Cancel)

  print('Starting action server...')
  mj:start()
  mp:start()
  --ml:start()

  while ros.ok() do
    worker:spin()
    ros.spinOnce()
    collectgarbage()
  end
  worker:shutdown()
  mj:shutdown()
  mp:shutdown()
  --ml:shutdown()
  shutdownSetup()
end


moveActionServer()
