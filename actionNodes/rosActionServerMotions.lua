local ros = require 'ros'
local tf = ros.tf
require 'ros.actionlib.SimpleActionServer'
local GoalStatus = require 'ros.actionlib.GoalStatus'
require 'Worker'
local actionlib = ros.actionlib

local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local environmentSetup = xamlamoveit.environmentsetup
local planning = xamlamoveit.planning

local xutils = xamlamoveit.xutils
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


local function movePActionServerGoal(goal_handle)
  ros.INFO("movePActionServerGoal")
  
  local g = goal_handle:acceptNewGoal()
  local suc = true
  local traj = {
      starttime = ros.Time.now(), duration = t1,
      goal_handle = goal_handle, goal = g,
      accept = function()  
        --goal_handle:setAccepted('Starting trajectory execution')
        return true
      end,
      proceed = function()
        --if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
          return true
        --else
        --  ros.WARN('Goal status of current trajectory no longer ACTIVE (actual: %d).', goal_handle:getGoalStatus().status)
        --  return false
        --end
      end,
      abort = function(self, msg)
        goal_handle:setAborted(nil, msg or 'Error')
      end,
      completed = function()
        local r = goal_handle:createResult()
        r.result = worker.errorCodes.SUCCESSFUL
        goal_handle:setSucceeded(r, 'Completed')
      end
    }

    if suc then
      worker:doTrajectoryAsync(traj)    -- queue for processing
    else
      -- trajectory is not valid, immediately abort it
      ros.WARN('Aborting trajectory processing: ' .. reason)
      local r = goal_handle:createResult()
      r.result = Worker.INVALID_GOAL
      goal_handle:setRejected(r, 'Validation of trajectory failed')
    end
end


local function moveJActionServerCancel(goal_handle)
  ros.INFO("moveJActionServerCancel")
  --goal_handle:setCanceled(nil, 'blub')
end


local function moveJActionServerGoal(goal_handle)
  ros.INFO("moveJActionServerGoal")
  local feedback = goal_handle:createFeeback()
  local g = goal_handle:acceptNewGoal()
  local traj = {
    starttime = ros.Time.now(), duration = t1,
    goal_handle = goal_handle, goal = g,
    accept = function()
      print("in accept")
      return true
      --[[
      if goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        goal_handle:setAccepted('Starting trajectory execution')
        return true
      else
        ros.WARN('Status of queued trajectory is not pending but %d.', goal_handle:getGoalStatus().status)
        return false
      end
      ]]
    end,
    proceed = function()
      --[[
        if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
        return true
      else
        ros.WARN('Goal status of current trajectory no longer ACTIVE (actual: %d).', goal_handle:getGoalStatus().status)
        return false
      end
      ]]
      return true
    end,
    abort = function(self, msg)
      goal_handle:setAborted(nil, msg or 'Error')
    end,
    completed = function()
      local r = goal_handle:createResult()
      r.result = worker.errorCodes.SUCCESSFUL
      goal_handle:setSucceeded(r, 'Completed')
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

  local mj = actionlib.SimpleActionServer(nodehandle, 'moveJ_action', 'xamlamoveit_msgs/moveJ')
  local mp = actionlib.SimpleActionServer(nodehandle, 'moveP_action', 'xamlamoveit_msgs/moveP')
  --local ml = actionlib.ActionServer(nodehandle, 'test_action', 'actionlib/Test')

  mj:registerGoalCallback(moveJActionServerGoal)
  mj:registerPreemptCallback(moveJActionServerCancel)
  mp:registerGoalCallback(movePActionServerGoal)
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
