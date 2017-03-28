
local ros = require 'ros'
local tf = ros.tf
local moveit = require 'moveit'
local planning = require 'xamlamoveit.planning'


local errorCodes = {}
errorCodes.SUCCESSFUL = 1
errorCodes.INVALID_GOAL = -1
errorCodes.ABORT = -2
errorCodes.NO_IK_FOUND = -3
errorCodes.INVALID_LINK_NAME = -4

local Worker = torch.class('Worker')

function Worker:__init(nh)
    self.trajectoryQueue = {}      -- list of pending trajectories
    self.syncCallbacks = {}
    self.nodehandle = nh
    self.errorCodes = errorCodes
end


function Worker:doTrajectoryAsync(traj)
  table.insert(self.trajectoryQueue, traj)
end


function Worker:addSyncCallback(fn)
  table.insert(self.syncCallbacks, fn)
end


function Worker:removeSyncCallback(fn)
  for i,x in ipairs(self.syncCallbacks) do
    if x == fn then
      table.remove(self.syncCallbacks, i)
      return
    end
  end
end


function Worker:sync()
    for i,fn in ipairs(self.syncCallbacks) do
      fn(self)
    end
    return true
end


function Worker:cancelCurrentPlan(abortMsg)
  if self.currentPlan ~= nil then
    if callAbortCallback then
      local traj = self.currentPlan.traj
      if traj.abort ~= nil then
        if traj.manipulator ~= nil then
            traj.manipulator:stop()
        end
        traj:abort(abortMsg or 'Canceled')        -- abort callback
      end
    end
    self.currentPlan = nil
  end
end


local function computeIK(planner, group_name, robot_state_msg, ik_link_names, poses, avoid_collisions)
  local robot_state = planner.g:getCurrentState()
  if robot_state_msg then
    robot_state:fromRobotStateMsg(robot_state_msg)
  end
--setEndEffectorLink(name)
  local all_EE_names = planner.robot_model:getEndEffectorNames()
  local all_group_joint_names = planner.robot_model:getJointModelGroupNames()
  print("")
  print("all_EE_names")
  print("----------")
  print(all_EE_names)
  print("")
  print("all_group_joint_names")
  print("----------")
  print(all_group_joint_names)
  print("")
  local suc = robot_state:setFromIK(group_name, poses[1])
  return robot_state, suc
end

local function checkMoveGroupName(name)
  local robot_model_loader = moveit.RobotModelLoader("robot_description")
  local robot_model = robot_model_loader:getModel()
  local all_group_joint_names = robot_model:getJointModelGroupNames()

  for k,v in pairs(all_group_joint_names) do
    if name == v then
      return true
    end
  end
  return false
end

local function checkConvergence(cq,target,jointNames)
  local fullJointStateNames = cq:getVariableNames()
  local currrentPosition = cq:getVariablePositions()
  local sum = 0
  for i,v in ipairs(jointNames) do
    if v == fullJointStateNames[i] then
      sum = sum + math.abs(target[i] - currrentPosition[i])
    end
  end
  if (sum/#jointNames) < 1e-4 then
    return true
  else
    return false
  end
end


--TODO move this funtion to tf.StampedTransform
local function convertPoseMessage2Transform(pose_msg)
  transform_auxiliar = tf.Transform()
  transform_auxiliar:setOrigin(torch.Tensor({pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z}))
  transform_auxiliar:setRotation(tf.Quaternion( pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w))
  return transform_auxiliar
end


local function initializeMoveGroup(group_id, velocityScaling)
  local group_id = group_id or 'manipulator'
  if checkMoveGroupName(group_id) then
    local velocityScaling = velocityScaling or 0.5
    ros.INFO("connection with movegroup: " .. group_id)
    local manipulator = moveit.MoveGroupInterface(group_id)

    manipulator:setMaxVelocityScalingFactor(velocityScaling)
    manipulator:setGoalTolerance(1E-5)
    manipulator:setPlanningTime(2.0)

    -- ask move group for current state
    manipulator:startStateMonitor(0.008)
    local cs = manipulator:getCurrentState()
    manipulator:setStartStateToCurrentState()
    local currentPose = manipulator:getCurrentPose():toTensor()
    print('Current robot pose:')
    print(currentPose)

    printf('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink())
    return manipulator
  end
end


local function dispatchTrajectory(self)
  local status = 0
  if self.currentPlan == nil then
    if #self.trajectoryQueue > 0 then    -- check if new trajectory is available
      
      while #self.trajectoryQueue > 0 do
        print("#self.trajectoryQueue " .. #self.trajectoryQueue)
        local traj = table.remove(self.trajectoryQueue, 1)
        local suc, msg, plan
        suc = false
        if traj.accept == nil or traj:accept() then   -- call optional accept callback
          if traj.flush ~= nil then
            flush = traj.flush
          end
          if traj.waitCovergence ~= nil then
            waitCovergence = traj.waitCovergence
          end
          local group_name
          local manipulator
          local planner

          if traj.goal.spec.type == "xamlamoveit_msgs/moveJActionGoal" then
            print("xamlamoveit_msgs/movePActionGoal")
             group_name = traj.goal.goal.group_name.data
             manipulator = initializeMoveGroup(group_name)
             if not manipulator then
              status = self.errorCodes.INVALID_GOAL
             else
              planner = planning.MoveitPlanning.new(self.nodeHandle,manipulator)
             
              suc, msg, plan = planner:moveq(traj.goal.goal.goal.positions)
              if not plan then
                status = self.errorCodes.INVALID_GOAL
              end
            end
          elseif traj.goal.spec.type == "xamlamoveit_msgs/movePActionGoal" then
            print("xamlamoveit_msgs/movePActionGoal")
            group_name = traj.goal.goal.goal.group_name
            print("group_name:" .. group_name)
            manipulator = initializeMoveGroup(group_name)
            if manipulator then
              planner = planning.MoveitPlanning.new(self.nodeHandle,manipulator)
              local avoid_collisions = traj.goal.goal.goal.avoid_collisions
              local robot_state_msg = traj.goal.goal.goal.robot_state
              if #traj.goal.goal.goal.pose_stamped_vector ~= #traj.goal.goal.goal.ik_link_names then
                status = self.errorCodes.INVALID_GOAL
                print(r, "Number of pose and link names do not correspond")
                break
              end
            
              local poses = {}
              for i,k in ipairs(traj.goal.goal.goal.pose_stamped_vector) do
                  table.insert(poses, convertPoseMessage2Transform(k))
              end
            
              local ik_link_names = traj.goal.goal.goal.ik_link_names 
              local ik_link_name = traj.goal.goal.goal.ik_link_names
              local pose = traj.goal.goal.goal.pose_stamped
              local result, ik_suc
              if poses then
                print("multiple poses specified ")
                result, ik_suc = computeIK(planner, group_name, robot_state_msg, ik_link_names, poses, avoid_collisions)
              else
                print("only one pose specified ")
                result, ik_suc = computeIK(planner, group_name, robot_state_msg, {ik_link_name}, {pose}, avoid_collisions)
              end
              if ik_suc == true then
                assert(result)
                suc, msg, plan = planner:moveq(result)
                print(msg)
                if plan == nil then
                  print("INVALID_GOAL")
                  status = self.errorCodes.INVALID_GOAL
                end
              else
                print("NO_IK_FOUND")
                status = self.errorCodes.NO_IK_FOUND
              end
            else
              status = self.errorCodes.INVALID_GOAL
            end
          end

          if plan then
            local trajectory = plan:getTrajectoryMsg()
            local viaPoints = trajectory.joint_trajectory.points
            local jointNames = trajectory.joint_trajectory.joint_names
            traj.duration = viaPoints[#viaPoints].time_from_start
            traj.manipulator = manipulator
            traj.target = viaPoints[#viaPoints].positions:clone()
            traj.jointNames = jointNames
            manipulator:asyncExecute(plan)
          end

          self.currentPlan = {
            startTime = sys.clock(),     -- debug information
            traj = traj,
            status = status
          }
          break
        end
      end
    end
  end

  -- ensure first points are send to robot immediately after accepting trajectory execution
  if self.currentPlan ~= nil then     -- if we have an exsting trajectory

    local traj = self.currentPlan.traj
    local status = self.currentPlan.status
    local d = ros.Time.now() - traj.starttime
    if d > traj.duration then
      if checkConvergence(traj.manipulator:getCurrentState(),traj.target,traj.jointNames) then
        status =  self.errorCodes.SUCCESSFUL
      end
    end
    -- check if trajectory execution is still desired (e.g. not canceled)
    if (traj.proceed == nil or traj:proceed()) then
      -- execute main update call
      if status < 0 then    -- error
        if traj.abort ~= nil then
          if traj.manipulator ~= nil then
            traj.manipulator:stop()
          end
          traj:abort()        -- abort callback
        end
        self.currentPlan = nil
      elseif status == self.errorCodes.SUCCESSFUL then
        if traj.completed ~= nil then
          traj:completed()    -- completed callback
        end
        self.currentPlan = nil
      end
    else
      -- robot not ready or proceed callback returned false
      self:cancelCurrentPlan('Stop plan execution.')
    end
  end
end


local function workerCore(self)
  dispatchTrajectory(self)
end


function Worker:spin()
    --local ok, err = pcall(function() workerCore(self) end)
    local ok = true
    workerCore(self)
    -- abort current trajectory
    if (not ok) and self.currentPlan then
      local traj = self.currentPlan.traj
      if traj.abort ~= nil then
        if traj.manipulator ~= nil then
          traj.manipulator:stop()
        end
        traj:abort()
      end
      self.currentPlan = nil
    end
end


function Worker:shutdown()
end
