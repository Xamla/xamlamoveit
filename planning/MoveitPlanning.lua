local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
require 'DMP'
local op = require 'optimplan'

local errorCodes = {
    SUCCESS = 1,
    FAILURE = 99999,
    PLANNING_FAILED = -1,
    INVALID_MOTION_PLAN = -2,
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3,
    CONTROL_FAILED = -4,
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5,
    TIMED_OUT = -6,
    PREEMPTED = -7,
    START_STATE_IN_COLLISION = -10,
    START_STATE_VIOLATES_PATH_CONSTRAINTS = -11,
    GOAL_IN_COLLISION = -12,
    GOAL_VIOLATES_PATH_CONSTRAINTS = -13,
    GOAL_CONSTRAINTS_VIOLATED = -14,
    INVALID_GROUP_NAME = -15,
    INVALID_GOAL_CONSTRAINTS = -16,
    INVALID_ROBOT_STATE = -17,
    INVALID_LINK_NAME = -18,
    INVALID_OBJECT_NAME = -19,
    FRAME_TRANSFORM_FAILURE = -21,
    COLLISION_CHECKING_UNAVAILABLE = -22,
    ROBOT_STATE_STALE = -23,
    SENSOR_INFO_STALE = -24,
    NO_IK_SOLUTION = -31
}

--@param desired joint angle position
local function isValid(q_des, q_curr) -- avoid large jumps in posture values and check if q_des tensor is valid
    local diff = 2
    if not q_curr then
        return true
    end
    if q_des:nDimension() > 0 then
        local curr_ = q_curr[{{1, q_des:size(1)}}]
        if curr_ then
            diff = torch.norm(curr_ - q_des)
        end
    end
    return diff < 1
end

local planning = require 'xamlamoveit.planning.env'
local MoveitPlanning = torch.class('xamlamoveit.planning.MoveitPlanning', planning)

function MoveitPlanning:__init(nh, move_group, dt)
    print('[robotControlAndSetup] init')
    self.g = move_group or moveit.MoveGroupInterface('manipulator')
    self.errorCodes = errorCodes
    local tmp = self.g:getJoints()
    local src_names = self.g:getCurrentState():getVariableNames()
    self.JOINT_NAMES = {}
    for i, dst_name in ipairs(tmp) do
        local found = false
        for j = 1, #src_names do
            if src_names[j] == dst_name then
                table.insert(self.JOINT_NAMES, dst_name)
                break
            end
        end
    end

    self.wait_duration = ros.Duration(dt or 0.008)
    self.counter = 0

    self.nodeHandle = nh or ros.NodeHandle()
    self.cartesianPath_spec = ros.SrvSpec('moveit_msgs/GetCartesianPath')
    self.commandClient = self.nodeHandle:serviceClient('/compute_cartesian_path', self.cartesianPath_spec)
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.plan_scene = moveit.PlanningScene(self.robot_model_loader:getModel())
    self.plan_scene:syncPlanningScene()

    self.robot_model = self.robot_model_loader:getModel()
    self.move_group_name = self.g:getName()
end

function MoveitPlanning:getRobotStart_stateMsg()
    local start_state = self.g:getCurrentState()
    return start_state:toRobotStateMsg()
end

function MoveitPlanning:getWayPointsMsg(points)
    local result = {}
    print(string.format('num Waypoints: %d', #points))
    for i = 1, #points do
        local m = ros.Message(ros.MsgSpec('geometry_msgs/Pose'))
        local pose = points[i]
        local position = pose:getOrigin()
        local orientation = pose:getRotation():toTensor()
        m.position.x = position[1]
        m.position.y = position[2]
        m.position.z = position[3]

        m.orientation.x = orientation[1]
        m.orientation.y = orientation[2]
        m.orientation.z = orientation[3]
        m.orientation.w = orientation[4]
        result[#result + 1] = m
    end
    return result
end

function MoveitPlanning:getCurrentPose()
    return self.g:getCurrentPose()
end

function MoveitPlanning:getJointPositionsFromRobotState(state)
    local p = state:getVariablePositions()
    local q = torch.zeros(#self.JOINT_NAMES)
    local src_names = state:getVariableNames()
    for i, dst_name in ipairs(self.JOINT_NAMES) do
        local found = false
        for j = 1, #src_names do
            if src_names[j] == dst_name then
                q[i] = p[j]
                found = true
                break
            end
        end
        assert(found, 'Robot state does not contain values of all robot joints.')
    end

    return q
end

function MoveitPlanning:getCurrentJointPositions()
    local s = self.g:getCurrentState()
    return self:getJointPositionsFromRobotState(s)
end

function MoveitPlanning:moveDMPPathPlanner(endPose, group_id)
    local group_id = group_id or self.g:getName()

    local currentStartPose = self.g:getCurrentPose()

    print('currentPose:')
    print(currentStartPose)

    print('endPose:')
    print(endPose)

    local succ = true
    local traj = moveit.RobotTrajectory(self.robot_model, self.g:getName())

    local dim = 3
    local dmp = DMP(dim)
    dmp.dt = 0.016
    dmp.goal:copy(endPose:getOrigin())
    dmp.start:copy(currentStartPose:getOrigin())
    dmp.currPos:copy(dmp.start)
    dmp:setStiffness(10)

    local trajectory = dmp:generateOfflineTrajectory(dmp.start, dmp.goal, 3)
    local posture = self.g:getCurrentState()
    local pose = self.g:getCurrentPose()
    local startOri = currentStartPose:getRotation()
    local endOri = endPose:getRotation()

    --[[
  print("startOri")
  print(startOri)
  print("endOri")
  print(endOri)
  ]]
    for i = 1, trajectory:size(1) do
        pose:setOrigin(trajectory[{i, {}}])
        pose:setRotation(startOri:slerp(endOri, (i - 1) / trajectory:size(1)))

        local ok = posture:setFromIK(group_id, pose, 10, 0.1)
        if not ok then
            error('NO IK solution found!')
            break
        end

        if not traj:empty() then
            if isValid(posture:getVariablePositions(), traj:getLastWayPoint():getVariablePositions()) then
                traj:addSuffixWayPoint(posture, dmp.dt)
            else
                succ = false
                print('big jump in IK solution!! at index i ' .. i)
            end
        else
            traj:addSuffixWayPoint(posture, dmp.dt)
        end
    end
    --self.g:setStartStateToCurrentState()
    --[[local start_state = self.g:getCurrentState()

  if not self.plan_scene:isPathValid(start_state, traj, self.g:getName(), false) then
    print("Path not valid")
    succ = false
  end
--]]
    return succ, traj
end

function MoveitPlanning:optimizeIPTP(path)
    local iptp = moveit.IterativeParabolicTimeParameterization()
    iptp:computeTimeStamps(path)
    path:unwind()
    return false, path
end

function MoveitPlanning:optimizePath(path, dt)
    local waypoints = path:toTensor()
    local maxDeviation = 0.1
    local maxVelocities = torch.Tensor({3.3, 3.3, 3.3, 3.3, 3.3, 3.3, 3.3, 3.3}) / 2
    local maxAccelerations = torch.Tensor({3.8, 3.8, 3.8, 3.0, 3.0, 3.0, 3.0, 3.0}) -- Wrist 3 is limited

    assert(waypoints:size(1) == maxVelocities:size(1))
    assert(waypoints:size(1) == maxAccelerations:size(1))
    local myP = op.Path(waypoints, maxDeviation)
    local trajectory_draft = op.Trajectory(myP, maxVelocities, maxAccelerations, dt)
    local succ = true
    local traj
    if not trajectory_draft:isValid() then
        traj = path
        succ = false
        print('Path optimisation failed')
    else
        traj = moveit.RobotTrajectory(self.robot_model, self.g:getName())
        print('SUCCESS')
        local duration = trajectory_draft:getDuration()
        printf('Duration: %f', duration)
        local robotState_tmp = path:getFirstWayPoint()

        for t = dt, duration, dt do
            local p = trajectory_draft:getPosition(t)
            robotState_tmp:setVariablePositions(p)
            local v = trajectory_draft:getVelocity(t)
            robotState_tmp:setVariableVelocities(v)

            printf('TimeStep: %f', t)
            print('Position:')
            print(p)
            print('Velocity:')
            print(v)

            traj:addSuffixWayPoint(robotState_tmp, dt)
        end
    end
    return succ, traj
end

function MoveitPlanning:moveRobotTo(pose)
    if torch.isTensor(pose) then
        pose = tf.Transform():fromTensor(pose)
    end

    self.g:setStartStateToCurrentState()
    local currentStart_state = self.g:getCurrentState()
    local dt = self.wait_duration:toSec()

    local succP, succT
    local path, traj

    succP, path = self:moveDMPPathPlanner(pose)
    if succP then
    --succT, traj = self:optimizePath(path, dt)
    end

    local myPlan = moveit.Plan()

    local msg = currentStart_state:toRobotStateMsg()
    myPlan:setStartStateMsg(msg)

    if succT then
        print('execute optimized plan')
        --traj:addPrefixWayPoint(currentStart_state, dt)
        myPlan:setTrajectoryMsg(traj:getRobotTrajectoryMsg())
    elseif succP then
        print('execute NOT optimized plan')
        myPlan:setTrajectoryMsg(path:getRobotTrajectoryMsg())
    else
        error('No path found')
    end

    --[[
  local traj = moveit.RobotTrajectory(self.robot_model, self.g:getName())
  local myPlan = moveit.Plan()
  s, myPlan = self:moveCartPathPlanner(pose,myPlan)
  traj:setRobotTrajectoryMsg(currentStart_state,myPlan:getTrajectoryMsg())
   local succT, traj = self:optimizePath(traj,dt)
   myPlan:setTrajectoryMsg(traj:getRobotTrajectoryMsg())
  --]]
    self.g:execute(myPlan)

    print('done.')
end

function MoveitPlanning:moveDMPPathPlanner(endPose, group_id, eeLinkName)
    local group_id = group_id or self.g:getName()
    local eeLinkName = eeLinkName or self.g:getEndEffectorLink()

    local currentStartPose = self.g:getCurrentPose(eeLinkName)

    print('currentPose:')
    print(currentStartPose)

    print('endPose:')
    print(endPose)

    local succ = true
    local traj = moveit.RobotTrajectory(self.robot_model, self.g:getName())

    local dim = 3
    local dmp = DMP(dim)
    dmp.dt = 0.016
    dmp.goal:copy(endPose:getOrigin())
    dmp.start:copy(currentStartPose:getOrigin())
    dmp.currPos:copy(dmp.start)
    dmp:setStiffness(10)

    local trajectory = dmp:generateOfflineTrajectory(dmp.start, dmp.goal, 3)
    local posture = self.g:getCurrentState()
    local pose = self.g:getCurrentPose()
    local startOri = currentStartPose:getRotation()
    local endOri = endPose:getRotation()
    self.plan_scene:syncPlanningScene()
    --[[
  print("startOri")
  print(startOri)
  print("endOri")
  print(endOri)
  ]]
    for i = 1, trajectory:size(1) do
        pose:setOrigin(trajectory[{i, {}}])
        pose:setRotation(startOri:slerp(endOri, (i - 1) / trajectory:size(1)))

        local ok = posture:setFromIK(group_id, pose, 10, 0.1)
        if not ok then
            error('NO IK solution found! Group ID: ' .. group_id)
            break
        end

        if not traj:empty() then
            if isValid(posture:getVariablePositions(), traj:getLastWayPoint():getVariablePositions()) then
                traj:addSuffixWayPoint(posture, dmp.dt)
            else
                succ = false
                print('big jump in IK solution!! at index i ' .. i)
            end
        else
            traj:addSuffixWayPoint(posture, dmp.dt)
        end
    end
    self.g:setStartStateToCurrentState()
    local start_state = self.g:getCurrentState()
    local group_name = self.g:getName()
    local verbose = true
    self.plan_scene:syncPlanningScene()
    if not self.plan_scene:isPathValid(start_state, traj, group_name, verbose) then
        print('Path not valid')
        succ = false
    end

    return succ, traj
end

local function generateRobotTrajectory(self, trajectory, dt)
    local traj = moveit.RobotTrajectory(self.robot_model, self.move_group_name)
    local time, pos, vel = trajectory:sample(dt)
    local start_state = self.g:getCurrentState()
    local dstNames = start_state:getVariableNames()

    --- Copy known values to target tensor.
    -- values are joint positions in ur5 order
    -- original is tensor with fallback values to use for non-ur5 joints.
    local function assignJoints(values, original) -- values are joint positions in ur5 order
        local r = original:clone()
        for i = 1, #dstNames do
            local dstName = dstNames[i]
            for j, srcName in ipairs(self.JOINT_NAMES) do
                if srcName == dstName then
                    r[i] = values[j]
                    break
                end
            end
        end
        return r
    end

    start_state:setVariablePositions(assignJoints(pos[1], start_state:getVariablePositions()))
    start_state:setVariableVelocities(assignJoints(vel[1], start_state:getVariableVelocities()))
    traj:addSuffixWayPoint(start_state, dt)
    local p = start_state:clone()
    for i = 2, pos:size(1) do
        p:setVariablePositions(assignJoints(pos[i], start_state:getVariablePositions()))
        p:setVariableVelocities(assignJoints(vel[i], start_state:getVariableVelocities()))
        traj:addSuffixWayPoint(p, dt)
    end

    return traj, start_state
end

-- Use optim path to generate direct trajectory from start joint positions to end joint positions
function MoveitPlanning:generateDirectPlan_qq(
    q_start,
    q_end,
    velocity_scaling,
    velocity_base,
    acceleration_base,
    check_path)
    velocity_scaling = velocity_scaling or 1.0
    velocity_base = velocity_base or torch.ones(#self.JOINT_NAMES) * math.pi
    if velocity_base:nDimension() < 1 then
        velocity_base = torch.ones(#self.JOINT_NAMES) * math.pi
    end
    acceleration_base = acceleration_base or torch.Ones(#self.JOINT_NAMES) * math.pi
    if acceleration_base:nDimension() < 1 then
        acceleration_base = torch.ones(#self.JOINT_NAMES) * math.pi
    end
    if check_path == nil then
        check_path = true
    end

    if torch.isTypeOf(q_start, moveit.RobotState) then
        q_start = self:getJointPositionsFromRobotState(q_start)
    end
    if torch.isTypeOf(q_end, moveit.RobotState) then
        q_end = self:getJointPositionsFromRobotState(q_end)
    end

    local dt = 0.008
    local waypoints = torch.Tensor(2, #self.JOINT_NAMES)
    waypoints[1] = q_start
    waypoints[2] = q_end

    if torch.norm(waypoints[2] - waypoints[1]) < 1e-4 then
        return nil, -1, 'Not moving: Already at goal.'
    end

    local maxVelocities, maxAccelerations
    local vel = velocity_base * velocity_scaling
    local acc = acceleration_base * velocity_scaling
    print(vel)
    if torch.isTypeOf(vel, torch.DoubleTensor) then
        maxVelocities = vel
    else
        maxVelocities = torch.Tensor(#self.JOINT_NAMES):fill(vel)
    end

    if torch.isTypeOf(acc, torch.DoubleTensor) then
        maxAccelerations = acc
    else
        maxAccelerations = torch.Tensor(#self.JOINT_NAMES):fill(acc)
    end

    local path = op.Path(waypoints, 1.0)
    local trajectory = op.Trajectory(path, maxVelocities, maxAccelerations, dt)

    if not trajectory:isValid() then
        return nil, -2, 'Path optimization failed.'
    end

    local traj, start_state = generateRobotTrajectory(self, trajectory, dt)

    -- check trajectory against planning scene
    if check_path and not self.plan_scene:isPathValid(start_state, traj, self.g:getName(), true) then
        return nil, -3, 'Path not valid.'
    end

    local plan = moveit.Plan()
    plan:setStartStateMsg(start_state:toRobotStateMsg())
    plan:setTrajectoryMsg(traj:getRobotTrajectoryMsg())
    return plan, 0, 'OK'
end

function MoveitPlanning:findGoodEndState(state, startPose, endPose)
    local stepSize = 0.001
    local groupName = self.g:getName()
    local startPos = endPose:getOrigin()
    local endPos = startPose:getOrigin()
    local off = endPos - startPos
    local dist = off:norm()
    local steps = dist / stepSize
    local startOri = startPose:getRotation()
    local endOri = endPose:getRotation()
    local pose = startPose:clone()
    for i = 1, steps do
        pose:setOrigin(torch.lerp(startPos, endPos, i / steps))
        pose:setRotation(startOri:slerp(endOri, i / steps))
        if not state:setFromIK(groupName, pose, 10, 0.1) then
            error('No IK solution found for goal pose.')
        end
    end
    return state
end

function MoveitPlanning:movep(pose, group_id, eeLinkName, velocity_scaling, velocity_base, acceleration_base, check_path)
    local groupName = group_id or self.g:getName()
    local eeLinkName = eeLinkName or self.g:getEndEffectorLink()
    if torch.isTensor(pose) then
        pose = tf.Transform():fromTensor(pose)
    end

    local start_state = self.g:getCurrentState()
    local endState = start_state:clone()

    --endState = self:findGoodEndState(endState,self.g:getCurrentPose(),pose)

    if not endState:setFromIK(groupName, pose, 10, 0.1) then
        error('No IK solution found for goal pose. Group_id: ' .. groupName)
    end
    local succ, traj = self:moveDMPPathPlanner(pose, groupName, eeLinkName)

    endState = traj:getLastWayPoint() --TODO this is not nice. Better --> get more access to the ikSolverSearch.

    local plan,
        status,
        msg =
        self:generateDirectPlan_qq(
        start_state,
        endState,
        velocity_scaling,
        velocity_base,
        acceleration_base,
        check_path
    )
    if plan then
        return true, plan, msg
    elseif status == -1 then
        ros.INFO('Already at goal.')
    else
        ros.WARN(msg)
    end

    return false, nil, msg
end

function MoveitPlanning:moveq(q_target, velocity_scaling, velocity_base, acceleration_base, check_path)
    local q_start = self:getCurrentJointPositions()
    local q_end = q_target or q_start
    if torch.isTypeOf(q_end, moveit.RobotState) then
        q_end = self:getJointPositionsFromRobotState(q_end)
    end

    local plan,
        status,
        msg = self:generateDirectPlan_qq(q_start, q_end, velocity_scaling, velocity_base, acceleration_base, check_path)
    if plan then
        --self.g:execute(plan)
        --print('done.')
        print('moving...')
    elseif status == -1 then
        print('Already at goal.')
    else
        ros.ERROR(msg)
        return false, msg, nil
    end

    return true, msg, plan
end

function MoveitPlanning:moveqtraj(q_waypoints, velocity_scaling, velocity_base, acceleration_base, check_path)
    velocity_scaling = velocity_scaling or 1.0
    velocity_base = velocity_base or math.pi
    acceleration_base = acceleration_base or math.pi
    if check_path == nil then
        check_path = false
    end

    local dt = 0.008
    local waypoints
    local q_start = self:getCurrentJointPositions()
    if torch.isTensor(q_waypoints) then
        assert(waypoints:nDimension() == 2 and waypoints:size(2) == #self.JOINT_NAMES)
        waypoints = torch.Tensor(waypoints:size(1) + 1, #self.JOINT_NAMES)
        waypoints[1] = q_start
        waypoints[{2, waypoints:size()}] = q_waypoints
    elseif type(q_waypoints) == 'table' then
        waypoints = torch.Tensor(#q_waypoints + 1, #self.JOINT_NAMES)
        waypoints[1] = q_start
        for i, q in ipairs(q_waypoints) do
            if torch.isTensor(q) then
                waypoints[i + 1] = q
            else
                waypoints[i + 1] = torch.Tensor(q) -- try to convert to tensor
            end
        end
    else
        error("Invalid argument 'q_waypoints': Tensor or table expected.")
    end

    local vel = velocity_base * velocity_scaling
    local acc = acceleration_base * velocity_scaling

    local maxVelocities
    if (torch.isTensor(vel)) then
        maxVelocities = vel
    else
        maxVelocities = torch.Tensor(#self.JOINT_NAMES):fill(vel)
    end

    local maxAccelerations
    if (torch.isTensor(acc)) then
        maxAccelerations = acc
    else
        maxAccelerations = torch.Tensor(#self.JOINT_NAMES):fill(acc)
    end

    print(waypoints)
    local path = op.Path(waypoints, 1.0)
    local trajectory = op.Trajectory(path, maxVelocities, maxAccelerations, dt)

    if not trajectory:isValid() then
        return nil, -2, 'Path optimization failed.'
    end

    local traj, start_state = generateRobotTrajectory(self, trajectory, dt)

    -- check trajectory against planning scene
    if check_path and not self.plan_scene:isPathValid(start_state, traj, self.g:getName(), false) then
        return nil, -3, 'Path not valid.'
    end

    local plan = moveit.Plan()
    plan:setStartStateMsg(start_state:toRobotStateMsg())
    plan:setTrajectoryMsg(traj:getRobotTrajectoryMsg())
    --self.g:execute(plan)
    return plan, 0, 'OK'
end

function MoveitPlanning:moveRobotToJointValues(joint_values)
    self.g:getCurrentState()
    self.g:setStartStateToCurrentState()
    if not self.g:setJointValueTarget(joint_values) then
        ros.ERROR('Setting joint value target failed.')
        return false, 'Setting joint value target failed.'
    end
    ros.INFO('planning...')
    local s, p = self.g:plan()
    if s == 0 then
        ros.ERROR('failed.')
        return false, 'FAILED'
    end
    return true, 'OK', p
end

return MoveitPlanning
