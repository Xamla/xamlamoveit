local ros = require 'ros'
local moveit = require 'moveit'
local core = require 'xamlamoveit.core'
local xutils = require 'xamlamoveit.xutils'

local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local errorCodes = {}
errorCodes.SUCCESSFUL = 1
errorCodes.INVALID_GOAL = -1
errorCodes.ABORT = -2
errorCodes.NO_IK_FOUND = -3
errorCodes.INVALID_LINK_NAME = -4
errorCodes.SIGNAL_LOST = -9999

local MoveJWorker = torch.class('MoveJWorker')

local function checkMoveGroupName(self, name)
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.plan_scene = moveit.PlanningScene(self.robot_model_loader:getModel())
    self.plan_scene:syncPlanningScene()
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    ros.INFO('available move_groups:\n%s', tostring(all_group_joint_names))
    for k, v in pairs(all_group_joint_names) do
        if name == v then
            return true
        end
    end
    ros.ERROR('could not find move_group: ' .. name)
    return false
end

local function checkConvergence(cq, target, jointNames)
    local fullJointStateNames = cq:getVariableNames():totable()
    local currentPosition = cq:getVariablePositions()
    local sum = 0
    for i, v in ipairs(jointNames) do
        if v == fullJointStateNames[i] then
            ros.DEBUG('jointName: %s, target = %f, current %f', v, target[i], currentPosition[i])
            sum = sum + math.abs(target[i] - currentPosition[i])
        end
    end
    if sum / #jointNames < 1e-3 then
        ros.INFO('[checkConvergence] Converged')
        return true
    else
        return false
    end
end

local function checkParameterForAvailability(self, topic, wait_duration)
    wait_duration = wait_duration or ros.Duration(1.0)
    local counter = 0
    local value
    while value == nil and ros.ok() do
        if counter % 10 == 1 then
            ros.WARN('%s not available trying again in 1 sec', topic)
        end
        wait_duration:sleep()
        value = self.nodehandle:getParamVariable(topic)
        counter = counter + 1
        ros.spinOnce()
    end
    return value
end

local function initializeMoveGroup(self, group_id, velocity_scaling)
    local group_id = group_id or 'manipulator'
    if checkMoveGroupName(self, group_id) then
        local velocity_scaling = velocity_scaling or 0.5
        ros.INFO('connection with movegroup: ' .. group_id)
        local manipulator = moveit.MoveGroupInterface(group_id)
        ros.INFO('set parameters for movegroup: ' .. group_id)
        manipulator:setMaxVelocityScalingFactor(velocity_scaling)
        manipulator:setPlanningTime(2.0)

        ros.INFO('start state monitor for movegroup: ' .. group_id)
        -- ask move group for current state
        manipulator:startStateMonitor(0.008)
        local cs = manipulator:getCurrentState()
        manipulator:setStartStateToCurrentState()
        local currentPose = manipulator:getCurrentPose():toTensor()
        ros.INFO('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink())
        return manipulator
    end
end

function MoveJWorker:__init(nh)
    self.trajectoryQueue = {} -- list of pending trajectories
    self.syncCallbacks = {}
    self.nodehandle = nh
    self.errorCodes = errorCodes
    self.allowed_execution_duration_scaling =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_execution_duration_scaling')
    self.allowed_goal_duration_margin =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_goal_duration_margin')
    self.allowed_start_tolerance =
        math.max(
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_start_tolerance') or 0,
        0.01
    )
    self.execution_duration_monitoring =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/execution_duration_monitoring')
    self.execution_velocity_scaling =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/execution_velocity_scaling')

    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.plan_scene = moveit.PlanningScene(self.robot_model_loader:getModel())
    self.plan_scene:syncPlanningScene()
    self.joint_monitor = core.JointMonitor(self.robot_model:getVariableNames():totable())
    local once = true
    local ready = false
    while not ready and ros.ok() do
        if once then
            ros.ERROR('joint states not ready')
            once = false
        end
        ready = self.joint_monitor:waitReady(20.0)
    end
    self.query_resource_lock_service =
        self.nodehandle:serviceClient('xamlaResourceLockService/query_resource_lock', 'xamlamoveit_msgs/QueryLock')
    self.action_client =
        actionlib.SimpleActionClient('moveit_msgs/ExecuteTrajectory', 'execute_trajectory', self.node_handle)
    self.manipulators = {}
    local move_group_names = self.robot_model:getJointModelGroupNames()
    for i, v in ipairs(move_group_names) do
        self.manipulators[v] = initializeMoveGroup(self, v)
    end
end

function MoveJWorker:doTrajectoryAsync(traj)
    table.insert(self.trajectoryQueue, traj)
end

function MoveJWorker:addSyncCallback(fn)
    table.insert(self.syncCallbacks, fn)
end

function MoveJWorker:removeSyncCallback(fn)
    for i, x in ipairs(self.syncCallbacks) do
        if x == fn then
            table.remove(self.syncCallbacks, i)
            return
        end
    end
end

function MoveJWorker:sync()
    for i, fn in ipairs(self.syncCallbacks) do
        fn(self)
    end
    return true
end

function MoveJWorker:cancelCurrentPlan(abortMsg)
    ros.INFO('MoveJWorker:cancelCurrentPlan %s', abortMsg)
    if self.currentPlan ~= nil then
        if callAbortCallback then
            local traj = self.currentPlan.traj
            if traj.abort ~= nil then
                if traj.manipulator ~= nil then
                --traj.manipulator:stop()
                end
                traj:abort(abortMsg or 'Canceled') -- abort callback
            end
        end
        self.currentPlan = nil
    end
end

local function queryLock(self, id_resources, id_lock, release_flag)
    ros.DEBUG('queryLock')
    local request = self.query_resource_lock_service:createRequest()
    request.release = release_flag or false
    request.id_resources = id_resources
    request.id_lock = id_lock or ''
    local responds = self.query_resource_lock_service:call(request)
    if responds.success then
        return responds.success, responds.id_lock, responds.creation_date, responds.expiration_date
    else
        return responds.success
    end
end

local function lockResource(self, id_resources, id_lock)
    return queryLock(self, id_resources, id_lock, false)
end

local function releaseResource(self, id_resources, id_lock)
    return queryLock(self, id_resources, id_lock, true)
end

local function executeAsync(self, traj, plan)
    if not self.action_client:isServerConnected() then
        return false
    end
    local g = self.action_client:createGoal()
    g.trajectory = plan:getTrajectoryMsg()
    traj.status = 0

    local function action_done(state, result)
        ros.INFO('actionDone')
        ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
        ros.INFO('Result:\n%s', result)
        if result then
            traj.status = result.error_code.val
        else
            traj.status = errorCodes.SIGNAL_LOST -- SIGNAL LOST
        end
        print("the trajecory status: ", traj.status)
    end

    local function action_active()
        ros.INFO('executeAsync Action_active')
    end

    local function action_feedback(feedback)
        ros.INFO('Action_feedback \n\t%s ', tostring(feedback))
    end
    self.action_client:sendGoal(g, action_done, action_active, action_feedback)
    return true
end

local function executePlan(self, plan, manipulator, traj)
    if plan then
        local trajectory = plan:getTrajectoryMsg()
        local jointNames = trajectory.joint_trajectory.joint_names
        ros.WARN('executePlan')
        local suc, id_lock, creation, expiration = lockResource(self, jointNames, nil)
        if suc then
            ros.WARN('executeAsync')
            if executeAsync(self, traj, plan) then
                local viaPoints = trajectory.joint_trajectory.points
                traj.duration = viaPoints[#viaPoints].time_from_start
                traj.manipulator = manipulator
                traj.target = viaPoints[#viaPoints].positions:clone()
                traj.jointNames = jointNames
                traj.id_lock = id_lock
                traj.expiration_date = expiration
                traj.creation_date = creation
            end
        else
            ros.ERROR('could not aquire Lock!')
        end
    end
    return traj
end

local function generateRobotTrajectory(self, manipulator, trajectory, check_collision)
    local suc = true
    local move_group_name = manipulator:getName()
    local traj = moveit.RobotTrajectory(self.robot_model, move_group_name)
    tic('getCurrentState')
    local start_state = manipulator:getCurrentState()
    toc('getCurrentState')
    local ori_start_state = start_state:clone()
    local dt = trajectory.points[1].time_from_start:toSec()
    start_state:setVariablePositions(trajectory.points[1].positions, trajectory.joint_names)
    start_state:setVariableVelocities(trajectory.points[1].velocities, trajectory.joint_names)
    start_state:setVariableAccelerations(trajectory.points[1].accelerations, trajectory.joint_names)
    start_state:update()

    local distance = ori_start_state:distance(start_state)
    ros.INFO('start state distance to current state: %f', distance)
    if distance > self.allowed_start_tolerance then
        ros.ERROR('start state is to far away from current state. tolerance: %f', self.allowed_start_tolerance)
        suc = false
        return traj, start_state, suc
    end

    traj:addSuffixWayPoint(start_state, dt)
    local p = start_state:clone()
    for i = 1, #trajectory.points do
        local k = math.min(i + 1, #trajectory.points)
        dt = trajectory.points[k].time_from_start:toSec() - trajectory.points[i].time_from_start:toSec()
        p:setVariablePositions(trajectory.points[i].positions, trajectory.joint_names)
        p:setVariableVelocities(trajectory.points[i].velocities, trajectory.joint_names)
        p:setVariableAccelerations(trajectory.points[i].accelerations, trajectory.joint_names)
        p:update()
        traj:addSuffixWayPoint(p, dt)
    end

    if check_collision == true then
        tic('checkCollision')
        if self.plan_scene:syncPlanningScene() then --TODO parameter einstellen. performance check
            if not self.plan_scene:isPathValid(start_state, traj, move_group_name, true) then
                ros.ERROR('[generateRobotTrajectory] Path not valid')
                suc = false
            end
        else
            ros.ERROR('[generateRobotTrajectory] Planning Scene could not be updated')
            suc = false
        end
        toc('checkCollision')
    end
    return traj, start_state, suc
end

local function checkJointNames(self, move_group_name, joint_names)
    local ori_joint_names = self.robot_model:getGroupJointNames(move_group_name)
    return table.isSimilar(ori_joint_names, joint_names)
end

local function findGroupNameFromJointNames(self, joint_names)
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    for i, v in pairs(all_group_joint_names) do
        if checkJointNames(self, v, joint_names) then
            return v
        end
    end
    return ''
end

local function handleMoveJTrajectory(self, traj)
    local group_name
    local manipulator
    local suc, msg, plan, status, rest, start_state
    status = 0
    ros.INFO('xamlamoveit_msgs/moveJActionGoal')
    group_name = findGroupNameFromJointNames(self, traj.goal.goal.trajectory.joint_names)
    ros.INFO('Specified groupName: ' .. group_name)
    manipulator = self.manipulators[group_name]
    if not manipulator then
        status = self.errorCodes.INVALID_GOAL
    else
        traj.manipulator = manipulator
        traj.joint_monitor = self.joint_monitor
        tic('generateRobotTrajectory')
        rest,
            start_state,
            suc = generateRobotTrajectory(self, manipulator, traj.goal.goal.trajectory, traj.check_collision)
        toc('generateRobotTrajectory')

        if suc == false then
            status = self.errorCodes.INVALID_GOAL
            suc = false
            msg = 'Could create valid Trajectory.'
        else
            plan = moveit.Plan()
            plan:setStartStateMsg(start_state:toRobotStateMsg())
            plan:setTrajectoryMsg(rest:getRobotTrajectoryMsg())
        end
    end
    if suc == true then
        traj.starttime = ros.Time.now()
        traj = executePlan(self, plan, manipulator, traj)
    end
    traj.status = status
    return suc, msg, traj, status
end

local function dispatchTrajectory(self)
    local status = 0
    if self.currentPlan == nil then
        if #self.trajectoryQueue > 0 then -- check if new trajectory is available
            while #self.trajectoryQueue > 0 do
                local traj = table.remove(self.trajectoryQueue, 1)
                if traj.accept == nil or traj:accept() then -- call optional accept callback
                    suc, msg, traj, status = handleMoveJTrajectory(self, traj)

                    self.currentPlan = {
                        startTime = sys.clock(), -- debug information
                        traj = traj,
                        status = status
                    }
                    break
                end
            end
        end
    end

    -- ensure first points are send to robot immediately after accepting trajectory execution
    if self.currentPlan ~= nil then -- if we have an exsting trajectory
        local suc, id_lock, creation, expiration
        local traj = self.currentPlan.traj
        local d = ros.Time.now() - traj.starttime
        status = traj.status
        if status == 0 then
            if traj.expiration_date then
                ros.DEBUG('Expiration data: ' .. tostring(traj.expiration_date))
                local dur = ros.Duration((traj.expiration_date:toSec() - traj.creation_date:toSec()) / 2)
                if dur > (traj.expiration_date - ros.Time.now()) then
                    suc, id_lock, creation, expiration = lockResource(self, traj.jointNames, traj.id_lock)
                    traj.creation_date = creation
                    traj.expiration_date = expiration
                    traj.id_lock = id_lock
                end
            end

            if traj.duration == nil then
                error('Trajectory duration field must not be nil.')
            end

            -- check if trajectory execution is still desired (e.g. not canceled)
            if traj:proceed() == false then
                -- robot not ready or proceed callback returned false
                status = self.errorCodes.ABORT
                ros.ERROR('Stop plan execution. proceed method returned false')
                self:cancelCurrentPlan('Stop plan execution.')
            else
                status = traj.status
                if status < 1 then
                    if self.execution_duration_monitoring == true then
                        if
                            d >
                                (traj.duration * self.allowed_execution_duration_scaling +
                                    self.allowed_goal_duration_margin)
                         then
                            ros.ERROR('trajecotry duration timeout reached')
                            status = self.errorCodes.ABORT
                        end
                    else
                        if d > traj.duration + self.allowed_goal_duration_margin + 1 then
                            ros.ERROR('trajecotry duration timeout reached')
                            print((d - traj.duration + self.allowed_goal_duration_margin), status)
                            status = self.errorCodes.ABORT
                        end
                    end
                end
            end

            if suc == false then
                status = self.errorCodes.ABORT
                ros.ERROR('Stop plan execution. Lock failed.')
                self:cancelCurrentPlan('Stop plan execution. Lock failed.')
            end
        end
        -- execute main update call
        if status < 0 then -- error
            if traj.abort ~= nil then
                ros.ERROR('status: ' .. status)
                traj:abort('status: ' .. status, status) -- abort callback
            end
            if traj.id_lock and traj.jointNames then
                suc, id_lock, creation, expiration = releaseResource(self, traj.jointNames, traj.id_lock)
            end
            self.currentPlan = nil
        elseif status == self.errorCodes.SUCCESSFUL then
            if traj.completed ~= nil then
                traj:completed() -- completed callback
            end
            suc, id_lock, creation, expiration = releaseResource(self, traj.jointNames, traj.id_lock)
            self.currentPlan = nil
        end
    end
end

function MoveJWorker:reset()
    if self.currentPlan then
        local traj = self.currentPlan.traj
        if traj.abort ~= nil then
            ros.ERROR('currentPlan failed (reset was triggered)')
            traj:abort()
        end
        self.currentPlan = nil
    end
    if #self.trajectoryQueue > 0 then -- check if new trajectory is available
        while #self.trajectoryQueue > 0 do
            local traj = table.remove(self.trajectoryQueue, 1)
            if traj.abort ~= nil then
                traj:abort()
            end
        end
    end
    self.allowed_execution_duration_scaling =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_execution_duration_scaling')
    self.allowed_goal_duration_margin =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_goal_duration_margin')
    self.allowed_start_tolerance =
        math.max(
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_start_tolerance') or 0,
        0.01
    )
    self.execution_duration_monitoring =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/execution_duration_monitoring')
    self.execution_velocity_scaling =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/execution_velocity_scaling')
    --self.query_resource_lock_service =
    --    self.nodehandle:serviceClient('xamlaResourceLockService/query_resource_lock', 'xamlamoveit_msgs/QueryLock')
    --self.action_client =
    --    actionlib.SimpleActionClient('moveit_msgs/ExecuteTrajectory', 'execute_trajectory', self.node_handle)
end

local function MoveJWorkerCore(self)
    dispatchTrajectory(self)
end

local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
function MoveJWorker:spin()
    local ok, err = xpcall(function() MoveJWorkerCore(self) end, error_msg_func)
    -- abort current trajectory
    if (not ok) and self.currentPlan then
        local traj = self.currentPlan.traj
        if traj.abort ~= nil then
            ros.ERROR('currentPlan failed because of internal error')
            traj:abort()
        end
        self.currentPlan = nil
    end
end

function MoveJWorker:shutdown()
    self.query_resource_lock_service:shutdown()
    self.action_client:shutdown()
end

return MoveJWorker
