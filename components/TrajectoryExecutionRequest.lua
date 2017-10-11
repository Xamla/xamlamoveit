local ros = require 'ros'

local components = require 'xamlamoveit.components.env'
local TrajectoryExecutionRequest = torch.class('xamlamoveit.components.TrajectoryExecutionRequest', components)

local GoalStatus = require 'ros.actionlib.GoalStatus'

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
-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
local TrajectoryResultStatus = {
    SUCCESSFUL = 0,
    INVALID_GOAL = -1,
    INVALID_JOINTS = -2,
    OLD_HEADER_TIMESTAMP = -3,
    PATH_TOLERANCE_VIOLATED = -4,
    GOAL_TOLERANCE_VIOLATED = -5
}

local function checkConvergence(cq, target, jointNames)
    local fullJointStateNames = cq:getVariableNames()
    local currrentPosition = cq:getVariablePositions()
    local sum = 0
    for i, v in ipairs(jointNames) do
        if v == fullJointStateNames[i] then
            sum = sum + math.abs(target[i] - currrentPosition[i])
        end
    end
    if (sum / #jointNames) < 1e-4 then
        return true
    else
        return false
    end
end

function TrajectoryExecutionRequest:__init(goal_handle)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle
    self.manipulator = nil
    self.joint_monitor = nil
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
end

function TrajectoryExecutionRequest:accept()
    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.goal_handle:setAccepted('Starting trajectory execution')
        self.starttime_debug = ros.Time.now()
        return true
    else
        ros.WARN('Status of queued trajectory is not pending but %d.', self.goal_handle:getGoalStatus().status)
        return false
    end
end

-- Returns joints positions at time 't'
local function interpolateCubic(t, t0, t1, p0, p1, v0, v1)
    local dt = t1 - t0
    if dt < 1e-6 then
        return p1, p0:clone():zero()
    end
    local pos = p0:clone()
    local vel = p0:clone()
    for i = 1, p0:size(1) do
        local a = p0[i]
        local b = v0[i]
        local c = (-3 * p0[i] + 3 * p1[i] - 2 * dt * v0[i] - dt * v1[i]) / dt ^ 2
        local d = (2 * p0[i] - 2 * p1[i] + dt * v0[i] + dt * v1[i]) / dt ^ 3
        pos[i] = a + b * t + c * t ^ 2 + d * t ^ 3
        vel[i] = b + 2 * c * t + 3 * d * t ^ 2
    end
    return pos, vel
end


function TrajectoryExecutionRequest:proceed()
    ros.INFO('proceed')
    if self.goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
        if self.manipulator == nil then
            ros.ERROR('[TrajectoryExecutionRequest] move group interface is nil')
            self.status = errorCodes.ABORT
            return false
        elseif self.joint_monitor then
            local now = ros.Time.now()
            local p, l = self.joint_monitor:getNextPositionsTensor()

            local traj = self.goal.goal.trajectory
            local index = 1
            local dist_to_start = (traj.points[index].positions - p):norm()
            if dist_to_start == 0 then
                self.starttime = now
            end
            local t = now - self.starttime
            if t < ros.Duration(0) then
                t = ros.Duration(0)
            end

            while traj.points[index].time_from_start:toSec() < t:toSec() and index < #traj.points do
                index = index + 1
            end
            --if traj.points[index].time_from_start:toSec() > t:toSec() then
            --    index = index -1
            --end
            local k = math.min(index + 1, #traj.points)
            local t0, t1 = traj.points[index].time_from_start:toSec(), traj.points[k].time_from_start:toSec()
            local p0, v0 = traj.points[index].positions, traj.points[index].velocities
            local p1, v1 = traj.points[k].positions, traj.points[k].velocities
            local q, qd = interpolateCubic(t:toSec() - t0, t0, t1, p0, p1, v0, v1)
            local delta = torch.abs(q - p)
            ros.INFO('time planned after start: ' .. t0)
            ros.INFO('time after start: ' .. t:toSec())
            ros.INFO('time after index: %d, %f sec ',index, t:toSec() - t0)
            ros.INFO('position error: ' .. tostring(delta))
            ros.INFO('Latency: ' .. tostring(l))
            if now:toSec() - self.starttime_debug:toSec() > ros.Duration(5):toSec() then
                ros.ERROR('[TrajectoryExecutionRequest] Trajectory start is not working!!')
                self.status = errorCodes.CONTROL_FAILED
                return false
            end
            if delta:gt(0.3):sum() > 0 then
                ros.ERROR('[TrajectoryExecutionRequest] joint tracking error is too big!!')
                self.status = errorCodes.CONTROL_FAILED
                return false
            end
        end
        ros.INFO('moveing')
        return true
    else
        ros.WARN(
            '[TrajectoryExecutionRequest] Goal status of current trajectory no longer ACTIVE (actual: %d).',
            self.goal_handle:getGoalStatus().status
        )
        return false
    end
end

function TrajectoryExecutionRequest:abort(msg, code)
    local code = code or errorCodes.FAILURE
    local r = self.goal_handle:createResult()
    r.result = code
    ros.WARN(tostring(r))
    ros.WARN(tostring(msg))
    ros.WARN(tostring(code))
    self.goal_handle:setAborted(r, msg or 'Error')
    if self.joint_monitor then
        self.joint_monitor:shutdown()
        self.joint_monitor = nil
    end
end

function TrajectoryExecutionRequest:completed()
    local r = self.goal_handle:createResult()
    r.result = errorCodes.SUCCESS
    print(r)
    self.goal_handle:setSucceeded(r, 'Completed')
    if self.joint_monitor then
        self.joint_monitor:shutdown()
        self.joint_monitor = nil
    end
end

return TrajectoryExecutionRequest
