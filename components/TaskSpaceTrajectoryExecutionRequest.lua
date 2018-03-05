local ros = require 'ros'
local components = require 'xamlamoveit.components.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local xutils = require 'xamlamoveit.xutils'
local epsilon = 1e-2

local TaskSpaceTrajectoryExecutionRequest = torch.class('xamlamoveit.components.TaskSpaceTrajectoryExecutionRequest', components)

local errorCodes = require 'xamlamoveit.core.ErrorCodes'.error_codes
errorCodes = table.merge(errorCodes, table.swapKeyValue(errorCodes))

function TaskSpaceTrajectoryExecutionRequest:__init(goal_handle)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
    self.check_collision = self.goal_handle.goal.goal.check_collision
end

function TaskSpaceTrajectoryExecutionRequest:accept()
    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.goal_handle:setAccepted('Starting trajectory execution')
        self.starttime_debug = ros.Time.now()
        return true
    else
        ros.WARN('Status of queued trajectory is not pending but %d.', self.goal_handle:getGoalStatus().status)
        return false
    end
end


function TaskSpaceTrajectoryExecutionRequest:proceed()
    ros.DEBUG('proceed')
    local status = self.goal_handle:getGoalStatus().status
    if  status == GoalStatus.ACTIVE or status == GoalStatus.PENDING or status == GoalStatus.PREEMPTING then

        return true
    else
        ros.DEBUG(
            '[TaskSpaceTrajectoryExecutionRequest] Goal status of current trajectory no longer ACTIVE (actual: %d).',
            self.goal_handle:getGoalStatus().status
        )
        return false
    end
end

function TaskSpaceTrajectoryExecutionRequest:abort(msg, code)
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        local code = code or errorCodes.FAILURE
        local r = self.goal_handle:createResult()
        r.result = code
        ros.WARN(tostring(r))
        ros.WARN(tostring(msg))
        ros.WARN(tostring(code))
        self.goal_handle:setAborted(r, msg or 'Error')
    elseif status == GoalStatus.PREEMPTING then
        ros.INFO("Notifying client about PREEMTING state")
        local code = code or errorCodes.PREEMPTED
        local r = self.goal_handle:createResult()
        r.result = code
        self.goal_handle:setAborted(r, msg or 'Abort')
    elseif status == GoalStatus.RECALLING then
        ros.INFO("Notifying client about RECALLING state")
        self.goal_handle:setCanceled(nil, msg or 'Canceled')
    else
        ros.INFO("nothing to be done")
    end
    collectgarbage()
end

function TaskSpaceTrajectoryExecutionRequest:completed()
    local r = self.goal_handle:createResult()
    r.result = errorCodes.SUCCESS
    self.goal_handle:setSucceeded(r, 'Completed')
    collectgarbage()
end

function TaskSpaceTrajectoryExecutionRequest:cancel()
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        self.goal_handle:setCancelRequested()
    end
end

return TaskSpaceTrajectoryExecutionRequest
