local torch = require 'torch'
local xutils = require 'xamlamoveit.xutils.env'
require 'xamlamoveit.xutils.TrajectorySampler'

local GOAL_CONVERGENCE_POSITION_THRESHOLD = 0.00051 -- in rad
local GOAL_CONVERGENCE_VELOCITY_THRESHOLD = 0.001 -- in rad/s
local MAX_CONVERGENCE_CYCLES = 50

local TrajectoryHandlerStatus = {
    ProtocolError = -3,
    ConnectionLost = -2,
    Canceled = -1,
    Fresh = 0,
    Streaming = 1,
    Flushing = 2,
    Completed = 1000
}

local TrajectoryHandler = torch.class('xamlamoveit.xutils.TrajectoryHandler', xutils)

function TrajectoryHandler:__init(ringSize, servoTime, reverseConnection, traj, flush, waitCovergence, maxBuffering, logger)
    self.ringSize = ringSize
    self.traj = traj
    self.flush = flush
    self.waitCovergence = waitCovergence
    self.maxBuffering = maxBuffering or ringSize
    self.logger = logger
    self.noResponse = 0
    self.status = TrajectoryHandlerStatus.Fresh
    self.sampler = TrajectorySampler(traj, servoTime)
    self.noResponse = 0
    self.convergenceCycle = 0
    self.reverseConnection = reverseConnection
end

function TrajectoryHandler:cancel()
    if self.status > 0 then
        self.reverseConnection:cancel() -- send cancel message to robot
        self.status = TrajectoryHandlerStatus.Canceled
    end
end



local function reachedGoal(self)
    local state_pos = self.traj.joint_monitor:getNextPositionsTensor()
    local feedback_idx =
        table.findIndicesTensor(
        self.traj.state_joint_names,
        function(x)
            return table.indexof(self.traj.joint_names, x) > -1
        end
    )

    local q_goal = self.sampler:getGoalPosition()
    local q_actual = state_pos[feedback_idx]
    --self.realtimeState.q_actual
    --local qd_actual = state_vel[feedback_idx]

    local goal_distance = torch.norm(q_goal - q_actual)

    self.logger.info('Convergence cycle %d: goal_distance (joints): %f;', self.convergenceCycle, goal_distance)

    self.convergenceCycle = self.convergenceCycle + 1
    if self.convergenceCycle >= MAX_CONVERGENCE_CYCLES then
        error(string.format('Did not reach goal after %d convergence cycles.', MAX_CONVERGENCE_CYCLES))
    end

    return goal_distance < GOAL_CONVERGENCE_POSITION_THRESHOLD
end

function TrajectoryHandler:update()
    if self.status < 0 or self.status == TrajectoryHandlerStatus.Completed then
        return false
    end

    if self.sampler:atEnd() and self.flush == false then
        self.status = TrajectoryHandlerStatus.Completed
        return false -- all data sent, nothing to do
    end

    -- self.logger.debug('avail: %d', avail)

    if not self.sampler:atEnd() then -- if trajectory is not at end
        self.status = TrajectoryHandlerStatus.Streaming

        local pts = self.sampler:generateNextPoints(1) -- send new trajectory points via reverse conncection
        self.reverseConnection:sendPoints(pts, self.traj.joint_names)
    else -- all servo points have been sent, wait for robot to empty its queue
        self.reverseConnection:sendPoints({}, self.traj.joint_names) -- send zero count
        if (reachedGoal(self) or not self.waitCovergence) then -- when buffer is empty we are done
            self.status = TrajectoryHandlerStatus.Completed
            return false
        else
            self.status = TrajectoryHandlerStatus.Flushing
        end
    end

    return true -- not yet at end of trajectory
end

return TrajectoryHandler
