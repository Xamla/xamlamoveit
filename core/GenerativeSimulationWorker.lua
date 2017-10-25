local ros = require 'ros'
local TrajectoryHandler = require 'xamlamoveit.core.TrajectoryHandler'

local TrajectoryHandlerStatus = {
    ProtocolError = -3,
    ConnectionLost = -2,
    Canceled = -1,
    Fresh = 0,
    Streaming = 1,
    Flushing = 2,
    Completed = 1000
}

local logger = {
    debug = ros.DEBUG,
    info = ros.INFO,
    warn = ros.WARN,
    error = ros.ERROR
}

local errorCodes = {}
errorCodes.SUCCESSFUL = 1
errorCodes.INVALID_GOAL = -1
errorCodes.ABORT = -2
errorCodes.NO_IK_FOUND = -3
errorCodes.INVALID_LINK_NAME = -4

local SimulationConnection = torch.class('SimulationConnection')
function SimulationConnection:__init(nh, logger)
    self.nodehandle = nh
    self.joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
    self.joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
    self.publisher_point_position_ctrl = self.nodehandle:advertise('joint_command', self.joint_pos_spec)
end

--@param desired joint angle position
function SimulationConnection:sendPositionCommand(q_des, q_dot, joint_names)
    local m = ros.Message(self.joint_pos_spec)
    local mPoint = ros.Message(self.joint_point_spec)
    m.joint_names = {}
    for ii = 1, q_des:size(1) do
        m.joint_names[ii] = joint_names[ii]
    end
    mPoint.positions:set(q_des:clone())
    if not (q_dot == nil) then
        mPoint.velocities:set(q_dot:clone())
    end
    mPoint.time_from_start = ros.Duration(0.008)
    --ros.Time.now() - BEGIN_EXECUTION
    m.points = {mPoint}
    m.header.stamp = ros.Time.now()
    self.publisher_point_position_ctrl:publish(m)
end

function SimulationConnection:sendPoints(points, joint_names)
    for j = 1, #points do
        local q = points[j]
        self:sendPositionCommand(q, nil, joint_names)
    end
end

function SimulationConnection:cancel()
end
----

local GenerativeSimulationWorker = torch.class('GenerativeSimulationWorker')

function GenerativeSimulationWorker:__init(nh)
    self.trajectoryQueue = {} -- list of pending trajectories
    self.syncCallbacks = {}
    self.nodehandle = nh
    self.errorCodes = errorCodes
    self.logger = logger
    self.servoTime = 0.008
    self.reverseConnection = SimulationConnection.new(nh, logger)
end

function GenerativeSimulationWorker:doTrajectoryAsync(traj)
    table.insert(self.trajectoryQueue, traj)
end

function GenerativeSimulationWorker:addSyncCallback(fn)
    table.insert(self.syncCallbacks, fn)
end

function GenerativeSimulationWorker:removeSyncCallback(fn)
    for i, x in ipairs(self.syncCallbacks) do
        if x == fn then
            table.remove(self.syncCallbacks, i)
            return
        end
    end
end

function GenerativeSimulationWorker:sync()
    for i, fn in ipairs(self.syncCallbacks) do
        fn(self)
    end
    return true
end

function GenerativeSimulationWorker:cancelCurrentPlan(abortMsg)
    if self.currentPlan ~= nil then
        if callAbortCallback then
            local traj = self.currentPlan.traj
            if traj.abort ~= nil then
                --TODO
                traj:abort(abortMsg or 'Canceled') -- abort callback
            end
        end
        self.currentPlan = nil
    end
end

function GenerativeSimulationWorker:createTrajectoryHandler(traj, flush, waitCovergence, maxBuffering)
    if flush == nil then
        flush = true
    end
    if waitCovergence == nil then
        waitCovergence = true
    end
    return TrajectoryHandler.new(
        self.ringSize or 10,
        self.servoTime or 0.008,
        self.reverseConnection,
        traj,
        flush,
        waitCovergence,
        maxBuffering,
        self.logger
    )
end

function GenerativeSimulationWorker:cancelCurrentTrajectory(abortMsg)
    if self.currentTrajectory ~= nil then
        self.logger.info('[GenerativeSimulationWorker] Cancelling trajectory execution.')
        local handler = self.currentTrajectory.handler
        if callAbortCallback then
            local traj = self.currentTrajectory.traj
            if traj.abort ~= nil then
                traj:abort(abortMsg or 'Canceled') -- abort callback
            end
        end
        self.currentTrajectory = nil
        handler:cancel()
    end
end

local function dispatchTrajectory(self)
    if self.currentTrajectory == nil then
        if #self.trajectoryQueue > 0 then -- check if new trajectory is available
            while #self.trajectoryQueue > 0 do
                local traj = table.remove(self.trajectoryQueue, 1)
                if traj.accept == nil or traj:accept() then -- call optional accept callback
                    local flush, waitCovergence = true, true
                    local maxBuffering = self.ringSize

                    if traj.flush ~= nil then
                        flush = traj.flush
                    end

                    if traj.waitCovergence ~= nil then
                        waitCovergence = traj.waitCovergence
                    end

                    if traj.maxBuffering ~= nil then
                        maxBuffering = math.max(1, traj.maxBuffering)
                    end

                    self.currentTrajectory = {
                        startTime = sys.clock(), -- debug information
                        traj = traj,
                        handler = self:createTrajectoryHandler(traj, flush, waitCovergence, maxBuffering)
                    }
                    break
                end
            end
        end
    end

    -- ensure first points are send to robot immediately after accepting trajectory execution
    if self.currentTrajectory ~= nil then -- if we have an exsting trajectory
        local traj = self.currentTrajectory.traj
        local handler = self.currentTrajectory.handler

        -- check if trajectory execution is still desired (e.g. not canceled)
        if (traj.proceed == nil or traj:proceed()) then
            -- execute main update call
            local ok,
                err =
                pcall(
                function()
                    handler:update()
                end
            )
            if not ok then
                self.logger.warn('Exception during handler update: %s', err)
            end

            if not ok or handler.status < 0 then -- error
                if traj.abort ~= nil then
                    traj:abort() -- abort callback
                end
                self.currentTrajectory = nil
            elseif handler.status == TrajectoryHandlerStatus.Completed then
                if traj.completed ~= nil then
                    traj:completed() -- completed callback
                end
                self.currentTrajectory = nil
            end
        else
            -- robot not ready or proceed callback returned false
            self:cancelCurrentTrajectory('Robot not ready or proceed callback returned false.')
        end
    end
end

local function workerCore(self)
    dispatchTrajectory(self)
end

function GenerativeSimulationWorker:spin()
    local ok,
        err =
        pcall(
        function()
            workerCore(self)
        end
    )
    -- abort current trajectory
    if (not ok) and self.currentPlan then
        local traj = self.currentPlan.traj
        if traj.abort ~= nil then
            traj:abort()
        end
        self.currentPlan = nil
    end
end

function GenerativeSimulationWorker:shutdown()
    if self.publisher_point_position_ctrl then
        self.publisher_point_position_ctrl:shutdown()
    end
end

return GenerativeSimulationWorker
