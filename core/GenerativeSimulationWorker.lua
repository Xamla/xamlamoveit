--[[
GenerativeSimulationWorker.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local TrajectoryHandler = require 'xamlamoveit.core.TrajectoryHandler'

local DEFAULT_SERVO_TIME = 0.008
local DEFAULT_MAX_IDLE_CYCLES = 250 -- number of cycles before simulated reverse connection disconnects
local DEFAULT_REVERSE_CONNECTION_DELAY = 0.3 -- delay in seconds (e.g. to simulate script upload to UR)

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

--- SimulationConnection class
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

--- GenerativeSimulationWorker class
local GenerativeSimulationWorker = torch.class('GenerativeSimulationWorker')

function GenerativeSimulationWorker:__init(nh)
    self.trajectoryQueue = {} -- list of pending trajectories
    self.syncCallbacks = {}
    self.nodehandle = nh
    self.errorCodes = errorCodes
    self.logger = logger
    self.servoTime = DEFAULT_SERVO_TIME
    self.reverseConnectionDelay = DEFAULT_REVERSE_CONNECTION_DELAY
    self.idleCycles = 0
    self.maxIdleCycles = DEFAULT_MAX_IDLE_CYCLES
    self.reverseConnectionEstablished = false
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

function GenerativeSimulationWorker:cancelCurrentTrajectory()
    if self.currentTrajectory == nil then
        return
    end

    self.logger.info('[GenerativeSimulationWorker] Cancelling trajectory execution.')
    local traj = self.currentTrajectory.traj
    local handler = self.currentTrajectory.handler
    -- we try to bring the robot gracefully to a standstill
    handler:cancel()
    if traj.cancel ~= nil then
        traj:cancel() -- cancel callback (e.g. enter canel requested state)
    end
end

local error_msg_func = function(x)
    ros.ERROR(debug.traceback())
    return x
end
local function dispatchTrajectory(self)
    if self.currentTrajectory == nil then
        if #self.trajectoryQueue > 0 then -- check if new trajectory is available
            -- simulate reverse connection delay
            if not self.reverseConnectionEstablished then
                if self.reverseConnectionDelay > 0 then
                    sys.sleep(self.reverseConnectionDelay) -- sleep (simulate blocking accept call in UR driver)
                end
                self.reverseConnectionEstablished = true
                self.idleCycles = 0
            end

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

        if ( handler.status == TrajectoryHandlerStatus.Canceled or traj.proceed == nil or traj:proceed()) then
            -- execute main update call
            local ok,
                err =
                xpcall(
                function()
                    handler:update()
                end,
                error_msg_func
            )
            if not ok then
                self.logger.warn('Exception during handler update: %s', err)
            end

            if not ok or handler.status < 0 then -- error
                if traj.abort ~= nil then
                    local msg
                    if handler.status == TrajectoryHandlerStatus.Canceled then
                        msg = 'Robot was stopped due to trajectory cancel request.'
                    end
                    traj:abort(msg) -- abort callback
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
            self:cancelCurrentTrajectory()
        end
    else
        self.idleCycles = self.idleCycles + 1
        if self.reverseConnectionEstablished and self.idleCycles > self.maxIdleCycles then
            self.logger.info(
                '[GenerativeSimulationWorker] Robot was idle for %d cycles. Closing reverse connection.',
                self.idleCycles
            )
            self.reverseConnectionEstablished = false
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
