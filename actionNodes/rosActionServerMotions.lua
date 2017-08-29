local ros = require 'ros'
local tf = ros.tf
require 'ros.actionlib.ActionServer'
local GoalStatus = require 'ros.actionlib.GoalStatus'
require 'xamlamoveit.xutils.Worker'
local actionlib = ros.actionlib

local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local environmentSetup = xamlamoveit.environmentsetup

local xutils = xamlamoveit.xutils

local nodehandle, sp, worker, service_queue, ik_service_queue

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/QueryMoveGroupInterfaces')
local msg_spec = ros.MsgSpec('xamlamoveit_msgs/MoveGroupInterfaceDescription')

function queryServiceHandler(request, response, header)
    local robot_model_loader = moveit.RobotModelLoader('robot_description')
    local robot_model = robot_model_loader:getModel()

    local all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
    local all_group_joint_names = robot_model:getJointModelGroupNames()

    for k, v in pairs(all_group_joint_names) do
        local l = ros.Message(msg_spec)
        l.name = v
        l.move_group_ids = robot_model:getJointModelSubGroupNames(v)
        table.insert(response.move_group_interfaces, l)
    end
    return true
end

local function initSetup(name)
    ros.init('MoveActions')
    nodehandle = ros.NodeHandle(name)
    service_queue = ros.CallbackQueue()
    ik_service_queue = ros.CallbackQueue()

    sp = ros.AsyncSpinner() -- background job
    worker = Worker(nodehandle)
    sp:start()
end

local function shutdownSetup()
    sp:stop()
    ros.shutdown()
end

local function movePActionServerGoal(goal_handle)
    ros.INFO('movePActionServerGoal')

    local g = goal_handle:getGoal()
    local suc = true
    local traj = {
        starttime = ros.Time.now(),
        goal_handle = goal_handle,
        goal = g,
        accept = function()
            goal_handle:setAccepted('Starting trajectory execution')
            return true
        end,
        proceed = function()
            if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
                return true
            else
                ros.WARN(
                    'Goal status of current trajectory no longer ACTIVE (actual: %d).',
                    goal_handle:getGoalStatus().status
                )
                return false
            end
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
    worker:doTrajectoryAsync(traj) -- queue for processing
end

local function moveJActionServerCancel(goal_handle)
    ros.INFO('moveJActionServerCancel')
    goal_handle:setPreempted(nil, msg or 'Error')
end

local function moveJActionServerGoal(goal_handle, mode)
    ros.INFO('moveJActionServerGoal')
    print(goal_handle)
    local g = goal_handle:getGoal()
    local traj = {
        planning_mode = mode,
        starttime = ros.Time.now(),
        goal_handle = goal_handle,
        goal = g,
        accept = function()
            if goal_handle:getGoalStatus().status == GoalStatus.PENDING then
                goal_handle:setAccepted('Starting trajectory execution')
                return true
            else
                ros.WARN('Status of queued trajectory is not pending but %d.', goal_handle:getGoalStatus().status)
                return false
            end
        end,
        proceed = function()
            if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
                return true
            else
                ros.WARN(
                    'Goal status of current trajectory no longer ACTIVE (actual: %d).',
                    goal_handle:getGoalStatus().status
                )
                return false
            end
        end,
        abort = function(self, msg, code)
            local r = goal_handle:createResult()
            r.result = code or worker.errorCodes.ABORT
            goal_handle:setAborted(r, msg or 'Error')
        end,
        completed = function()
            local r = goal_handle:createResult()
            r.result = worker.errorCodes.SUCCESSFUL
            goal_handle:setSucceeded(r, 'Completed')
        end
    }
    worker:doTrajectoryAsync(traj) -- queue for processing
end

local function moveP_ActionServer_Cancel(goal_handle)
    ros.INFO('moveP_ActionServer_Cancel')
    goal_handle:setPreempted(nil, 'NOT IMPLEMENTED YET')
end

local function moveActionServer(parameter)
    local parameter = parameter or {}
    initSetup(parameter['__name'])

    --moveGroup = initializeMoveGroup()
    local psi = moveit.PlanningSceneInterface()
    environmentSetup.labRoboteur(psi)
    --local dp = moveGroup:getCurrentPose()
    local dt = parameter.frequency or 1 / 125

    ros.console.setLoggerLevel('actionlib', ros.console.Level.Warn)

    local mj_direct = actionlib.ActionServer(nodehandle, 'moveJ_action', 'xamlamoveit_msgs/moveJ')
    local mj_moveit = actionlib.ActionServer(nodehandle, 'moveJ_moveit_action', 'xamlamoveit_msgs/moveJ')
    local mj_moveit_tuned = actionlib.ActionServer(nodehandle, 'moveJ_moveit_tuned_action', 'xamlamoveit_msgs/moveJ')
    local mp = actionlib.ActionServer(nodehandle, 'moveP_action', 'xamlamoveit_msgs/moveP')
    --local ml = actionlib.ActionServer(nodehandle, 'test_action', 'actionlib/Test')

    mj_direct:registerGoalCallback(
        function(gh)
            moveJActionServerGoal(gh, 1)
        end
    )
    mj_direct:registerCancelCallback(moveJActionServerCancel)
    mj_moveit:registerGoalCallback(
        function(gh)
            moveJActionServerGoal(gh, 2)
        end
    )
    mj_moveit:registerCancelCallback(moveJActionServerCancel)
    mj_moveit_tuned:registerGoalCallback(
        function(gh)
            moveJActionServerGoal(gh, 3)
        end
    )
    mj_moveit_tuned:registerCancelCallback(moveJActionServerCancel)

    mp:registerGoalCallback(movePActionServerGoal)
    mp:registerCancelCallback(movePActionServerCancel)

    ros.INFO('Starting action server...')
    mj_direct:start()
    mj_moveit:start()
    mj_moveit_tuned:start()
    mp:start()
    --ml:start()
    local info_server =
        nodehandle:advertiseService('/query_move_group_interface', srv_spec, queryServiceHandler, service_queue)

    while ros.ok() do
        worker:spin()
        if not service_queue:isEmpty() then
            ros.INFO('[!] incoming service call')
            service_queue:callAvailable()
        end
        if not ik_service_queue:isEmpty() then
            ros.INFO('[!] incoming ik service call')
            ik_service_queue:callAvailable()
        end
        ros.spinOnce()
        collectgarbage()
    end

    info_server:shutdown()
    worker:shutdown()
    mj_direct:shutdown()
    mj_moveit:shutdown()
    mj_moveit_tuned:shutdown()
    mp:shutdown()
    --ml:shutdown()
    shutdownSetup()
end

local cmd = torch.CmdLine()
cmd:option('-frequency', 0.008, 'Node cycle time')
local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
moveActionServer(parameter)
