local ros = require 'ros'
local tf = ros.tf
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local moveit = require 'moveit'

ros.init('test_action_client')
nh = ros.NodeHandle()
ros.console.setLoggerLevel('actionlib', ros.console.Level.Info)
sp = ros.AsyncSpinner() -- background job
sp:start()

local ac = actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_moveit_action', nh)
local robot_model_loader = moveit.RobotModelLoader('robot_description')
local robot_model = robot_model_loader:getModel()

local all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
local all_group_joint_names = robot_model:getJointModelGroupNames()

ros.spinOnce()
local robot_interface = moveit.MoveGroupInterface(all_group_joint_names[1])
robot_interface:getCurrentState()
robot_interface:setStartStateToCurrentState()
ros.spinOnce()
local current_robot_state = robot_interface:getCurrentState()
print(all_group_joint_names[1])
print(current_robot_state:getVariablePositions())
local testJointPosition = current_robot_state:copyJointGroupPositions(robot_interface:getName())

print("testJointPosition")
print(testJointPosition)
local testJointPosition2 = testJointPosition - 0.1

function testSyncApi()
    print("createGOAl")
    local g = ac:createGoal()

    g.group_name.data = all_group_joint_names[1]
    print(g.waypoints)
    g.waypoints[1] = ros.Message('trajectory_msgs/JointTrajectoryPoint')
    g.waypoints[1].positions = testJointPosition
    local state = ac:sendGoalAndWait(g, 5, 5)
    ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
    local result = ac:getResult()
    ros.INFO('Result:\n%s', result)
end

local function generateIKRequest(group_name, robot_state, avoid_collisions, ik_link_names, poses_stamped)
    local robot_state_msg = robot_state:toRobotStateMsg()
    local ik_spec = ros.MsgSpec('moveit_msgs/PositionIKRequest')
    local req_msg = ros.Message(ik_spec)
    req_msg.group_name = group_name
    req_msg.avoid_collisions = avoid_collisions
    req_msg.robot_state = robot_state_msg
    print(torch.type(robot_state_msg))

    if #ik_link_names > 1 then
        local poses = {}
        for i, k in ipairs(poses_stamped) do
            table.insert(poses, k:toStampedPoseMsg())
        end
        req_msg.ik_link_names = ik_link_names
        req_msg.pose_stamped_vector = poses
    else
        req_msg.ik_link_name = ik_link_names[1]
        req_msg.pose_stamped = poses_stamped[1]:toStampedPoseMsg()
        req_msg.timeout = ros.Duration(0.1)
        req_msg.attempts = 1
    end
    return req_msg
end

function testAsyncApi()
    -- test async api
    local done = false

    function actionDone(state, result)
        ros.INFO('actionDone')
        ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
        ros.INFO('Result:\n%s', result)
        done = true
    end

    function Action_active()
        ros.INFO('Action_active')
    end

    function Action_feedback(feedback)
        ros.INFO('Action_feedback')
        print(feedback)
    end

    local g2 = ac:createGoal()
    g2.group_name.data = all_group_joint_names[1]
    g2.waypoints[1] = ros.Message('trajectory_msgs/JointTrajectoryPoint')
    g2.waypoints[1].positions = testJointPosition2
    ac:sendGoal(g2, actionDone, Action_active, Action_feedback)

    while ros.ok() and not done do
        ros.spinOnce()
    end
end

ros.INFO('waiting for server connection...')
if ac:waitForServer(ros.Duration(5.0)) then
    ros.INFO('connected.')

    testSyncApi()
    testAsyncApi()
else
    ros.ERROR('failed.')
end
sp:stop()
ac:shutdown()
nh:shutdown()
ros.shutdown()
