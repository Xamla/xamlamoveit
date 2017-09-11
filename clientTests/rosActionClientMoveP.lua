local ros = require "ros"
local tf = ros.tf
require "ros.actionlib.SimpleActionClient"
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local moveit = require 'moveit'
ros.init("test_movep_action_client")
nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()  -- background job
  sp:start()

ros.console.setLoggerLevel("actionlib", ros.console.Level.Info)
 local manipulator = moveit.MoveGroupInterface("arm_left")
local ac = actionlib.SimpleActionClient("xamlamoveit_msgs/moveP", "moveP_action", nh)

local testPose2 = manipulator:getCurrentPose() -- tf.Transform():setOrigin({(0.84285), 0.335428, 1.12581})
--local rot = testPose:getRotation()
--rot:setRPY(math.pi, 0, 0)
--testPose:setRotation(tf.Quaternion(0.8018,-0.27319, -0.531491, 0.000000001))
local offset = tf.Transform():setOrigin({0.0, 0.04, 0.0})
local testPose = manipulator:getCurrentPose():mul(offset) --
--testPose2:setRotation(rot)

local function generateIKRequest(group_name, robot_state, avoid_collisions, ik_link_names, poses_stamped)
    local robot_state_msg
    if robot_state then
        robot_state_msg = robot_state:toRobotStateMsg()
    end
    local ik_spec = ros.MsgSpec("moveit_msgs/PositionIKRequest")
    local req_msg = ros.Message(ik_spec)
    req_msg.group_name = group_name
    req_msg.avoid_collisions = avoid_collisions
    if robot_state_msg then
        req_msg.robot_state = robot_state_msg
        print(torch.type(robot_state_msg))
    end

    if #ik_link_names == 1 then
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

function testSyncApi()
    local g = ac:createGoal()
    print(g)
    g.waypoints[1] = generateIKRequest("arm_left", nil, false, {"arm_left_link_tool0"}, {tf.StampedTransform(testPose2)})

    local state = ac:sendGoalAndWait(g, 5, 5)
    ros.INFO("Finished with states: %s (%d)", SimpleClientGoalState[state], state)
    local result = ac:getResult()
    ros.INFO("Result:\n%s", result)
end

function testAsyncApi()
    -- test async api
    local done = false

    function actionDone(state, result)
        ros.INFO("actionDone")
        ros.INFO("Finished with states: %s (%d)", SimpleClientGoalState[state], state)
        ros.INFO("Result:\n%s", result)
        done = true
    end

    function actionActive()
        ros.INFO("actionActive")
    end

    function actionFeedback(feedback)
        ros.INFO("actionFeedback")
    end

    local g2 = ac:createGoal()
    g2.waypoints[1] = generateIKRequest("arm_left", nil, false, {"arm_left_link_tool0"}, {tf.StampedTransform(testPose)})
    ac:sendGoal(g2, actionDone, actionActive, actionFeedback)

    while ros.ok() and not done do
        ros.spinOnce()
    end
end

ros.INFO("waiting for server connection...")
if ac:waitForServer(ros.Duration(5.0)) then
    testAsyncApi()
    ros.INFO("connected.")

    testSyncApi()
else
    ros.ERROR("failed.")
end
  sp:stop()
ac:shutdown()
nh:shutdown()
ros.shutdown()
