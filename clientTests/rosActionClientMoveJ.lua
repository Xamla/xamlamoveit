#!/usr/bin/env th
local ros = require 'ros'
local tf = ros.tf
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

ros.init('test_action_client')
nh = ros.NodeHandle()
ros.console.setLoggerLevel('actionlib', ros.console.Level.Debug)


local ac = actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', nh)
local testJointPosition = torch.Tensor({ 1.2658,
                                  -1.9922,
                                  2.3327,
                                  -3.0303,
                                  -1.6643,
                                  2.3804})
local testJointPosition2 = torch.Tensor({ 1.2658,
                                  -1.9922,
                                  1.3327,
                                  -2.0303,
                                  -1.6643,
                                  2.3804})



function testSyncApi()
  local g = ac:createGoal()

  g.group_name.data = 'manipulator'
  g.goal.positions = testJointPosition
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
    for i,k in ipairs(poses_stamped) do
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
  g2.group_name.data = 'manipulator'
  g2.goal.positions = testJointPosition2
  ac:sendGoal(g2, actionDone, Action_active, Action_feedback)

  while ros.ok() and not done do
    ros.spinOnce()
  end
end


print('waiting for server connection...')
if ac:waitForServer(ros.Duration(5.0)) then
  print('connected.')

  testSyncApi()
  testAsyncApi()

else
  print('failed.')
end


ac:shutdown()
nh:shutdown()
ros.shutdown()
