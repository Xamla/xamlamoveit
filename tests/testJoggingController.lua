local Controller = require 'xamlamoveit.controller'
local ros = require 'ros'
tf = ros.tf
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
local printf = xutils.printf
local joint_traj_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')




local function initializeMoveIt(groupName, velocityScaling)
  local velocityScaling = velocityScaling or 0.5

  local planningSceneInterface = moveit.PlanningSceneInterface()
  local manipulator = moveit.MoveGroupInterface(groupName or 'manipulator')

  manipulator:setMaxVelocityScalingFactor(velocityScaling)
  manipulator:setGoalTolerance(1E-5)
  manipulator:setPlanningTime(10.0)

  -- ask move group for current state
  manipulator:getCurrentState()
  manipulator:setStartStateToCurrentState()
  local currentPose = manipulator:getCurrentPose():toTensor()
  ros.INFO('Current robot pose:')
  print(currentPose)

  ros.INFO(string.format('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink()))
  return manipulator, planningSceneInterface
end

--@param desired joint angle position
function sendPositionCommand( q_dot, names, publisherPointPositionCtrl)
  ros.INFO('sendPositionCommand')
  assert(q_dot:size(1) == #names)
  local m = ros.Message(joint_traj_spec)
  local mPoint = ros.Message(joint_point_spec)

  m.joint_names = {}
  for ii = 1, q_dot:size(1) do
    m.joint_names[ii] = names[ii]
  end
  mPoint.velocities:set(q_dot) --TODO this is probably not optimal.
  mPoint.time_from_start = ros.Duration(0.008)
  m.points = {mPoint}
  print(m)
  publisherPointPositionCtrl:publish(m)
end

function main()

  ros.init('JoggingTest')
  BEGIN_EXECUTION = ros.Time.now()
  -- What does this command?
  local nodehandle = ros.NodeHandle()
  local sp = ros.AsyncSpinner()  -- background job
  sp:start()

  local moveGroup, psi = initializeMoveIt("arm_left")
  local myTopic = "/jogging_joystick_server/jogging_command"
  local publisherPointPositionCtrl = nodehandle:advertise(myTopic, joint_traj_spec)
  local set_bool_spec = ros.SrvSpec('std_srvs/SetBool')
  local start_stop_controll_client = nodehandle:serviceClient('/jogging_joystick_server/start_stop_tracking', set_bool_spec)

  local names = std.StringVector()
  moveGroup:getActiveJoints(names)
  names = names:totable()

  local get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')

  local stepSize = 0.001 --rad/s
  local q_dot = torch.zeros(1)
  while ros.ok() do
      local input = io.read()
      if input == '1' then q_dot[1] = stepSize
        sendPositionCommand(q_dot, {names[1]}, publisherPointPositionCtrl)
      elseif input == '2' then q_dot[1] = stepSize
        sendPositionCommand(q_dot, {names[2]}, publisherPointPositionCtrl)
      elseif input == '3' then
        q_dot[1] = stepSize
        sendPositionCommand(q_dot, {names[3]}, publisherPointPositionCtrl)
      elseif input == '4' then
        q_dot[1] = stepSize
        sendPositionCommand(q_dot, {names[4]}, publisherPointPositionCtrl)
      elseif input == '5' then
        q_dot[1] = stepSize
        sendPositionCommand(q_dot, {names[5]}, publisherPointPositionCtrl)
      elseif input == '6' then
        q_dot[1] = stepSize
        sendPositionCommand(q_dot, {names[6]}, publisherPointPositionCtrl)
      elseif input == '-1' then
        q_dot[1] = -stepSize
        sendPositionCommand(q_dot, {names[1]}, publisherPointPositionCtrl)
      elseif input == '-2' then
        q_dot[1] = -stepSize
        sendPositionCommand(q_dot, {names[2]}, publisherPointPositionCtrl)
      elseif input == '-3' then
        q_dot[1] = -stepSize
        sendPositionCommand(q_dot, {names[3]}, publisherPointPositionCtrl)
      elseif input == '-4' then
        q_dot[1] = -stepSize
        sendPositionCommand(q_dot, {names[4]}, publisherPointPositionCtrl)
      elseif input == '-5' then
        q_dot[1] = -stepSize
        sendPositionCommand(q_dot, {names[5]}, publisherPointPositionCtrl)
      elseif input == '-6' then
        q_dot[1] = -stepSize
        sendPositionCommand(q_dot, {names[6]}, publisherPointPositionCtrl)
      elseif input == 'm' then
        local req_msg = start_stop_controll_client:createRequest()
        req_msg:fillFromTable({data = true})
        print(start_stop_controll_client:call(req_msg))
      elseif input == 's' then
        local req_msg = start_stop_controll_client:createRequest()
        req_msg:fillFromTable({data = false})
        print(start_stop_controll_client:call(req_msg))
      elseif input == 'q' then
        break;
      end
      ros:spinOnce()
  end

  publisherPointPositionCtrl:shutdown()
  sp:stop()
  sys.sleep(1.0)

  ros.shutdown()
end


main()
