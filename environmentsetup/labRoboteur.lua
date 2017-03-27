local environmentSetup = require 'xamlamoveit.environmentsetup.env'
local ros = require 'ros'
local tf = ros.tf


function environmentSetup.labRoboteur(planningSceneInterface)
  local planningSceneInterface = planningSceneInterface
  assert(planningSceneInterface)

  local rot = tf.Transform():setRotation(tf.Quaternion():setRPY(0,0,45,1))
    -- collision box for table
  local box_pose = rot:mul(tf.Transform():setOrigin({0.11, 0.29, -0.431}))

  planningSceneInterface:addBox('ground', 0.9, 0.80, 0.8, box_pose)

  -- collision box for wall A
  local wall_a_pose = tf.Transform():setOrigin({-0.0, -0.79, 0.0}):setRotation(tf.Quaternion():setRPY(0,0,0,1))
  planningSceneInterface:addBox('WallA', 10.1, 0.1, 10.8, wall_a_pose:fromTensor(box_pose:toTensor()*wall_a_pose:toTensor()))

  -- collision box for wall B
  local wall_b_pose = tf.Transform():setOrigin({-0.8, -0.0, 0.0}):setRotation(tf.Quaternion():setRPY(0,0,90,1))
  planningSceneInterface:addBox('WallB', 10.1, 0.1, 10.8, wall_b_pose:fromTensor(box_pose:toTensor()*wall_b_pose:toTensor()))

  -- Collision box for CamLeft
  --local box_CamLeft_pose = tf.Transform():setOrigin({0.08, 0.36, 0.6}):setRotation(tf.Quaternion():setRPY(0,0,-90,1))
  --planningSceneInterface:addBox('CamLeft',0.18, 0.18, 0.4, box_CamLeft_pose:fromTensor(box_pose:toTensor()*box_CamLeft_pose:toTensor()))

  -- Collision box for CamRight
  local box_CamPost_pose = tf.Transform():setOrigin({0.43, 0.38, 0.70}):setRotation(tf.Quaternion():setRPY(0,0,0,1))
  planningSceneInterface:addBox('CamPost', 0.05, 0.05, 0.7, box_CamPost_pose:fromTensor(box_pose:toTensor()*box_CamPost_pose:toTensor()))

-- Collision box for CamRight
  local box_Cam_pose = tf.Transform():setOrigin({-0.15, 0.0, 0.25}):setRotation(tf.Quaternion():setRPY(0,0,0,1))
  planningSceneInterface:addBox('Cam', 0.35, 0.24, 0.2, box_Cam_pose:fromTensor(box_CamPost_pose:toTensor()*box_Cam_pose:toTensor()))


  print('MoveIt planning scene configured.')
end

return environmentSetup