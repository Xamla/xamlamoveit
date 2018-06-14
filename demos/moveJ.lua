local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local xamlamoveit = require 'xamlamoveit'
local motionLibrary = xamlamoveit.motionLibrary
local JointSet = xamlamoveit.datatypes.JointSet
local JointValues = xamlamoveit.datatypes.JointValues

--[[
    This example works with meca500
    for other setup settings adjustments are necessary
    look for 'Adapt according to your setup'
]]
--Init ros node
ros.init('MoveL')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()

--Create you dedicated motion service
local motion_service = motionLibrary.MotionService(nh)
local move_group_name = '/sda10d/sda10d_r1_controller'
--Query necessary information about setup
local move_group_names, move_groups_details = motion_service:queryAvailableMoveGroups()
assert(table.indexof(move_group_names, move_group_name) > 0)

--Define Xamla move group
local xamla_mg = motionLibrary.MoveGroup(motion_service, move_group_name) -- motion client

local MoveJTest01 =
    JointValues(
    JointSet(
        {
            'arm_left_joint_1_s',
            'arm_left_joint_2_l',
            'arm_left_joint_3_e',
            'arm_left_joint_4_u',
            'arm_left_joint_5_r',
            'arm_left_joint_6_b',
            'arm_left_joint_7_t'
        }
    ),
    torch.Tensor {
        1.0047744512557983,
        -0.65015220642089844,
        -0.56269752979278564,
        -1.969570517539978,
        -0.55776149034500122,
        -0.10201731324195862,
        -0.59777122735977173
    }
)

local MoveJTest02 =
    JointValues(
    JointSet(
        {
            'arm_left_joint_1_s',
            'arm_left_joint_2_l',
            'arm_left_joint_3_e',
            'arm_left_joint_4_u',
            'arm_left_joint_5_r',
            'arm_left_joint_6_b',
            'arm_left_joint_7_t'
        }
    ),
    torch.Tensor {
        1.3624882104493454,
        -0.75471668552025373,
        -0.81698250853724352,
        -2.1300559561207195,
        -0.46127787429665462,
        0.15954084004409735,
        -0.48686285534575319
    }
)

local velocity_scaling = 1
local check_for_collisions = true
--Start motion
tic('startExperiment')
for i = 1, 10 do
    ros.INFO('start moveJoints')
    xamla_mg:moveJoints(MoveJTest01, velocity_scaling, check_for_collisions)
    ros.INFO('finished moveJoints')
    ros.INFO('start moveJoints')
    xamla_mg:moveJoints(MoveJTest02, velocity_scaling, check_for_collisions)
    ros.INFO('finished moveJoints')
end
toc('startExperiment')
-- shutdown ROS
sp:stop()

ros.shutdown()
