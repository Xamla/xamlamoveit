luaunit = require 'luaunit'
local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
ros.init('TestPlanningService')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()
local datatypes = require 'xamlamoveit.datatypes'
local mc = require 'xamlamoveit.motionLibrary'.MotionService(nh) -- motion client
local move_group_names, move_group_details = mc:queryAvailableMoveGroups()
local move_group = move_group_names[1]
local current_joint_values = mc:queryJointState(move_group_details[move_group].joint_names)
local plan_parameters = mc:getDefaultPlanParameters(move_group, move_group_details[move_group].joint_names)

--os.exit()
TestPlanningService = {}

function TestPlanningService:testMoveit()
    local joint_set_A = datatypes.JointSet(move_group_details[move_group].joint_names)

    local joint_values_A = datatypes.JointValues(joint_set_A, current_joint_values)
    local joint_values_B = joint_values_A * 0.9
    local ok, joint_path = mc:planJointPath(joint_values_A.values, joint_values_B.values, plan_parameters)

    luaunit.assertEquals(ok, true)
    print('joint_values_A.values', joint_values_A.values[1])
    print('joint_values_B.values', joint_values_B.values[1])
    print(joint_path[{joint_path:size(1), 1}])
    local norm = (joint_path[{joint_path:size(1), {}}] - joint_values_B.values):norm()
    luaunit.assertAlmostEquals(norm, 0, 1e-12)
    local success, joint_trajectory = mc:planMoveJoint(joint_path, plan_parameters)
    luaunit.assertEquals(success, 1)
    local num_points = #joint_trajectory.points
    local result_joint_set = datatypes.JointSet(joint_trajectory.joint_names)
    local result = datatypes.JointValues(result_joint_set, joint_trajectory.points[num_points].positions)

    norm = (result:select(joint_set_A.joint_names).values - joint_values_B.values):norm()
    luaunit.assertAlmostEquals(norm, 0, 1e-6)
end

function TestPlanningService:testTvp()
    local joint_set_A = datatypes.JointSet(move_group_details[move_group].joint_names)

    local joint_values_A = datatypes.JointValues(joint_set_A, current_joint_values)
    local joint_values_B = joint_values_A * 2.9
    local success,
        joint_trajectory = mc:planMoveJoint(torch.cat(joint_values_A.values, joint_values_B.values, 2):t(), plan_parameters)
    luaunit.assertEquals(success, 1)
    local num_points = #joint_trajectory.points
    local result_joint_set = datatypes.JointSet(joint_trajectory.joint_names)
    local result = datatypes.JointValues(result_joint_set, joint_trajectory.points[num_points].positions)

    norm = (result:select(joint_set_A.joint_names).values - joint_values_B.values):norm()
    luaunit.assertAlmostEquals(norm, 0, 1e-12)
end


function TestPlanningService:testMoveInSingleAxisWithTvp()
    local joint_set_A = datatypes.JointSet(move_group_details[move_group].joint_names)

    local joint_values_A = datatypes.JointValues(joint_set_A, current_joint_values)
    local joint_values_B = joint_values_A:clone()
    joint_values_B.values[6] = joint_values_B.values[6] + 0.4
    local success,
        joint_trajectory = mc:planMoveJoint(torch.cat(joint_values_A.values, joint_values_B.values, 2):t(), plan_parameters)
    luaunit.assertEquals(success, 1)
    local num_points = #joint_trajectory.points
    local result_joint_set = datatypes.JointSet(joint_trajectory.joint_names)
    local result = datatypes.JointValues(result_joint_set, joint_trajectory.points[num_points].positions)

    norm = (result:select(joint_set_A.joint_names).values - joint_values_B.values):norm()
    luaunit.assertAlmostEquals(norm, 0, 1e-12)
end


os.exit(luaunit.LuaUnit.run())
