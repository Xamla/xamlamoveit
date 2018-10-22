--[[
moveInSteps.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local xutils = require 'xamlamoveit.xutils'
local error_codes = require 'xamlamoveit.core.ErrorCodes'.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))
local tf = ros.tf
local xutils = require 'xamlamoveit.xutils'
local poseClass = require 'xamlamoveit.datatypes.Pose'

--[[
    This example works with meca500
    for other setup settings adjustments are necessary
    look for 'Adapt according to your setup'
]]
local function readKeySpinning()
    local function spin()
        if not ros.ok() then
            return false, 'ros shutdown requested'
        else
            ros.spinOnce()
            return true
        end
    end
    return xutils.waitKey(spin)
end

--Init ros node
ros.init('MoveInSteps')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()

--Create you dedicated motion service
local motionLibrary = require 'xamlamoveit.motionLibrary'
local mc = motionLibrary.MotionService(nh)

--Query necessary information about setup
local move_group_names, move_group_details = mc:queryAvailableMoveGroups()

--Select one moveit move group
local move_group = move_group_names[1]
--Define Xamla Movegroup
local xamla_mg = motionLibrary.MoveGroup(mc, move_group) -- motion client
local xamla_ee = xamla_mg:getEndEffector()

--Specify target relative to 'end_effector_link_name'
local target = poseClass.new()
target:setFrame(xamla_ee.link_name)
target:setTranslation(torch.Tensor {0.02, 0.02, 0.02}) -- meter

local velocity_scaling = 1
local acceleration_scaling = 1
local check_for_collisions = true
local do_interaction = true

-- prepare done callback
local result_state = 1
local result_payload = ''
local function done_cb(state, result)
    print('done.')
    do_interaction = false
    result_state = state
    result_payload = result
end

--Start motion
local handle = xamla_ee:movePoseLinearSupervised(target, velocity_scaling, check_for_collisions, acceleration_scaling, done_cb)
assert(torch.isTypeOf(handle, motionLibrary.SteppedMotionClient))

xutils.enableRawTerminal()
local input
while ros.ok() and do_interaction do
    print('Step through planned trajectory:')
    print('=====')
    print("'+'             next position")
    print("'-'             previous position")
    print("'ESC' or 'q'    quit")
    print()
    input = readKeySpinning()
    if input == '+' then
        handle:next()
    elseif input == '-' then
        handle:previous()
    elseif string.byte(input) == 27 or input == 'q' then
        do_interaction = false
        handle:abort()
    elseif input == 'f' then
        print('feedback: ', handle:getFeedback())
    end
end
xutils.restoreTerminalAttributes()

local ok, msg = handle:getResult()
print(msg)
handle:shutdown()
-- shutdown ROS
sp:stop()

ros.shutdown()
