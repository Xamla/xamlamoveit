local ros = require 'ros'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local xutils = require 'xamlamoveit.xutils'
local error_codes = require 'xamlamoveit.core.ErrorCodes'.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))
local tf = ros.tf
local xutils = require 'xamlamoveit.xutils'

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
local mc = require 'xamlamoveit.motionLibrary'.MotionService(nh)

--Query necessary information about setup
local move_group_names, move_group_details = mc:queryAvailableMoveGroups()

--Select one moveit move group
local move_group = move_group_names[1]
local end_effector_name = 'EE_manipulator' --Adapt according to your setup
local end_effector_link_name = 'wrist_3_link' --Adapt according to your setup

--Define Xamla Movegroup
local xamla_mg = require 'xamlamoveit.motionLibrary'.MoveGroup(mc, move_group) -- motion client

--Specify target relative to 'end_effector_link_name'
local target = tf.StampedTransform()
target:set_frame_id(end_effector_link_name)
target:setOrigin(torch.Tensor {0.02, 0.0, 0.01}) -- meter

local velocity_scaling = 1
local check_for_collisions = true
local doInteraction = true
local result_state = 1
local result_payload = ''
local function done_cb(state, result)
    doInteraction = false
    result_state = state
    result_payload = result
end

--Start motion
local action_client, handle = xamla_mg:steppedMoveL(end_effector_name, target, velocity_scaling, check_for_collisions)

xutils.enableRawTerminal()
local input
local canceled = false
while ros.ok() and doInteraction do
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
        handle:prev()
    elseif string.byte(input) == 27 or input == 'q' then
        doInteraction = false
        canceled = true
        handle:abort()
    end
end
xutils.restoreTerminalAttributes()
handle:shutdown()

local ok = action_client:waitForResult()
local state, state_msg = action_client:getState()
local answere = action_client:getResult()
ros.INFO('moveJ_action returned: %s, [%d]', state_msg, state)
if answere == nil then
    answere = {result = state}
end
if ok and state == SimpleClientGoalState.SUCCEEDED or canceled then
    return true, state_msg
else
    return false, string.format('%s, Result: %s (%d)', state_msg, error_codes[answere.result], answere.result)
end
-- shutdown ROS
sp:stop()

ros.shutdown()
