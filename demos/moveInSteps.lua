--[[
moveInSteps.lua

Copyright (C) 2018  Xamla info@xamla.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]
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
local motionLibrary = require 'xamlamoveit.motionLibrary'
local mc = motionLibrary.MotionService(nh)

--Query necessary information about setup
local move_group_names, move_group_details = mc:queryAvailableMoveGroups()

--Select one moveit move group
local move_group = move_group_names[1]
--Define Xamla Movegroup
local xamla_mg = motionLibrary.MoveGroup(mc, move_group) -- motion client
local end_effector = xamla_mg:getEndEffector()
local end_effector_name = end_effector.name
local end_effector_link_name = end_effector.link_name

print(end_effector_name, end_effector_link_name)
--Specify target relative to 'end_effector_link_name'
local target = tf.StampedTransform()
target:set_frame_id(end_effector_link_name)
target:setOrigin(torch.Tensor {0.02, 0.0, 0.01}) -- meter

local velocity_scaling = 1
local check_for_collisions = true
local do_interaction = true
local result_state = 1
local result_payload = ''
local function done_cb(state, result)
    print('done.')
    do_interaction = false
    result_state = state
    result_payload = result
end

--Start motion
local handle = xamla_mg:moveLSupervised(end_effector_name, target, velocity_scaling, check_for_collisions, done_cb)
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
