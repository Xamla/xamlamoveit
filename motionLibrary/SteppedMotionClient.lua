local ros = require 'ros'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local xutils = require 'xamlamoveit.xutils'
local motionLibrary = require 'xamlamoveit.motionLibrary.env'
local goal_id_spec = ros.MsgSpec('actionlib_msgs/GoalID')
local feedback_spec = ros.MsgSpec('xamlamoveit_msgs/TrajectoryProgress')

local error_codes = require 'xamlamoveit.core.ErrorCodes'.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))


local SteppedMotionClient = torch.class('SteppedMotionClient', motionLibrary)

function SteppedMotionClient:__init(action_client)
    assert(torch.isTypeOf(action_client, actionlib.SimpleActionClient), string.format('action_client hat type %s', torch.type(action_client)))
    self.action_client = action_client
    self.node_handle = self.action_client.nh

    self.last_progress = 0
    self.last_error_msg = 'OK'
    self.canceled = false
    self.subscriber = nil

    self.goal_id_msg = ros.Message(goal_id_spec)
    self.goal_id_msg.id = action_client.gh.id

    self.stepping_next_topic = '/xamlaMoveActions/next'
    self.stepping_prev_topic = '/xamlaMoveActions/prev'
    self.stepping_feedback_topic = '/xamlaMoveActions/feedback'

    self.publisherNext = self.node_handle:advertise(self.stepping_next_topic, goal_id_spec)
    self.publisherPrev = self.node_handle:advertise(self.stepping_prev_topic, goal_id_spec)

    local feedback_callback = function(msg, header)
        self.last_progress = msg.progress
        self.last_error_msg = msg.error_msg
    end
    self.subscriber = self.node_handle:subscribe(self.stepping_feedback_topic, feedback_spec)
    self.subscriber:registerCallback(feedback_callback)
end

function SteppedMotionClient:next()
    self.publisherNext:publish(self.goal_id_msg)
end

function SteppedMotionClient:previous()
    self.publisherPrev:publish(self.goal_id_msg)
end

function SteppedMotionClient:abort()
    self.action_client:cancelGoal()
    self.canceled = true
end

function SteppedMotionClient:getFeedback()
    return self.last_progress, self.last_error_msg
end

function SteppedMotionClient:getResult()
    local action_client = self.action_client
    local ok = action_client:waitForResult()
    local state, state_msg = action_client:getState()
    local answere = action_client:getResult()
    ros.INFO('moveJ_action returned: %s, [%d]', state_msg, state)
    if answere == nil then
        answere = {result = state}
    end
    if ok and state == SimpleClientGoalState.SUCCEEDED or self.canceled then
        return true, state_msg
    else
        return false, string.format('%s, Result: %s (%d)', state_msg, error_codes[answere.result], answere.result)
    end
end

function SteppedMotionClient:getActionClient()
    return self.action_client
end

function SteppedMotionClient:shutdown()
    self.publisherNext:shutdown()
    self.publisherPrev:shutdown()
    if self.subscriber then
        self.subscriber:shutdown()
    end
end

return SteppedMotionClient
