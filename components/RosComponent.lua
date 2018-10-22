--[[
RosComponent.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local circle_states = {IDLE = 0, CREATED = 1, STOPPED = 2, INITIALIZED = 3, EXECUTE = 4, FINISHED = 5, ERROR = -1}

local xamal_sysmon = require 'xamla_sysmon'
local components = require 'xamlamoveit.components.env'
local RosComponent = torch.class('xamlamoveit.components.RosComponent',components)

function RosComponent:__init()
    self.all_states = circle_states
    self.current_state = circle_states.IDLE
    self.state_info_server = nil
    self.state_callback_queue = nil
    self.subscriber = nil
    self:create()
    self:initialize()
end

function RosComponent:updateSystemState(msg, header)
    if msg.system_status >= 8 then -- Emerg_stop
        self:shutdown()
    elseif msg.system_status == 0 and self.current_state == circle_states.ERROR then
        self.current_state = circle_states.STOPPED
        self:reset()
        self:start()
    elseif msg.system_status >= 4 and self.current_state == circle_states.EXECUTE then
        self:stop()
        self.current_state = circle_states.ERROR
    end
end

local function switch(t)
    t.case = function(self, x)
        local f = self[x] or self.default
        if f then
            if type(f) == 'function' then
                f(x, self)
            else
                error('case ' .. tostring(x) .. ' not a function')
            end
        end
    end
    return t
end

function RosComponent:initialize()
    print('initialize is Called')
    if not (self.current_state == circle_states.CREATED or self.current_state == circle_states.STOPPED) then
        error('node is not in correct state is in: ' .. self.current_state)
    end

    self:onInitialize()
    self.current_state = circle_states.INITIALIZED
end

function RosComponent:create()
    print('create is Called')
    self.StateMachine =
        switch {
        [circle_states.IDLE] = function(x)
        end,
        [circle_states.STOPPED] = function(x)
        end,
        [circle_states.INITIALIZED] = function(x)
        end,
        [circle_states.CREATED] = function(x)
        end,
        [circle_states.FINISHED] = function(x)
        end,
        [circle_states.ERROR] = function(x)
            ros.ERROR('system error detected: %s', torch.type(self))
        end,
        [circle_states.EXECUTE] = function(x)
            self:process()
        end,
        default = function(x)
            print(string.format('state %d not known', x))
        end
    }
    self:onCreate()
    self.current_state = circle_states.CREATED
end

function RosComponent:onCreate()
    print('onCreate was called')
end

function RosComponent:onInitialize()
    print('onInitialize was called')
    self.current_state = circle_states.INITIALIZED
end

function RosComponent:start()
    print('start was called')
    if not (self.current_state == circle_states.INITIALIZED or self.current_state == circle_states.STOPPED) then
        error('start was called not in state STOPPED or INITIALIZED')
    end
    self:onStart()
    self.current_state = circle_states.EXECUTE
end

function RosComponent:onStart()
    print('onStart was called')
end

function RosComponent:process()
    if self.current_state ~= circle_states.EXECUTE then
        error('process was called not in state EXECUTE')
    end
    self:onProcess()
end

function RosComponent:onProcess()
    print('onProcess was called')
    --IMPEMENT IN YOUR APPLICATION
end

function RosComponent:stop()
    print('stop was called')
    self:onStop()
    self.current_state = circle_states.STOPPED
end

function RosComponent:onReset()
    print('onReset was called')
end

function RosComponent:reset()
    if self.current_state == circle_states.EXECUTE then
        self:stop()
    end
    self:onReset()
    self:initialize()
end

function RosComponent:onStop()
    print('onStop was called')
end

function RosComponent:spin()
    self.StateMachine:case(self.current_state)
end

function RosComponent:onShutdown()
end

function RosComponent:shutdown()
    print('shutdown was called')
    if self.current_state ~= circle_states.FINISHED then
        if self.current_state == circle_states.EXECUTE then
            self:stop()
        end
        self:onShutdown()
        self.current_state = circle_states.FINISHED
    end
end

return RosComponent
