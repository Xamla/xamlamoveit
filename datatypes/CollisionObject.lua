--[[
CollisionObject.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local datatypes = require 'xamlamoveit.datatypes.env'
local xutils = require 'xamlamoveit.xutils'

local CollisionObject = torch.class('xamlamoveit.datatypes.CollisionObject', datatypes)
function CollisionObject:__init(frame_id, primitives)
    self.primitves = {}
    for i, prim in ipairs(primitives) do
        assert(torch.isTypeOf(prim, datatypes.CollisionPrimitive))
        self.primitves[#self.primitves + 1] = prim:clone()
    end
    self.frame = frame_id or "world"
end

function CollisionObject:getFrame()
    return self.frame
end

function CollisionObject:getPrimitives()
    return self.primitves
end

return CollisionObject
