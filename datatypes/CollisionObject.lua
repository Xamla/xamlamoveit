--[[
CollisionObject.lua

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

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
