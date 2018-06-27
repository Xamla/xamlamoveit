--[[
CollisionPrimitive.lua

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

local CollisionPrimitiveOperation = {
    Add = 0,
    Remove = 1,
    Append = 2,
    Move = 3
}

local CollisionPrimitiveKind = {
    Plane = 0,
    Box = 1,
    Sphere = 2,
    Cylinder = 3,
    Cone = 4
}

CollisionPrimitiveKind =
    table.merge(CollisionPrimitiveKind, table.swapKeyValue(CollisionPrimitiveKind))

local ExpectedParameterCount = {
    Plane = 4,
    Box = 3,
    Sphere = 1,
    Cylinder = 2,
    Cone = 2
}

local CollisionPrimitive = torch.class('xamlamoveit.datatypes.CollisionPrimitive', datatypes)
function CollisionPrimitive:__init(kind, parameters, pose)
    self.kind = kind or CollisionPrimitiveKind.Box
    self.pose = pose or datatypes.Pose()
    assert(
        torch.isTypeOf(parameters, torch.DoubleTensor),
        'Should be xamlamoveit.datatypes.Pose but is:' .. torch.type(point)
    )
    local info = CollisionPrimitiveKind[self.kind]
    if info == nil then
        error('Unknown collision primitve kind value')
    end
    local count = ExpectedParameterCount[info]
    assert(
        parameters:size(1) == count,
        string.format(
            "Expected %d parameters for collision primitive '%s' but %d were provided.",
            count,
            info,
            parameters:size(1)
        )
    )
    if kind == CollisionPrimitiveKind.Plane then
        assert(parameters[{{1, 3}}]:eq(0):sum() > 0, 'Invalid normal vector for plane specified.')
    else
        assert(
            parameters:lt(0):sum() == 0,
            string.format("Parameter for collision primitive '%s' must not be negative.", info)
        )
    end
    self.parameters = parameters
end

function CollisionPrimitive:clone()
    return CollisionPrimitive.new(self.kind, self.parameters:clone(), self.pose:clone())
end

function CollisionPrimitive:getPose()
    return self.pose:clone()
end

function CollisionPrimitive:getParameters()
    return self.parameters:clone()
end

function CollisionPrimitive:getKind()
    return self.kind, CollisionPrimitiveKind[self.kind]
end

function CollisionPrimitive:WithPose(value)
    return datatypes.CollisionPrimitive.new(self.kind, self.parameters, value)
end

function CollisionPrimitive.CreatePlane(a, b, c, d, pose)
    return datatypes.CollisionPrimitive(CollisionPrimitiveKind.Plane, torch.DoubleTensor {a, b, c, d}, pose)
end

function CollisionPrimitive.CreateBox(x, y, z, pose)
    return datatypes.CollisionPrimitive(CollisionPrimitiveKind.Box, torch.DoubleTensor {x, y, z}, pose)
end

function CollisionPrimitive.CreateSphere(radius, pose)
    return datatypes.CollisionPrimitive(CollisionPrimitiveKind.Sphere, torch.DoubleTensor {radius}, pose)
end

function CollisionPrimitive.CreateCylinder(height, radius, pose)
    return datatypes.CollisionPrimitive(
        CollisionPrimitiveKind.Cylinder,
        torch.DoubleTensor {height, radius},
        pose
    )
end

function CollisionPrimitive.CreateCone(height, radius, pose)
    return datatypes.CollisionPrimitive(
        CollisionPrimitiveKind.Cone,
        torch.DoubleTensor {height, radius},
        pose
    )
end

CollisionPrimitive.UnitSphere = CollisionPrimitive.CreateSphere(1.0, datatypes.Pose())
CollisionPrimitive.UnitBox = CollisionPrimitive.CreateBox(1.0, 1.0, 1.0, datatypes.Pose())
CollisionPrimitive.CollisionPrimitiveKind = CollisionPrimitiveKind
CollisionPrimitive.CollisionPrimitiveOperation = CollisionPrimitiveOperation

return CollisionPrimitive
