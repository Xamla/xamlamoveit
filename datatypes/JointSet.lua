--[[
JointSet.lua

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

local JointSet = torch.class('xamlamoveit.datatypes.JointSet', datatypes)
function JointSet:__init(names)
  self.joint_names = {}

  if torch.type(names) == 'table' then
    for i,v in ipairs(names) do
      if torch.type(v) == 'string' then
        table.insert(self.joint_names,v)
      end
    end
  elseif torch.type(names) == 'string' then
    table.insert(self.joint_names,names)
  end

  self.index_joint_names = {}
  if #self.joint_names > 0 then
    self.index_joint_names = table.swapKeyValue(self.joint_names)
  end
end

function JointSet:add(name)
  assert(torch.type(name) == 'string')
  if self:contains(name) then
    return
  end
  local index = #self.joint_names + 1
  self.joint_names[index] = name
  self.index_joint_names[name] = index
end

function JointSet:addPrefix(prefix)
  assert(torch.type(prefix) == 'string')
  for i, v in self.joint_names do
    self.joint_names[i] = prefix + v
  end
end

function JointSet:isSubset(other)
  assert(torch.isTypeOf(other, datatypes.JointSet))
  return table.isSubset(self.joint_names, other.joint_names)
end

function JointSet:isSimilar(other)
  assert(torch.isTypeOf(other, datatypes.JointSet))
  return table.isSimilar(self.joint_names, other.joint_names)
end

function JointSet:getIndexOf(name)
  assert(torch.type(name) == 'string')
  return self.index_joint_names[name]
end

function JointSet:tryGetIndexOf(name)
  assert(torch.type(name) == 'string')
  local index = self:getIndexOf(name)
  if index then
    return true, index
  else
    return false, index
  end
end

function JointSet:contains(name)
  local suc, index = self:tryGetIndexOf(name)
  return suc
end

function JointSet:count()
  return #self.joint_names
end

function JointSet:getNames()
  local result = {}
  for i,v in ipairs(self.joint_names) do
    result[i] = v
  end
  return result
end

function JointSet:clone()
  return JointSet.new(self.joint_names)
end

function JointSet:union(other)
  for i,v in ipairs(other:getNames()) do
    self:add(v)
  end
end

function JointSet:__tostring()
  local res = 'JointSet: '
    for i, v in ipairs(self.joint_names) do
      res = string.format("%s\n %s", res, v)
    end
  return res
end

function JointSet.__eq(a, b)
  assert(torch.isTypeOf(a, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(a)))
  assert(torch.isTypeOf(b, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(b)))
  if #b.joint_names ~= #a.joint_names then
    return false
  end
  for i, v in ipairs(a.joint_names) do
     if b.joint_names[i] ~= v then
        return false
     end
  end
  return true
end

function JointSet.__lt(a, b)
  assert(torch.isTypeOf(a, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(a)))
  assert(torch.isTypeOf(b, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(b)))
  return not a:isSimilar(b) and a:isSubset(b)
end

function JointSet.__add(a, b)
  assert(torch.isTypeOf(a, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(a)))
  assert(torch.isTypeOf(b, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(b)))
  local result = a:clone()
  result:union(b)
  return result
end

function JointSet.__len(a)
  assert(torch.isTypeOf(a, datatypes.JointSet), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointSet] but has [%s]', torch.type(a)))
  return a:count()
end

function JointSet.__next(t, k)
  return next(t.joint_names, k)
end


return datatypes.JointSet
