

local datatypes = require 'xamlamoveit.components.datatypes.env'

local function switchKeyValue(t)
  local result = {}
  for k,v in pairs(t) do
      result[v] = k
  end
  return result
end

local JointSet = torch.class('xamlamoveit.components.datatypes.JointSet',datatypes)
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
  if #self.joint_names <1 then
    error("At least one Joint name needs to be specified.")
  end
  self.index_joint_names = switchKeyValue(self.joint_names)
end

function JointSet:addPrefix(prefix)
  assert(torch.type(prefix) == 'string')
  for i, v in self.joint_names do
    self.joint_names[i] = prefix + v
  end
end

function JointSet:isSubset(other)
  assert(torch.type(other) == 'JointSet')
  return table.isSubset(self.joint_names, other.joint_names)
end

function JointSet:isSimilar(other)
  assert(torch.type(other) == 'JointSet')
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


function JointSet:clone()
  return JointSet.new(self.joint_names)
end

function JointSet:__tostring()
  local res = 'JointSet: '
    for i, v in self.joint_names do
      res = string.format("%s %s", res, v)
    end
  return res
end

return JointSet
