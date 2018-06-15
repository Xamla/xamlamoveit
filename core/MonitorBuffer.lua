--[[
MonitorBuffer.lua
simple buffer

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

local core = require 'xamlamoveit.core.env'
local MonitorBuffer = torch.class('xamlamoveit.core.MonitorBuffer', core)


function MonitorBuffer:__init(windowSize, vectorDim)
  self.DIM = vectorDim
  self.WINDOW_SIZE = windowSize
  self.write_pos = 0
  self.offset = 0
  self.buffer = torch.zeros(self.WINDOW_SIZE,self.DIM)
end


function MonitorBuffer:count()
  if self.WINDOW_SIZE<self.write_pos then
    return self.WINDOW_SIZE
  else
    return self.write_pos
  end
end


function MonitorBuffer:getPastIndex(offset)
  local offset = offset or self.offset
  local tmp_index = self.WINDOW_SIZE + self.write_pos - offset
  local index = tmp_index % self.WINDOW_SIZE + 1
  return self.buffer[{index,{}}]:clone()
end


function MonitorBuffer:avail()
  return self.WINDOW_SIZE - self:count()
end


function MonitorBuffer:add(l)
  local next_pos = self.write_pos % self.WINDOW_SIZE +1
  self.buffer[{next_pos,{}}]:copy(l)
  self.write_pos = next_pos
end


function MonitorBuffer:clear()
  self.buffer:zeros()
  self.write_pos = 0
end


return MonitorBuffer
