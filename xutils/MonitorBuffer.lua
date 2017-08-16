-- simple buffer
local xutils = require 'xamlamoveit.xutils.env'
local MonitorBuffer = torch.class('xamlamoveit.xutils.MonitorBuffer',xutils)


function MonitorBuffer:__init(windowSize, vectorDim)
  self.DIM = vectorDim
  self.WINDOW_SIZE = windowSize
  self.write_pos = 0

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
  local offset = offset or 0
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
