-- simple buffer
local xutils = require 'xamlamoveit.xutils.env'
local monitorBuffer = torch.class('xamlamoveit.xutils.MonitorBuffer',xutils)


function monitorBuffer:__init(windowSize, vectorDim)
  self.DIM = vectorDim
  self.WINDOW_SIZE = windowSize
  self.write_pos = 0

  self.buffer = torch.zeros(self.WINDOW_SIZE,self.DIM)
end


function monitorBuffer:count()
  if self.WINDOW_SIZE<self.write_pos then
    return self.WINDOW_SIZE
  else
    return self.write_pos
  end
end


function monitorBuffer:avail()
  return self.WINDOW_SIZE - self:count()
end


function monitorBuffer:add(l)
  local next_pos = self.write_pos % self.WINDOW_SIZE +1
  self.buffer[{next_pos,{}}]:copy(l)
  self.write_pos = next_pos
end


function monitorBuffer:clear()
  self.buffer:zeros()
  self.write_pos = 0
end


return monitorBuffer
