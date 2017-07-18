local xutils = require 'xamlamoveit.xutils.env'
require 'xamlamoveit.xutils.Xtable'
require 'xamlamoveit.xutils.Kbhit'
require 'xamlamoveit.xutils.MonitorBuffer'
require 'xamlamoveit.xutils.JointMonitor'
require 'xamlamoveit.xutils.Cuboid'


local tictocStack = {}


function xutils.printf(...)
  print(string.format(...))
end

-- register printf in global namespace (not overwriting existing)
if not printf then
  printf = xutils.printf
end


function xutils.tic(id)
  if id then
    tictocStack[id]= sys.clock()
  else
    tictocStack[#tictoc_stack+1] = sys.clock()
  end
end


function xutils.toc(id)
  if id then
    if tictocStack[id] ~= nil then
      xutils.printf("Time elapsed (ID %s): %f sec", id, sys.clock() - tictocStack[id])
      tictocStack[id] = nil
    else
      xutils.printf('unknown ID')
    end
  else
    xutils.printf("Time elapsed (ID %d) : %f sec", #tictocStack, (sys.clock() - tictocStack[#tictocStack]))
    table.remove(tictocStack)
  end
end


function xutils.clamp(t, min, max)
    return torch.cmin(t, max):cmax(min)
end


-- register tic&toc in global namespace (not overwriting existing)
if not tic and not toc then
  tic,toc = xutils.tic,xutils.toc
end

if not printf then
  printf = xutils.printf
end


return xutils
