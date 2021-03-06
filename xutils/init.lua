local xutils = require 'xamlamoveit.xutils.env'
require 'xamlamoveit.xutils.Xtable'
require 'xamlamoveit.xutils.Kbhit'
require 'xamlamoveit.xutils.Cuboid'
require 'xamlamoveit.xutils.Prompt'


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
  local t
  if id then
    if tictocStack[id] ~= nil then
      t = sys.clock() - tictocStack[id]
      xutils.printf("Time elapsed (ID %s): %f sec", id, t)
      tictocStack[id] = nil
    else
      xutils.printf('unknown ID')
    end
  else
    t = sys.clock() - tictocStack[#tictocStack]
    xutils.printf("Time elapsed (ID %d) : %f sec", #tictocStack, (sys.clock() - tictocStack[#tictocStack]))
    table.remove(tictocStack)
  end
  return t
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


function xutils.parseRosParametersFromCommandLine(args,cmd)
  local cmd = cmd or  torch.CmdLine()
  local result = {}
  local residual = {}
  for i, v in ipairs(args) do
      if i > 0 then
          local tmp = string.split(v, ':=')
          if #tmp > 1 then
              result[tmp[1]] = tmp[2]
          else
              residual[i] = v
          end
      end
  end
  return table.merge(result,cmd:parse(residual))
end

return xutils
