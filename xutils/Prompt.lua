local xutils = require 'xamlamoveit.xutils.env'
local ros = require 'ros'
local p = require 'posix'


local STDIN = 0


local Prompt = torch.class('xamlamoveit.xutils.Prompt', xutils)


local function defaultSpin()
  if not ros.ok() then
    return false, 'ros shutdown requested'
  else
    ros.spinOnce()
    return true
  end
end


function Prompt:__init(default_spin_fn)
  self.default_spin_fn = default_spin_fn or defaultSpin
  self.default_poll_timeout = 20
  self:saveTerminalAttributes()
end


function Prompt:saveTerminalAttributes()
  self.savedTerminalAttributes = p.tcgetattr(STDIN)
end


function Prompt:restoreTerminalAttributes()
  xutils.restoreTerminalAttributes(self.savedTerminalAttributes)
  self.rawTerminal = false
end


function Prompt:enableRawTerminal()
  xutils.enableRawTerminal()
  self.rawTerminal = true
end


function Prompt:printTitle(title)
  print(' ________________________')
  print('|')
  print('|  '..title)
  print('|________________________\n')
end


function Prompt:anyKey(message)
  message = message or 'press any key to continue...'
  print(message)
  self:waitKeySpinning()
end


-- returns true if enter was pressed and false if ESC was pressed.
function Prompt:waitEnterOrEsc()
  while true do
    local key = self:waitKeySpinning()
    if not key or key == string.char(3) or key == string.char(27) then
      return false
    elseif string.byte(key) == 13 then
      return true
    end
  end
end


function Prompt:readLine()
  if self.rawTerminal then
    self:restoreTerminalAttributes()
    local ln = io.read()
    self:enableRawTerminal()
    return ln
  else
    return io.read()
  end
end


function Prompt:readNumber()
  return tonumber(self:readLine())
end


function Prompt:showList(list, title)
  print(title)
  if list ~= nil and #list > 0 then
    for i, name in ipairs(list) do
      printf("%d: '%s'", i, name)
    end
  else
    print('None')
  end
end


function Prompt:chooseFromList(list, title)
  self:showList(list, title)
  print('Select index (hit return to keep existing choice):')
  local index = self:readNumber()
  if index == nil or index < 1 or index > #list or not list[index] then
    print('Invalid index, nothing changed.')
    return nil
  end

  return list[index]
end


function Prompt:kbhit()
  return xutils.kbhit()
end


function Prompt:waitKeySpinning(custom_spin_fn, poll_timeout_ms)
  local spin_fn = custom_spin_fn or self.default_spin_fn
  local poll_timeout = poll_timeout_ms or self.default_poll_timeout
  return xutils.waitKey(spin_fn, poll_timeout)
end


function Prompt:printXamlaBanner()
  print([[
    _  __                __
   | |/ /___ _____ ___  / /___ _
   |   / __ `/ __ `__ \/ / __ `/
  /   / /_/ / / / / / / / /_/ /
 /_/|_\__,_/_/ /_/ /_/_/\__,_/]])
end


function Prompt:showMenu(title, menu_options, custom_spin_fn, custom_info_fn)
  while true do
    self:printTitle(title)
    if custom_info_fn ~= nil then
      custom_info_fn()
    end

    local entries
    if type(menu_options) == 'table' then
      entries = menu_options
    elseif type(menu_options) == 'function' then
      entries = menu_options()
      assert(type(entries) == 'table', 'Menu entry generator did not return a table.')
    else
      error("Invalid argument: 'menu_options'.")
    end

    local key_fn_map = {}
    for i,o in ipairs(entries) do
      print(o[1] .. ' ' .. o[2])
      if #o[1] == 1 then
        key_fn_map[o[1]] = o[3]
      end
    end

    local key, err = self:waitKeySpinning(custom_spin_fn)
    if not key or key == string.char(3) or key == string.char(27) then
      return
    else
      local fn = key_fn_map[key]
      if fn == false then
        return
      elseif fn ~= nil then
        local result = fn()
        if result == false then
          return
        end
      end
    end
  end
end
