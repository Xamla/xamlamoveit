local xutils = require 'xamlamoveit.xutils.env'
local p = require 'posix'
local signal = require 'posix.signal'
signal.signal(signal.SIGINT, function(signum)
  log("Caught CTRL+C, terminating...")
  active = false
end)

local STDIN = 0


local savedTerminalAttributes = p.tcgetattr(STDIN)
local fds = { [STDIN] = { events = {IN=true}, revents = {} } }



function xutils.enableRawTerminal()
  local term = p.tcgetattr(STDIN) -- start with current settings
  -- modifierd raw mode
  -- see http://linux.die.net/man/3/tcsetattr
  term.iflag = bit.band(term.iflag, bit.bnot(bit.bor(p.ISTRIP, p.INLCR, p.IGNCR, p.ICRNL, p.IXON)))
  term.cflag = bit.band(term.cflag, bit.bnot(p.OPOST))
  term.lflag = bit.band(term.lflag, bit.bnot(bit.bor(p.ECHO, p.ECHONL, p.ICANON, p.IEXTEN)))  -- p.ISIG removed to allow CTRL-C processing
  term.cflag = bit.band(term.cflag, bit.bnot(bit.bor(p.CSIZE, p.PARENB)))
  term.cflag = bit.bor(term.cflag, p.CS8)
  p.tcsetattr(STDIN, 0, term)
end


function xutils.saveTerminalAttributes()
  savedTerminalAttributes = p.tcgetattr(STDIN)
  return savedTerminalAttributes
end


function xutils.restoreTerminalAttributes(attributes)
  attributes = attributes or savedTerminalAttributes
  local ok, errmsg = p.tcsetattr(STDIN, 0, attributes)
  assert(ok, errmsg)
end


function xutils.kbhit(timeout)
  fds[STDIN].revents = {}
  p.poll(fds, timeout or 0)
  if fds[STDIN].revents.IN then
    return true
  else
    return false
  end
end


function xutils.readKey(timeout)
  if not xutils.kbhit() then
    return nil
  end
  local d, err = p.read(STDIN, 1)
  return d
end


function xutils.waitKey(spinFunc, poll_timeout)
  while true do
    if xutils.kbhit(poll_timeout) == true then
      local d, err = p.read(STDIN, 1)   -- Svn client version: 1.8
      if not d then
        return nil, err
      end
      return d
    end
    if spinFunc then
      local ok, msg = spinFunc()
      if not ok then
        return nil, msg
      end
    end
  end
end
