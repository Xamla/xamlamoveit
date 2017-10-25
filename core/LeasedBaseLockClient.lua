-- simple buffer
local ros = require 'ros'
local core = require 'xamlamoveit.core.env'
local LeasedBaseLockClient = torch.class('xamlamoveit.core.LeasedBaseLockClient', core)

function LeasedBaseLockClient:__init(node_handle)
  self.nh = node_handle
  self.query_resource_lock_service =
  self.nh:serviceClient('/xamlaResourceLockService/query_resource_lock', 'xamlamoveit_msgs/QueryLock')
  local timeout = ros.Duration(5)
  local ok = self.query_resource_lock_service:waitForExistence(timeout)
  if not ok then
    ros.ERROR('could not reach lock service!')
end
end

local function query_lock(self, id_resources, id_lock, release_flag)
  ros.DEBUG(string.format("query_lock: id = %s, release = %s", id_lock, release_flag))
  local request = self.query_resource_lock_service:createRequest()
  request.release = release_flag or false
  request.id_resources = id_resources
  request.id_lock = id_lock or ''
  local responds = self.query_resource_lock_service:call(request)
  if responds.success then
      return responds.success, responds.id_lock, responds.creation_date, responds.expiration_date
  else
      return responds.success
  end
end

function LeasedBaseLockClient:lock(id_resources, id_lock)
  local suc, id_lock, creation_date, expiration_date = query_lock(self, id_resources, id_lock, false)
  local lock = {success = suc, id = id_lock, resources = id_resources, created = creation_date, expiration = expiration_date}
  return lock
end

function LeasedBaseLockClient:release(lock)
  assert(torch.type(lock) == 'table', "type should be 'table' but is: " .. torch.type(lock))
  assert(lock.id, "Lock should have an id but it is nil")
  return query_lock(self, lock.resources, lock.id, true)
end

function LeasedBaseLockClient:shutdown()
  self.query_resource_lock_service:shutdown()
end

return LeasedBaseLockClient
