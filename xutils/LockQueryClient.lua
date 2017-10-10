-- simple buffer
local xutils = require 'xamlamoveit.xutils.env'
local LeasedBaseLockClient = torch.class('xamlamoveit.xutils.LeasedBaseLockClient',xutils)

function LeasedBaseLockClient:__init(node_handle)
  self.nh = node_handle
  self.query_resource_lock_service =
  self.nh:serviceClient('xamlaservices/query_resource_lock', 'xamlamoveit_msgs/QueryLock')
end

local function query_lock(self, id_resources, id_lock, release_flag)
  ros.WARN("query_lock")
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
  local lock = {success = suc, id = id_lock, resource = id_resources, created = creation_date, expiration = expiration_date}
  return lock
end

function LeasedBaseLockClient:release(lock)
  assert(torch.type(lock) == 'table')
  return query_lock(self, lock.resources, lock.id, true)
end

return LeasedBaseLockClient
