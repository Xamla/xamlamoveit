--[[
LeaseBasedLockClient.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local core = require 'xamlamoveit.core.env'
local LeaseBasedLockClient = torch.class('xamlamoveit.core.LeaseBasedLockClient', core)

function LeaseBasedLockClient:__init(node_handle)
    self.nh = node_handle
    self.query_resource_lock_service =
        self.nh:serviceClient('/xamlaResourceLockService/query_resource_lock', 'xamlamoveit_msgs/QueryLock', true)
    local timeout = ros.Duration(5)
    local ok = self.query_resource_lock_service:waitForExistence(timeout)
    if not ok then
        ros.ERROR('could not reach lock service!')
    end
end

local function query_lock(self, id_resources, id_lock, release_flag)
    ros.DEBUG(string.format('query_lock: id = %s, release = %s', id_lock, release_flag))
    local request = self.query_resource_lock_service:createRequest()
    request.release = release_flag or false
    request.id_resources = id_resources
    request.id_lock = id_lock or ''
    if self.query_resource_lock_service:isValid() then
        local responds = self.query_resource_lock_service:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.id_lock, responds.creation_date, responds.expiration_date
            else
                return responds.success
            end
        end
    end
    return false
end

function LeaseBasedLockClient:lock(id_resources, id_lock)
    local suc, id_lock, creation_date, expiration_date = query_lock(self, id_resources, id_lock, false)
    local lock = {
        success = suc,
        id = id_lock,
        resources = id_resources,
        created = creation_date,
        expiration = expiration_date
    }
    return lock
end

function LeaseBasedLockClient:release(lock)
    assert(torch.type(lock) == 'table', "type should be 'table' but is: " .. torch.type(lock))
    assert(lock.id, 'Lock should have an id but it is nil')
    return query_lock(self, lock.resources, lock.id, true)
end

function LeaseBasedLockClient:shutdown()
    self.query_resource_lock_service:shutdown()
end

return LeaseBasedLockClient
