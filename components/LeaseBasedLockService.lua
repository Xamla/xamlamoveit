local ros = require 'ros'
local uuid = require 'uuid'

local database = {}
local expire_window = ros.Duration(5)

local function release(id_resources, id_lock)
    for i, id_resource in ipairs(id_resources) do
        local result = database[id_resource]
        if result then
            if result.id_lock == id_lock then
                ros.INFO('Released Lock: ' .. id_resource)
                database[id_resource] = nil
            end
        end
    end
    return true
end

local function lock(id_resources, id_lock)
    local id = id_lock or uuid()
    if id == '' then
        id = uuid()
    end
    ros.INFO('uuid: %s', tostring(id))
    local now = ros.Time.now()
    for i, id_resource in ipairs(id_resources) do
        local result = database[id_resource]

        if result then
            if result.id_lock ~= id or result.expiration_date < now then
                return false, {['id_lock'] = id, ['creation_date'] = now, ['expiration_date'] = now}
            end
            database[id_resource].id_lock = id
            database[id_resource].creation_date = now
            database[id_resource].expiration_date = now + expire_window
        else
            ros.INFO('create entry lock: ' .. id_resource)
            database[id_resource] = {
                ['id_lock'] = id,
                ['creation_date'] = now,
                ['expiration_date'] = now + expire_window
            }
        end
    end
    local entries = {
        ['id_resources'] = id_resources,
        ['id_lock'] = id,
        ['creation_date'] = now,
        ['expiration_date'] = now + expire_window
    }
    return true, entries
end

local function spin()
    local now = ros.Time.now()
    for k, v in pairs(database) do
        if v.expiration_date < now then
            ros.WARN('ResourceLock expired:\n\tResource name: %s , Lock id: %s', tostring(k), v.id_lock)
            database[k] = nil
        end
    end
end

local function queryLockResourceServiceHandler(request, response, header)
    ros.WARN('queryLockResourceServiceHandler')
    if request.release then
        ros.WARN('release resources')
        response.success = release(request.id_resources, request.id_lock)
        response.id_resources = request.id_resources
        if response.success then
            response.error_msg = 'Release was successful'
        else
            response.error_msg = 'Release was unsuccessful'
        end
    else
        local suc, entry = lock(request.id_resources, request.id_lock)
        response:fillFromTable(entry)
        response.success = suc
        response.id_resources = request.id_resources
        if response.success then
            response.error_msg = 'Lock was successfully aquired'
        else
            response.error_msg = 'Lock was not aquired'
            release(request.id_resources, entry.id_lock)
        end
    end
    return true
end

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/QueryLock')

local components = require 'xamlamoveit.components.env'
local LeaseBasedLockService,
    parent =
    torch.class('xamlamoveit.components.LeaseBasedLockService', 'xamlamoveit.components.RosComponent', components)

function LeaseBasedLockService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.info_server = nil
    parent.__init(self, node_handle)
end

function LeaseBasedLockService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_resource_lock',
        srv_spec,
        queryLockResourceServiceHandler,
        self.callback_queue
    )
end

function LeaseBasedLockService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming LeaseBasedLockService call')
        self.callback_queue:callAvailable()
    end
    spin()
end

function LeaseBasedLockService:onReset()
    self.info_server:shutdown()
    database = {}
end

function LeaseBasedLockService:onShutdown()
    self.info_server:shutdown()
end

return LeaseBasedLockService
