local ros = require 'ros'
local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local xamla_sysmon = require 'xamla_sysmon'
local xutils = xamlamoveit.xutils
require 'RosComponent'

local srv_spec = ros.SrvSpec('std_srvs/SetBool')

local function queryEmergencyStopServiceHandler(self, request, response, header)
    local msg = 'EMERG_STOP ...'
    if request.data then
        self.heartbeat:updateStatus(self.heartbeat.EMERG_STOP, msg)
    else
        msg = 'IDLE ...'
        self.heartbeat:updateStatus(self.heartbeat.IDLE, msg)
    end
    response.success = true
    response.message = msg
    return true
end

local EmergencyStopService, parent = torch.class('EmergencyStopService', 'RosComponent')

function EmergencyStopService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.info_server = nil
    self.heartbeat = xamla_sysmon.Heartbeat.new()
    parent.__init(self, node_handle)
end

function EmergencyStopService:onInitialize()
    self.heartbeat:start(self.node_handle, 0.5) --[Hz]
    self.heartbeat:updateStatus(self.heartbeat.IDLE, 'IDLE ...')
end

function EmergencyStopService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_emergency_stop',
        srv_spec,
        function(request, response, header)
            return queryEmergencyStopServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function EmergencyStopService:onProcess()
    self.heartbeat:publish()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming EmergencyStopService call')
        self.callback_queue:callAvailable()
    end
end

function EmergencyStopService:onStop()
    self.info_server:shutdown()
end


return EmergencyStopService
