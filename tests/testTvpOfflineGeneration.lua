local xamlamoveit = require 'xamlamoveit'
local data = torch.load("tests/example.t7")
local dt = 0.008
local dim = data[1]:size(1)
local controller = xamlamoveit.controller.TvpController(dim)
print(data[1], data[2])

local result = controller:generateOfflineTrajectory(data[1], data[2], dt)


local positions = torch.zeros(#result, dim)
local velocities = torch.zeros(#result, dim)
local accelerations = torch.zeros(#result, dim)
local time = {}
for i = 1, #result do
    time[i] = dt * i
    positions[{i, {}}]:copy(result[i].pos)
    velocities[{i, {}}]:copy(result[i].vel)
    accelerations[{i, {}}]:copy(result[i].acc)
end
time = torch.Tensor(time)

print(positions)