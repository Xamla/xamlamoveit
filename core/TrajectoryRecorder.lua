local ros = require "ros"
local gnuplot = require "gnuplot"
require "xamlamoveit.xutils.Xtable"
local JointValues = require "xamlamoveit.datatypes.JointValues"
local JointSet = require "xamlamoveit.datatypes.JointSet"
local core = require "xamlamoveit.core.env"

local function plot(self, result, filename)
    local dim = result:size(2) - 1
    local samples = result:size(1)
    local positions = torch.zeros(samples, dim)
    local time = {}
    for i = 1, samples do
        time[i] = result[{i, 1}]
        positions[{i, {}}]:copy(result[{i, {2, dim + 1}}])
    end

    local t = torch.Tensor(time)

    gnuplot.pngfigure(filename)
    gnuplot.raw("set terminal png size 1280,960")
    gnuplot.xlabel("time")
    gnuplot.ylabel("pos")
    gnuplot.grid(true)

    local g = {}
    local names = self.joint_set:getNames()
    for i = 1, dim do
        local p = positions[{{}, i}]
        g[#g + 1] = {names[i], t, p, "+-"}
    end

    gnuplot.plot(table.unpack(g))
    gnuplot.plotflush()
end

local TrajectoryRecorder = torch.class("xamlamoveit.core.TrajectoryRecorder", core)

function TrajectoryRecorder:__init(joint_names)
    self.joint_set = JointSet(joint_names)
    self.samples = {}
    self.timestampes = {}
    self.start_recording = ros.Time.now()
end

function TrajectoryRecorder:start()
    ros.WARN("start recording")
    self.start_recording = ros.Time.now()
end

function TrajectoryRecorder:reset()
    ros.WARN("reset")
    self.samples = {}
    self.timestampes = {}
    self.start_recording = ros.Time.now()
end

function TrajectoryRecorder:add(sample)
    table.insert(self.timestampes, (ros.Time.now() - self.start_recording):toSec())
    table.insert(self.samples, sample:select(self.joint_set:getNames()):getValues())
end

function TrajectoryRecorder:remove(sample)
    local index = -1
    if torch.type(sample) == "number" then
        index = sample
    else
        index =
            table.findFirst(
            sample,
            function(x)
                return x == sample
            end
        )
    end
    if index > 0 then
        table.remove(self.samples, index)
    end
end

function TrajectoryRecorder:save(path, name)
    name = name or string.format("%s_%d", "TrajectoryRecorder", ros.Time.now():toSec())
    path = path or ""

    if #self.samples > 0 then
        local time = torch.Tensor(self.timestampes)
        local values = torch.cat(self.samples, 2):t()
        local data = torch.cat(time, values)
        local full_path = paths.concat(path, name .. ".t7")
        ros.INFO("save trajectory to: %s", full_path)
        torch.save(full_path, data)
        plot(self, data, name .. ".png")
    end
end

function TrajectoryRecorder.__len(a)
    print("#a.timestampes", #a.timestampes)
    return #a.timestampes
end

return core.TrajectoryRecorder
