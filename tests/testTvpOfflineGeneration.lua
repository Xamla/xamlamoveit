local xamlamoveit = require 'xamlamoveit'
local gnuplot = require 'gnuplot'
local dt

local function generateRandomVector(size, min, max)
    return torch.rand(size) * (max-min) + min
end


local function plotVelocities(result, dt, dim, filename)
    local dim = result[1].pos:size(1)
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

    local t = torch.Tensor(time)

    gnuplot.pngfigure(filename)
    gnuplot.raw("set terminal png size 1280,960")
    gnuplot.xlabel('time')
    gnuplot.ylabel('pos')
    gnuplot.grid(true)


    local g = {}
    for i=1,dim do
        local v = velocities[{{},i}]
        g[#g+1] = { string.format('vel%d', i), t, v, '+-'}
    end

    gnuplot.plot(
        table.unpack(g)
    )
    gnuplot.plotflush()
end

local function runRandomTest(controllerName, dim, runs, dt, plot, plot_stride, goal_diff_scale)
    local controller = xamlamoveit.controller[controllerName](dim)
    for i=1,runs do
        local max_v = generateRandomVector(dim, 1, 15)
        local max_a = generateRandomVector(dim, 0.1, 20)
        local start = generateRandomVector(dim, -3 * math.pi, 3 * math.pi)
        local goal = start + generateRandomVector(dim, -3 * math.pi, 3 * math.pi) * goal_diff_scale

        --[[
        print('max_v')
        print(max_v)
        print('max_a')
        print(max_a)
        print('start')
        print(start)
        print('goal')
        print(goal)
        ]]

        controller.max_vel:copy(max_v)
        controller.max_acc:copy(max_a)
        local result, delta = controller:generateOfflineTrajectory(start, goal, dt)
        printf('[%d] Number of points: %d; final delta: %f;', i, #result, delta:norm())

        if plot and (plot_stride == nil or i % plot_stride == 0) then
            plotVelocities(result, dt, dim, string.format("plot%d.png", i))
        end
    end
end

local cmd = torch.CmdLine()
cmd:option('-controllerName','TvpController','controller class name. Choose From [TvpController, MultiAxisTvpController]')
cmd:option('-plot', false, 'whether to generate plots')
cmd:option('-plot_stride', 1, 'genaret plots for each i%plot_stide == 0')
cmd:option('-dt', 0.008, 'dt')
cmd:option('-dim', 10, 'number of axis')
cmd:option('-n', 100, 'number of tests')
-- parse input params
local params = cmd:parse(arg)
if params.controllerName == 'TvpController' or params.controllerName == 'MultiAxisTvpController' then
    torch.manualSeed(0)
    local t0 = torch.tic()
    runRandomTest(params.controllerName, params.dim, params.n, params.dt, params.plot, params.plot_stride, 1)
    local elapsed = torch.toc(t0)
    printf('elapsed: %f', elapsed)
else
    print('Unknown controller name. Choose From [TvpController, MultiAxisTvpController]')
end
