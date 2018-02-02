local xamlamoveit = require 'xamlamoveit'
local gnuplot = require 'gnuplot'


local function generateRandomVector(size, min, max)
    return torch.rand(size) * (max-min) + min
end


local function plotVelocities(result, filename)
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
    for i=1,7 do
        local v = velocities[{{},i}]
        g[#g+1] = { string.format('vel%d', i), t, v, '+-'}
    end
    
    gnuplot.plot(
        table.unpack(g)
    )
    gnuplot.plotflush()
end


local function runRandomTest(dim, runs, dt, plot)    
    local controller = xamlamoveit.controller.MultiAxisTvpController(dim)
    for i=1,runs do
        local max_v = generateRandomVector(dim, 1, 15)
        local max_a = generateRandomVector(dim, 0.1, 20)
        local start = generateRandomVector(dim, -3 * math.pi, 3 * math.pi)
        local goal = generateRandomVector(dim, -3 * math.pi, 3 * math.pi)

        --[[print('max_v')
        print(max_v)
        print('max_a')
        print(max_a)
        print('start')
        print(start)
        print('goal')
        print(goal)]]

        controller.max_vel:copy(max_v)
        controller.max_acc:copy(max_a)
        local result, delta = controller:generateOfflineTrajectory(start, goal, dt)
        printf('[%d] Number of points: %d; final delta: %f;', i, #result, delta:norm())

        if plot then
            plotVelocities(result, string.format("plot%d.png", i))
        end
    end
end


torch.manualSeed(0)
local t0 = torch.tic()
runRandomTest(10, 100, 0.008, false)
local elapsed = torch.toc(t0)
printf('elapsed: %f', elapsed)
