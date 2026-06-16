sim = require 'sim'
simOMPL = require 'simOMPL'

local function getPose(dummy)
    local p = sim.getObjectPosition(dummy)
    return {p[1], p[2], p[3], 0, 0, 0, 1}
end

local function toXYZ(path)
    local xyz = {}
    for i = 1, #path, 7 do
        xyz[#xyz+1] = path[i]
        xyz[#xyz+1] = path[i+1]
        xyz[#xyz+1] = path[i+2]
    end
    return xyz
end

local function drawPath(xyz)
    if not line then
        line = sim.addDrawingObject(sim.drawing_lines, 3, 0, -1, 99999, {0,1,1})
    end

    sim.addDrawingObjectItem(line, nil)

    for i = 1, #xyz-3, 3 do
        sim.addDrawingObjectItem(line, {
            xyz[i], xyz[i+1], xyz[i+2],
            xyz[i+3], xyz[i+4], xyz[i+5]
        })
    end
end

function sysCall_init()
    local pathObj = sim.getObject('/Path')
    sim.writeCustomDataBlock(pathObj, 'OMPL_XYZ_PATH', '')
    sim.writeCustomDataBlock(pathObj, 'OMPL_READY', sim.packInt32Table({0}))
end

function sysCall_thread()
    local pathObj = sim.getObject('/Path')
    local robot   = sim.getObject('/Path/omplRobot')

    -- kumpulkan semua checkpoints
    local checkpoints = {}
    local startNode = sim.getObject('/Path/start', {noError=true})
    if startNode ~= -1 then
        table.insert(checkpoints, startNode)
    else
        error("Node 'start' tidak ditemukan!")
    end

    local i = 1
    while true do
        local goalNode = sim.getObject('/Path/goal' .. i, {noError=true})
        if goalNode ~= -1 then
            table.insert(checkpoints, goalNode)
            i = i + 1
        else
            break
        end
    end

    if #checkpoints == 1 then
        local singleGoal = sim.getObject('/Path/goal', {noError=true})
        if singleGoal ~= -1 then table.insert(checkpoints, singleGoal) end
    end

    if #checkpoints < 2 then
        sim.addLog(sim.verbosity_errors, 'Butuh minimal start dan 1 goal!')
        return
    end

    local full_xyz = {}
    local low  = {-14, -14, 0.25}
    local high = { 14,  14, 3}

    local marginScale = 1.75
    local isShape = (sim.getObjectType(robot) == sim.sceneobject_shape)

    if isShape then
        sim.scaleObject(robot, marginScale, marginScale, marginScale)
    end

    -- hitung path per segmen
    for idx = 1, #checkpoints - 1 do
        local pStart = checkpoints[idx]
        local pGoal  = checkpoints[idx+1]

        local task = simOMPL.createTask('task_' .. idx)
        local ss = {
            simOMPL.createStateSpace('s', simOMPL.StateSpaceType.pose3d, robot, low, high, 1)
        }

        simOMPL.setStateSpace(task, ss)
        simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRTConnect)
        simOMPL.setCollisionPairs(task, {robot, sim.handle_all})
        simOMPL.setStateValidityCheckingResolution(task, 0.002)

        simOMPL.setStartState(task, getPose(pStart))
        simOMPL.setGoalState(task,  getPose(pGoal))

        simOMPL.setup(task)

        local solved, path = simOMPL.compute(task, 5, 0, 300)

        if not path then
            sim.addLog(sim.verbosity_errors, 'Path tidak ditemukan pada segmen ' .. idx)
            simOMPL.destroyTask(task)
            if isShape then
                sim.scaleObject(robot, 1/marginScale, 1/marginScale, 1/marginScale)
            end
            return
        end

        local segment_xyz = toXYZ(path)
        for j = 1, #segment_xyz do
            table.insert(full_xyz, segment_xyz[j])
        end

        simOMPL.destroyTask(task)
    end

    -- kembalikan ukuran robot
    if isShape then
        sim.scaleObject(robot, 1/marginScale, 1/marginScale, 1/marginScale)
    end

    -- simpan path gabungan
    sim.writeCustomDataBlock(pathObj, 'OMPL_XYZ_PATH', sim.packFloatTable(full_xyz))

    local id = 0
    local old = sim.readCustomDataBlock(pathObj, 'OMPL_PATH_ID')
    if old and old ~= '' then
        id = sim.unpackInt32Table(old)[1]
    end

    sim.writeCustomDataBlock(pathObj, 'OMPL_PATH_ID', sim.packInt32Table({id+1}))
    sim.writeCustomDataBlock(pathObj, 'OMPL_READY', sim.packInt32Table({1}))

    drawPath(full_xyz)
end
