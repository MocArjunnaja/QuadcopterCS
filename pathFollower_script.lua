sim = require 'sim'

local function distance(p1, p2)
    local dx = p1[1] - p2[1]
    local dy = p1[2] - p2[2]
    local dz = p1[3] - p2[3]
    return math.sqrt(dx*dx + dy*dy + dz*dz)
end

local function getTarget()
    local t = sim.getObject('/target', {noError=true})
    if t ~= -1 then return t end

    t = sim.getObject('/Quadcopter_target', {noError=true})
    if t ~= -1 then return t end

    return sim.getObject('/Quadcopter/target', {noError=true})
end

local function getReady(pathHandle)
    local data = sim.readCustomDataBlock(pathHandle, 'OMPL_READY')
    if not data or data == '' then return nil end
    return sim.unpackInt32Table(data)[1]
end

function sysCall_thread()
    local target = getTarget()
    if target == -1 then
        error("Target drone tidak ditemukan")
    end
    local pathObj = sim.getObject('/Path')

    -- reset: tunggu OMPL_READY = 0
    while true do
        if getReady(pathObj) == 0 then break end
        sim.step()
    end

    -- tunggu path selesai di-generate
    local xyz
    while true do
        local ready   = getReady(pathObj)
        local rawPath = sim.readCustomDataBlock(pathObj, 'OMPL_XYZ_PATH')

        if ready == 1 and rawPath and rawPath ~= '' then
            xyz = sim.unpackFloatTable(rawPath)
            break
        end
        sim.step()
    end

    -- susun waypoints
    local waypoints = {}
    for i = 1, #xyz, 3 do
        waypoints[#waypoints+1] = {xyz[i], xyz[i+1], xyz[i+2]}
    end

    -- gerakkan target sepanjang waypoints
    local speed     = 0.3
    local yaw_speed = 1.5  -- rad/s, kecepatan putar maksimal
    local dt        = sim.getSimulationTimeStep()

    -- inisialisasi yaw dari arah segmen pertama
    local current_yaw = 0
    if #waypoints >= 2 then
        local dx = waypoints[2][1] - waypoints[1][1]
        local dy = waypoints[2][2] - waypoints[1][2]
        current_yaw = math.atan2(dy, dx)
    end

    for i = 1, #waypoints-1 do
        local p1 = waypoints[i]
        local p2 = waypoints[i+1]

        local d     = distance(p1, p2)
        local T     = d / speed
        local steps = math.max(1, math.floor(T / dt))

        local dx         = p2[1] - p1[1]
        local dy         = p2[2] - p1[2]
        local target_yaw = math.atan2(dy, dx)

        for j = 1, steps do
            local s = j / steps
            local pos = {
                p1[1]*(1-s) + p2[1]*s,
                p1[2]*(1-s) + p2[2]*s,
                p1[3]*(1-s) + p2[3]*s
            }

            -- smooth yaw: rotate current_yaw menuju target_yaw
            local dyaw = target_yaw - current_yaw
            -- wrap ke [-pi, pi] agar putar arah terpendek
            while dyaw >  math.pi do dyaw = dyaw - 2*math.pi end
            while dyaw < -math.pi do dyaw = dyaw + 2*math.pi end
            local max_step = yaw_speed * dt
            if math.abs(dyaw) <= max_step then
                current_yaw = target_yaw
            else
                current_yaw = current_yaw + (dyaw > 0 and max_step or -max_step)
            end

            sim.setObjectPosition(target, pos)
            sim.setObjectOrientation(target, {0, 0, current_yaw}, -1)
            sim.step()
        end
    end
end
