sim = require 'sim'

function sysCall_actuation()
    local cam    = sim.getObject('.')
    local drone  = sim.getObject('/Quadcopter/base')
    
    local target = sim.getObject('/target',             {noError=true})
    if target == -1 then target = sim.getObject('/Quadcopter/target', {noError=true}) end

    if target == -1 then return end

    local dp = sim.getObjectPosition(drone,  -1)
    local tp = sim.getObjectPosition(target, -1)

    local dx = tp[1] - dp[1]
    local dy = tp[2] - dp[2]
    local dz = tp[3] - dp[3]

    local yaw     = math.atan2(dy, dx)
    local dist_xy = math.sqrt(dx*dx + dy*dy)
    local pitch   = -math.atan2(dz, dist_xy)

    sim.setObjectOrientation(cam, {0, pitch, yaw}, -1)
end
