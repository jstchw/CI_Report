-- Switch for velocity multipliers
switch = {
    [1] = function()
        leftVelMultiplier=0.55
        rightVelMultiplier=0.9
    end,
    [2] = function()
        leftVelMultiplier=0.9
        rightVelMultiplier=0.55
    end,

    [3] = function()
        leftVelMultiplier=0.99
        rightVelMultiplier=1
    end,
    [4] = function()
        leftVelMultiplier=1.5
        rightVelMultiplier=1.5
    end,
    [5] = function()
        leftVelMultiplier=-0.5
        rightVelMultiplier=0.5
        -- Reset it
        gen=0
    end,
    [6] = function()
        leftVelMultiplier=2
        rightVelMultiplier=0.03
    end
}

--Initializing the system
function sysCall_init()
    -- Creating an array of sensors (all -1 for now)
    usensors={ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 }
    -- Populating the sensor array
    for i = 1, 16, 1 do
        usensors[i] = sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    -- Assigning values to variables
    motorLeft = sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight = sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    -- No detection distance is set to 0.5 as a standard of Breitenberg
    noDetectionDist = 0.5
    -- Max detection distance is set to 0.2 as a standard of Breitenberg
    maxDetectionDist = 0.2
    -- Detection array is empty for now
    detect = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }
    -- Breitenberg arrays
    braitenbergL={ -0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 }
    braitenbergR={ -1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 }
    -- Velocity
    v0 = 5
    -- Multipliers
    leftVelMultiplier = 1
    rightVelMultiplier = 1
    -- Distance arrays
    dist = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }
    res = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }
    gen = 0

    func = switch[state]
end

function sysCall_cleanup()

end

function sysCall_actuation()
    -- Read only necessary sensors
    for i=5,9,1 do
        res[i],dist[i]=sim.readProximitySensor(usensors[i])
        if dist[i]==nil then
            dist[i]=1
        end
    end

    -- Multiplying the velocity by its multiplier
    vLeft=v0*leftVelMultiplier
    vRight=v0*rightVelMultiplier

    -- Maze algorithm
    -- Basically when certain distance to an object is achieved,
    -- we change the multiplier of the speed and the robot turns.
    -- Not perfect multipliers, might tweak them later.

    --1
    if dist[7]<0.32 then
        state = 1
    end

    --2
    if dist[7]>0.42 and dist[7]<1 or res[7]==0 then
        state = 2
    end

    --3
    if dist[7]>0.32 and dist[7]<0.42 then
        state = 3
    end

    --4
    if res[6]==1 and res[7]==1 and dist[6]>0.35 and dist[7]>0.35 and gen>20 then
        state = 4
    end

    --5
    if dist[5]<0.6 and res[6]==1 and res[7]==1 and res[8]==1 then
        state = 5
    end

    --6
    if res[8]==0 then
        state = 6
    end

    -- Every iteration adds one more, see functions above (one checks for it, the other resets it)
    gen=gen+1

    print("Gen: " .. gen)

    func = switch[state]

    if(func) then
        func()
    else
        print("Something's wrong")
    end

    -- Setting the velocities of the motors
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
end