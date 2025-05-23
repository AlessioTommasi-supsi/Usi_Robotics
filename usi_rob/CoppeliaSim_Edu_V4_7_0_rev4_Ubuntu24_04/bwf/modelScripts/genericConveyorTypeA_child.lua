simBWF=require('simBWF')
function getTriggerType()
    if stopTriggerSensor~=-1 then
        local data=sim.readCustomStringData(stopTriggerSensor,'XYZ_BINARYSENSOR_INFO')
        if data and #data > 0 then
            data=sim.unpackTable(data)
            local state=data['detectionState']
            if not lastStopTriggerState then
                lastStopTriggerState=state
            end
            if lastStopTriggerState~=state then
                lastStopTriggerState=state
                return -1 -- means stop
            end
        end
    end
    if startTriggerSensor~=-1 then
        local data=sim.readCustomStringData(startTriggerSensor,'XYZ_BINARYSENSOR_INFO')
        if data and #data > 0 then
            data=sim.unpackTable(data)
            local state=data['detectionState']
            if not lastStartTriggerState then
                lastStartTriggerState=state
            end
            if lastStartTriggerState~=state then
                lastStartTriggerState=state
                return 1 -- means restart
            end
        end
    end
    return 0
end

function overrideMasterMotionIfApplicable(override)
    if masterConveyor>=0 then
        local data=sim.readCustomStringData(masterConveyor,simBWF.modelTags.CONVEYOR)
        if data and #data > 0 then
            data=sim.unpackTable(data)
            local stopRequests=data['stopRequests']
            if override then
                stopRequests[model]=true
            else
                stopRequests[model]=nil
            end
            data['stopRequests']=stopRequests
            sim.writeCustomStringData(masterConveyor,simBWF.modelTags.CONVEYOR,sim.packTable(data))
        end
    end
end

function getMasterDeltaShiftIfApplicable()
    if masterConveyor>=0 then
        local data=sim.readCustomStringData(masterConveyor,simBWF.modelTags.CONVEYOR)
        if data and #data > 0 then
            data=sim.unpackTable(data)
            local totalShift=data['encoderDistance']
            local retVal=totalShift
            if previousMasterTotalShift then
                retVal=totalShift-previousMasterTotalShift
            end
            previousMasterTotalShift=totalShift
            return retVal
        end
    end
end

function sysCall_init()
    model=sim.getObject('..')
    local data=sim.readCustomStringData(model,simBWF.modelTags.CONVEYOR)
    data=sim.unpackTable(data)
    stopTriggerSensor=simBWF.getReferencedObjectHandle(model,1)
    startTriggerSensor=simBWF.getReferencedObjectHandle(model,2)
    masterConveyor=simBWF.getReferencedObjectHandle(model,3)
    getTriggerType()
    length=data['length']
    height=data['height']
    local err=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0) -- do not report errors
    textureB=sim.getObject('../genericConveyorTypeA_textureB')
    textureC=sim.getObject('../genericConveyorTypeA_textureC')
    jointB=sim.getObject('../genericConveyorTypeA_jointB')
    jointC=sim.getObject('../genericConveyorTypeA_jointC')
    sim.setInt32Param(sim.intparam_error_report_mode,err) -- report errors again
    textureA=sim.getObject('../genericConveyorTypeA_textureA')
    forwarderA=sim.getObject('../genericConveyorTypeA_forwarderA')
    lastT=sim.getSimulationTime()
    beltVelocity=0
    totShift=0
end 

function sysCall_actuation()
    local data=sim.readCustomStringData(model,simBWF.modelTags.CONVEYOR)
    data=sim.unpackTable(data)
    maxVel=data['velocity']
    accel=data['acceleration']
    enabled=(data['bitCoded']&64)>0
    if not enabled then
        maxVel=0
    end
    local stopRequests=data['stopRequests']
    local trigger=getTriggerType()
    if trigger>0 then
        stopRequests[model]=nil -- restart
    end
    if trigger<0 then
        stopRequests[model]=true -- stop
    end
    if next(stopRequests) then
        maxVel=0
        overrideMasterMotionIfApplicable(true)
    else
        overrideMasterMotionIfApplicable(false)
    end
    
    t=sim.getSimulationTime()
    dt=t-lastT
    lastT=t

    local masterDeltaShift=getMasterDeltaShiftIfApplicable()
    if masterDeltaShift then
        totShift=totShift+masterDeltaShift
        beltVelocity=masterDeltaShift/dt
    else
        local dv=maxVel-beltVelocity
        if math.abs(dv)>accel*dt then
            beltVelocity=beltVelocity+accel*dt*math.abs(dv)/dv
        else
            beltVelocity=maxVel
        end
        totShift=totShift+dt*beltVelocity
    end
    
    sim.setObjectFloatParam(textureA,sim.shapefloatparam_texture_y,totShift)

    if textureB~=-1 then
        sim.setObjectFloatParam(textureB,sim.shapefloatparam_texture_y,length*0.5+0.041574*height/0.2+totShift)
        sim.setObjectFloatParam(textureC,sim.shapefloatparam_texture_y,-length*0.5-0.041574*height/0.2+totShift)
        local a=sim.getJointPosition(jointB)
        sim.setJointPosition(jointB,a-beltVelocity*dt*2/height)
        sim.setJointPosition(jointC,a-beltVelocity*dt*2/height)
    end
    
    relativeLinearVelocity={0,beltVelocity,0}
    
    sim.resetDynamicObject(forwarderA)
    m=sim.getObjectMatrix(forwarderA,-1)
    m[4]=0
    m[8]=0
    m[12]=0
    absoluteLinearVelocity=sim.multiplyVector(m,relativeLinearVelocity)
    sim.setObjectFloatParam(forwarderA,sim.shapefloatparam_init_velocity_x,absoluteLinearVelocity[1])
    sim.setObjectFloatParam(forwarderA,sim.shapefloatparam_init_velocity_y,absoluteLinearVelocity[2])
    sim.setObjectFloatParam(forwarderA,sim.shapefloatparam_init_velocity_z,absoluteLinearVelocity[3])
    data['encoderDistance']=totShift
    sim.writeCustomStringData(model,simBWF.modelTags.CONVEYOR,sim.packTable(data))
end 