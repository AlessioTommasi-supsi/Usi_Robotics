function model.setCuboidMassAndInertia(h,sizeX,sizeY,sizeZ,mass,inertiaFact)
    local inertiaFact=1
    local transf=sim.getObjectMatrix(h,-1)
    local inertia={(sizeY*sizeY+sizeZ*sizeZ)*mass*inertiaFact/12,0,0,0,(sizeX*sizeX+sizeZ*sizeZ)*mass*inertiaFact/12,0,0,0,(sizeY*sizeY+sizeX*sizeX)*mass*inertiaFact/12}
    sim.setShapeMassAndInertia(h,mass,inertia,{0,0,0},transf)
end

function model.setColor(red,green,blue,spec)
    sim.setShapeColor(model.handle,nil,sim.colorcomponent_ambient_diffuse,{red,green,blue})
end

function model.getColor()
    local r,rgb=sim.getShapeColor(model.handle,nil,sim.colorcomponent_ambient_diffuse)
    return rgb[1],rgb[2],rgb[3]
end

function model.updateModel()
    local c=model.readInfo()
    local partInfo=model.readPartInfo()
    local partLabelC=partInfo['labelData']
    local w=c.partSpecific['width']
    local l=c.partSpecific['length']
    local h=c.partSpecific['height']
    local bitCText=c.partSpecific['bitCoded']
    local bitC=partLabelC['bitCoded']
    local mass=c.partSpecific['mass']
    local boxSize={w,l,h}
    local smallLabelSize=partLabelC['smallLabelSize']
    local largeLabelSize=partLabelC['largeLabelSize']

    partLabelC['boxSize']={w,l,h}
    partInfo['labelData']=partLabelC
    model.writePartInfo(partInfo)

    model.setObjectSize(model.handle,w,l,h)
    model.setCuboidMassAndInertia(model.handle,w,l,h,mass)

    model.setObjectSize(model.specHandles.smallLabel,smallLabelSize[1],smallLabelSize[2])
    -- Scale also the texture:
    sim.setObjectFloatParam(model.specHandles.smallLabel,sim.shapefloatparam_texture_scaling_y,0.11*smallLabelSize[1]/0.075)
    sim.setObjectFloatParam(model.specHandles.smallLabel,sim.shapefloatparam_texture_scaling_x,0.11*smallLabelSize[2]/0.0375)
    sim.setObjectFloatParam(model.specHandles.smallLabel,sim.shapefloatparam_texture_y,0.037*smallLabelSize[2]/0.0375)

    model.setObjectSize(model.specHandles.largeLabel,largeLabelSize[1],largeLabelSize[2])
    -- Scale also the texture:
    sim.setObjectFloatParam(model.specHandles.largeLabel,sim.shapefloatparam_texture_scaling_y,0.11*largeLabelSize[1]/0.075)
    sim.setObjectFloatParam(model.specHandles.largeLabel,sim.shapefloatparam_texture_scaling_x,0.11*largeLabelSize[2]/0.1125)

    local textureId=sim.getShapeTextureId(model.specHandles.texture)

    if (bitCText&4)>0 then
        -- textured
        sim.setShapeTexture(model.handle,textureId,sim.texturemap_cube,4+8,{0.3,0.3})
    else
        -- without texture
        sim.setShapeTexture(model.handle,-1,sim.texturemap_cube,4+8,{0.3,0.3})
    end
    
    -- Now the labels:

    -- Remove the current labels:
    local objs=sim.getObjectsInTree(model.handle,sim.handle_all,1)
    for i=1,#objs,1 do
        local h=objs[i]
        if h~=model.specHandles.texture and h~=model.specHandles.smallLabel and h~=model.specHandles.largeLabel then
            sim.removeObjects({h})
        end
    end
    
    -- Now process the 3 potential labels:
    for labelInd=1,3,1 do
        if (bitC&8*(2^(labelInd-1)))>0 then
            local useLargeLabel=((bitC&64*(2^(labelInd-1)))>0)
            local labelSize=smallLabelSize
            local modelLabelHandle=model.specHandles.smallLabel
            if useLargeLabel then
                labelSize=largeLabelSize
                modelLabelHandle=model.specHandles.largeLabel
            end
            local h=sim.copyPasteObjects({modelLabelHandle},0)[1]
            sim.setObjectParent(h,model.handle,true)
            sim.setObjectInt32Param(h,sim.objintparam_visibility_layer,255) -- make it visible
            sim.setObjectSpecialProperty(h,sim.objectspecialproperty_detectable_all+sim.objectspecialproperty_renderable) -- make it collidable, measurable, detectable, etc.
            local code=partLabelC['placementCode'][labelInd]
            local toExecute='local boxSizeX='..boxSize[1]..'\n'
            toExecute=toExecute..'local boxSizeY='..boxSize[2]..'\n'
            toExecute=toExecute..'local boxSizeZ='..boxSize[3]..'\n'
            toExecute=toExecute..'local labelSizeX='..labelSize[1]..'\n'
            toExecute=toExecute..'local labelSizeY='..labelSize[2]..'\n'
            toExecute=toExecute..'local labelRadius='..(0.5*math.sqrt(labelSize[1]*labelSize[1]+labelSize[2]*labelSize[2]))..'\n'

            toExecute=toExecute..'return {'..code..'}'
            local res,theTable=sim.executeLuaCode(toExecute)
            sim.setObjectPosition(h,model.handle,theTable[1])
            sim.setObjectOrientation(h,model.handle,theTable[2])
            local labelData={}
            labelData['labelIndex']=labelInd
            sim.writeCustomStringData(h,simBWF.modelTags.LABEL_PART,sim.packTable(labelData))
        end
    end
end

function sysCall_cleanup_specific()
    sim.removeObjects({model.specHandles.texture})
    sim.removeObjects({model.specHandles.smallLabel})
    sim.removeObjects({model.specHandles.largeLabel})
end

