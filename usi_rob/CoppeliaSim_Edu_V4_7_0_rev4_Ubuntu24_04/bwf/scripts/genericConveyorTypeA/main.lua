simBWF=require('simBWF')
local isCustomizationScript=sim.getScriptAttribute(sim.getScriptAttribute(sim.handle_self,sim.scriptattribute_scripthandle),sim.scriptattribute_scripttype)==sim.scripttype_customization

if false then -- if not sim.isPluginLoaded('Bwf') then
    function sysCall_init()
    end
else
    function sysCall_init()
        model={}
        simBWF.appendCommonModelData(model,simBWF.modelTags.CONVEYOR)
        model.conveyorType='A'
        if isCustomizationScript then
            -- Customization script
            if model.modelVersion==1 then
                -- Common:
                require("/bwf/scripts/conveyor_common/common")
                require("/bwf/scripts/conveyor_common/customization_main")
                require("/bwf/scripts/conveyor_common/customization_data")
                require("/bwf/scripts/conveyor_common/customization_ext")
                require("/bwf/scripts/conveyor_common/customization_dlg")
                -- Type A specific:
                require("/bwf/scripts/genericConveyorTypeA/common")
                require("/bwf/scripts/genericConveyorTypeA/customization_main")
                require("/bwf/scripts/genericConveyorTypeA/customization_dlg")
            end
        else
            -- Child script
            if model.modelVersion==1 then
                -- Common:
                require("/bwf/scripts/conveyor_common/common")
                require("/bwf/scripts/conveyor_common/child_main")
                -- Type A specific:
                require("/bwf/scripts/genericConveyorTypeA/common")
                require("/bwf/scripts/genericConveyorTypeA/child_main")
            end
        end
        sysCall_init() -- one of above's 'require' redefined that function
    end
end
