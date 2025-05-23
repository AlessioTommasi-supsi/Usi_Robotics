simBWF=require('simBWF')
local isCustomizationScript=sim.getScriptAttribute(sim.getScriptAttribute(sim.handle_self,sim.scriptattribute_scripthandle),sim.scriptattribute_scripttype)==sim.scripttype_customization

if false then -- if not sim.isPluginLoaded('Bwf') then
    function sysCall_init()
    end
else
    function sysCall_init()
        model={}
        simBWF.appendCommonModelData(model,simBWF.modelTags.IOHUB)
        if isCustomizationScript then
            -- Customization script
            if model.modelVersion==1 then
                require("/bwf/scripts/ioHub/common")
                require("/bwf/scripts/ioHub/customization_main")
                require("/bwf/scripts/ioHub/customization_data")
                require("/bwf/scripts/ioHub/customization_ext")
                require("/bwf/scripts/ioHub/customization_dlg")
            end
        end
        sysCall_init() -- one of above's 'require' redefined that function
    end
end
