simBWF=require('simBWF')
local isCustomizationScript=sim.getScriptAttribute(sim.getScriptAttribute(sim.handle_self,sim.scriptattribute_scripthandle),sim.scriptattribute_scripttype)==sim.scripttype_customization

if false then -- if not sim.isPluginLoaded('Bwf') then
    function sysCall_init()
    end
else
    function sysCall_init()
        model={}
        simBWF.appendCommonModelData(model,simBWF.modelTags.PALLETREPOSITORY)
        if isCustomizationScript then
            -- Customization script
            if model.modelVersion==1 then
                require("/bwf/scripts/palletRepository/common")
                require("/bwf/scripts/palletRepository/customization_main")
                require("/bwf/scripts/palletRepository/customization_data")
                require("/bwf/scripts/palletRepository/customization_ext")
                require("/bwf/scripts/palletRepository/customization_dlg")
            end
        else
            -- Child script

        end
        sysCall_init() -- one of above's 'require' redefined that function
    end
end