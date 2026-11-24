function in = setVariants(in, p, delayCode)
%SETVARIANTS  The variant control variables (variantVars) on a SimulationInput.
    for v = variantVars(p, delayCode)
        if v.where == "global"
            in = in.setVariable(v.name, v.value);
        else
            in = in.setVariable(v.name, v.value, "Workspace", v.where);
        end
    end
end
