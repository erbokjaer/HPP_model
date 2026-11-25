function addMatlabFunction(blk, fname, inNames, outNames, body, paramName, pos)
    % MATLAB Function block with the given ports; paramName ("" for none)
    % becomes a Parameter data item resolved from the model / mask workspace
    add_block("simulink/User-Defined Functions/MATLAB Function", blk, "Position", pos);
    rt = sfroot;
    ch = rt.find("-isa", "Stateflow.EMChart", "Path", char(blk));
    args = inNames;
    if paramName ~= "", args(end+1) = paramName; end
    sig = "function [" + strjoin(outNames, ", ") + "] = " + fname + "(" + strjoin(args, ", ") + ")";
    ch.Script = char(strjoin([sig; "%#codegen"; body(:); "end"], newline));
    if paramName ~= ""
        d = ch.find("-isa", "Stateflow.Data", "Name", char(paramName));
        d.Scope = 'Parameter';
    end
end
