function hOut = ctrlOutputs(sys, names, srcs, x, cm)
    % From -> Outport for every output, in a column on the right; srcs are
    % Goto tags or names of existing source blocks
    y = 40;  all = strings(0);
    for k = 1:numel(names)
        add_block("simulink/Sinks/Out1", sys + "/" + names(k), "Port", num2str(k), ...
            "OutDataTypeStr", "double", "Position", [0 0 30 14]);
        s = srcs(k);
        if ~startsWith(s, "@") && getSimulinkBlockHandle(sys + "/" + s) == -1, s = "@" + s; end
        [b, ~, bl] = rowChain(sys, x, y, [stg(s, 0), stg(names(k))], cm);
        y = b + 34;  all = [all, bl]; %#ok<AGROW>
    end
    hOut = areaAround(sys, "Outputs", all, areaColor("interface"));
end
