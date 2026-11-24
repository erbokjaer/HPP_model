function name = addFrom(sys, tag, color)
    % From for a local Goto; returns the (unique) block name
    i = 1;
    while getSimulinkBlockHandle(sys + "/" + sprintf("%s from %d", tag, i)) ~= -1
        i = i + 1;
    end
    name = sprintf("%s from %d", tag, i);
    add_block("simulink/Signal Routing/From", sys + "/" + name, "GotoTag", tag, ...
        "BackgroundColor", color, "ShowName", "off", "Position", [0 0 12 + 8*strlength(tag) 22]);
end
