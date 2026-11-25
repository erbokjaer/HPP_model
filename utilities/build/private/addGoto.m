function name = addGoto(sys, tag, color)
    % local Goto (visible only in this system); returns the block name
    name = tag + " goto";
    add_block("simulink/Signal Routing/Goto", sys + "/" + name, "GotoTag", tag, ...
        "TagVisibility", "local", "BackgroundColor", color, "ShowName", "off", ...
        "Position", [0 0 12 + 8*strlength(tag) 22]);
end
