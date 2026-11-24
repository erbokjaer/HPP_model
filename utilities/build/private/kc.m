function kc(sys, name, value)
    % parameter constant (shows its value; name hidden)
    w = max(40, 7*strlength(string(value)) + 14);
    add_block("simulink/Sources/Constant", sys + "/" + name, "Value", value, ...
        "Position", [0 0 w 24], "ShowName", "off");
end
