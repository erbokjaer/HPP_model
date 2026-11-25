function s = box(parent, name, ins, outs, rect, desc)
    % empty subsystem with named in- and outports, at rect; ports are placed
    % by the caller (portsAt)
    s = parent + "/" + name;
    newSubsystem(s);
    for k = 1:numel(ins)
        add_block("simulink/Sources/In1", s + "/" + ins(k), "Port", num2str(k), "Position", [0 0 30 14]);
    end
    for k = 1:numel(outs)
        add_block("simulink/Sinks/Out1", s + "/" + outs(k), "Port", num2str(k), "Position", [0 0 30 14]);
    end
    set_param(s, "Position", rect);
    if nargin > 5, set_param(s, "Description", desc); end
end
