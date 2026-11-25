function y = portY(blk, kind, k)
    ph = get_param(blk, "PortHandles");
    if kind == "in", h = ph.Inport(k); else, h = ph.Outport(k); end
    pos = get_param(h, "Position");
    y = pos(2);
end
