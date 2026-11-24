function placeLeft(sys, b, dst, port, gap)
    % b with its output level with input port of dst, gap px to its left
    q = get_param(dst, "PortHandles");  pin = get_param(q.Inport(port), "Position");
    p = get_param(sys + "/" + b, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
    set_param(sys + "/" + b, "Position", round([pin(1) - gap - w, pin(2) - h / 2, pin(1) - gap, pin(2) + h / 2]));
end
