function pk(sys, name, value, xy)
    % parameter constant placed like pb
    kc(sys, name, value);
    p = get_param(sys + "/" + name, "Position");  w = p(3) - p(1);
    set_param(sys + "/" + name, "Position", [xy(1), xy(2) - 12, xy(1) + w, xy(2) + 12]);
end
