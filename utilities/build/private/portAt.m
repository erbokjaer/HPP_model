function portAt(sys, name, xy)
    set_param(sys + "/" + name, "Position", [xy(1), xy(2) - 7, xy(1) + 30, xy(2) + 7]);
end
