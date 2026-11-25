function c = palette(name)
    % label colours (DTU palette): blue references / orders / commands,
    % green measurements, orange internal signals and flags, grey logging
    switch name
        case "blue",   c = "[0.592 0.620 0.957]";
        case "green",  c = "[0.122 0.816 0.510]";
        case "orange", c = "[0.988 0.463 0.204]";
        case "grey",   c = "[0.855 0.855 0.855]";
    end
end
