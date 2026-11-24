function c = areaColor(role)
    % light background tints by role (DTU palette, very light)
    switch role
        case "controller", c = "[0.894 0.906 0.988]";   % blue: controllers / future DLLs
        case "asset",      c = "[0.878 0.965 0.925]";   % green: asset (plant-side) models
        otherwise,         c = "[0.950 0.950 0.950]";   % grey: interfaces, network
    end
end
