function C = dtuColors(style)
%DTUCOLORS  Line colours from the DTU design guide, as an N-by-3 RGB matrix.
%
%   C = dtuColors()          palette for light backgrounds (default)
%   C = dtuColors("dark")    palette for dark backgrounds
%
%   Use as a colour order, e.g. colororder(ax, dtuColors()).
%
%   Light: DTU blue, corporate red, green, orange, purple, navy, red.
%   Dark:  the same hues as lighter DTU tints (blue-60, red, bright green,
%          orange, purple-50, yellow, pink), because navy and corporate red
%          are unreadable on dark axes.
%   The order keeps neighbouring lines far apart in hue and lightness.

    arguments
        style (1,1) string {mustBeMember(style, ["light", "dark"])} = "light"
    end

    if style == "light"
        hex = ["#2F3EEA"    % blue
               "#990000"    % corporate red
               "#008835"    % green
               "#FC7634"    % orange
               "#79238E"    % purple
               "#030F4F"    % navy blue
               "#E83F48"];  % red
    else
        hex = ["#828BF2"    % blue-60
               "#E83F48"    % red
               "#1FD082"    % bright green
               "#FC7634"    % orange
               "#BC91C6"    % purple-50
               "#F6D04D"    % yellow
               "#F7BBB1"];  % pink
    end

    C = validatecolor(hex, "multiple");
end
