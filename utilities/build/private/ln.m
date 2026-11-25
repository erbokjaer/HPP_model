function ln(sys, a, b, name)
    % line from port a to port b (smart routing); named if a name is given
    if nargin < 4, name = ""; end
    h = add_line(sys, a, b, "autorouting", "smart");
    if name == "", return, end
    srcType = get_param(sys + "/" + extractBefore(a, "/"), "BlockType");
    if srcType ~= "BusSelector"                 % bus selector outputs keep the element name
        set_param(h, "Name", name);
    end
end
