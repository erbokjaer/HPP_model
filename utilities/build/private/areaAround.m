function h = areaAround(sys, title, blks, color)
    box = [inf inf -inf -inf];
    for b = blks
        p = get_param(sys + "/" + b, "Position");
        box = [min(box(1), p(1)), min(box(2), p(2)), max(box(3), p(3)), max(box(4), p(4))];
    end
    h = add_block("built-in/Area", sys + "/" + title, "Position", box + [-15 -36 15 26]);
    set_param(h, "BackgroundColor", color);
end
