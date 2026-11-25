function putAt(mdl, n, x, kind, k, y)
    % block n with its left edge at x and its port (kind "in" / "out", k)
    % at height y
    blk = mdl + "/" + n;
    p = get_param(blk, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
    dy = y - portY(blk, kind, k);
    set_param(blk, "Position", round([x, p(2) + dy, x + w, p(2) + dy + h]));
end
