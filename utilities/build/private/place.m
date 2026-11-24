function place(blk, x, yc)
    % move a block so its left edge is at x and its centre at height yc
    pos = get_param(blk, "Position");
    w = pos(3) - pos(1);  h = pos(4) - pos(2);
    set_param(blk, "Position", round([x, yc - h/2, x + w, yc + h/2]));
end
