function alignColumns(hIn, hMid, hOut)
    % inputs and outputs areas span the full height; the function areas of
    % each column (hMid: one handle array per column) share their left and
    % right edges, and the last area of every column ends at the same height
    if ~iscell(hMid), hMid = {hMid}; end
    q1 = get_param(hIn, "Position");  q2 = get_param(hOut, "Position");
    top = min(q1(2), q2(2));  bot = max(q1(4), q2(4));
    for c = 1:numel(hMid)
        P = cell2mat(arrayfun(@(h) get_param(h, "Position"), hMid{c}(:), "UniformOutput", false));
        top = min(top, min(P(:, 2)));  bot = max(bot, max(P(:, 4)));
    end
    for c = 1:numel(hMid)
        h = hMid{c};
        P = cell2mat(arrayfun(@(x) get_param(x, "Position"), h(:), "UniformOutput", false));
        l = min(P(:, 1));  r = max(P(:, 3));
        for k = 1:numel(h)
            b = P(k, 4);
            if k == numel(h), b = bot; end
            set_param(h(k), "Position", [l, P(k, 2), r, b]);
        end
    end
    for h = [hIn, hOut]
        q = get_param(h, "Position");
        set_param(h, "Position", [q(1), top, q(3), bot]);
    end
end
