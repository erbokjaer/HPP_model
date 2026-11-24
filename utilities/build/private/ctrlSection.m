function [y, h, right] = ctrlSection(sys, title, rows, x, y, cm, color)
    % Function area: rows (cell of stage arrays) stacked from y; returns the
    % y for the next section, the area handle and the right edge
    y = y + 40;                       % area title
    all = strings(0);  right = -inf;
    for r = 1:numel(rows)
        [b, rr, bl] = rowChain(sys, x, y, rows{r}, cm);
        y = b + 34;                   % room for block names
        all = [all, bl]; %#ok<AGROW>
        right = max(right, rr);
    end
    h = areaAround(sys, title, all, color);
    y = y + 10;
end
