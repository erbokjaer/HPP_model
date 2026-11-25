function [bottom, right, blks] = rowChain(sys, x, yTop, stages, cm)
    % Place one row of stages (see stg) with its top at yTop, starting at x;
    % returns the lowest edge, the right edge and all blocks of the row
    n = numel(stages);
    names = strings(1, n);  sideNames = cell(1, n);  outNames = cell(1, n);
    blks = strings(0);
    y = 0;  xc = x;
    for i = 1:n
        st = stages(i);  b = st.blk;
        if startsWith(b, "@")
            t = extractAfter(b, 1);  b = addFrom(sys, t, tagColor(cm, t));
        elseif startsWith(b, ">")
            t = extractAfter(b, 1);  b = addGoto(sys, t, tagColor(cm, t));
        end
        names(i) = b;  full = sys + "/" + b;
        sb = strings(1, size(st.sides, 1));  wmax = 0;
        for j = 1:size(st.sides, 1)
            s = string(st.sides{j, 2});
            if startsWith(s, "@")
                t = extractAfter(s, 1);  s = addFrom(sys, t, tagColor(cm, t));
            end
            sb(j) = s;
            p = get_param(sys + "/" + s, "Position");  wmax = max(wmax, p(3) - p(1));
        end
        gap = 35;
        if wmax > 0, gap = wmax + 50; end
        if i == 1, gap = 0; end
        p = get_param(full, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
        xl = xc + gap;
        set_param(full, "Position", [xl, 0, xl + w, h]);
        if st.ip > 0
            dy = y - portY(full, "in", st.ip);
        else
            dy = y - portY(full, "out", 1);
        end
        set_param(full, "Position", [xl, dy, xl + w, dy + h]);
        for j = 1:numel(sb)
            sp = get_param(sys + "/" + sb(j), "Position");  sw = sp(3) - sp(1);  shh = sp(4) - sp(2);
            yc = portY(full, "in", st.sides{j, 1});
            set_param(sys + "/" + sb(j), "Position", round([xl - 20 - sw, yc - shh/2, xl - 20, yc + shh/2]));
        end
        ob = strings(1, size(st.outs, 1));
        for j = 1:size(st.outs, 1)
            t = string(st.outs{j, 2});
            ob(j) = addGoto(sys, t, tagColor(cm, t));
            gp = get_param(sys + "/" + ob(j), "Position");
            yc = portY(full, "out", st.outs{j, 1});
            set_param(sys + "/" + ob(j), "Position", round([xl + w + 30, yc - (gp(4)-gp(2))/2, ...
                xl + w + 30 + gp(3) - gp(1), yc + (gp(4)-gp(2))/2]));
        end
        sideNames{i} = sb;  outNames{i} = ob;
        ph = get_param(full, "PortHandles");
        if ~isempty(ph.Outport), y = portY(full, "out", 1); end
        xc = xl + w;
        blks = [blks, b, sb, ob]; %#ok<AGROW>
    end
    % shift the row to its place, then draw (straight) lines
    top = inf;  bottom = -inf;  right = -inf;
    for b = blks
        p = get_param(sys + "/" + b, "Position");
        top = min(top, p(2));  bottom = max(bottom, p(4));  right = max(right, p(3));
    end
    d = round(yTop - top);
    for b = blks
        set_param(sys + "/" + b, "Position", get_param(sys + "/" + b, "Position") + [0 d 0 d]);
    end
    bottom = bottom + d;
    for i = 1:n
        st = stages(i);
        if i > 1 && st.ip > 0
            add_line(sys, names(i-1) + "/1", names(i) + "/" + st.ip);
        end
        for j = 1:numel(sideNames{i})
            add_line(sys, sideNames{i}(j) + "/1", names(i) + "/" + st.sides{j, 1});
        end
        for j = 1:numel(outNames{i})
            add_line(sys, names(i) + "/" + st.outs{j, 1}, outNames{i}(j) + "/1");
        end
    end
end
