function applyCodedLayout(sys, A, tags, keep)
    % finish a coded layout (layoutRepc, layoutHppc, layoutConverter): the
    % blocks are already placed; areas A, lines drawn by applyLayout (labels
    % for the sources in tags except the connections in keep, lines
    % elsewhere; the signal connections are checked)
    S = captureLayout(sys);
    S.blocks = S.blocks(~arrayfun(@(x) any(x.type == ["Goto", "From"]), S.blocks));
    S.trees = S.trees([]);  S.areas = A;
    applyLayout(sys, S, Tags=tags, Keep=keep);
end
