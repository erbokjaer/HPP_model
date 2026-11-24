function rep = applyLayout(sys, S, opts)
%APPLYLAYOUT Put a captured layout (captureLayout) on one system level.
%   rep = applyLayout(sys, S) with S the struct or JSON file from
%   captureLayout. Blocks are matched by name (opts.Map: captured name ->
%   name here, also used for Goto tags, for a model of similar structure
%   with other block names). The signal connections come from sys itself:
%   every captured line or Goto / From that connects the same source and
%   destination is drawn as captured (exact route), every other connection
%   is drawn as a line with automatic routing. The signal connections
%   (Goto / From resolved) are the same before and after; if not, error.
%   rep lists the captured blocks missing here, the blocks here without a
%   captured place, and the connections drawn by automatic routing.
%   opts.Place: {name, x, yc; ...} puts blocks without a fitting captured
%   place at x (left edge; NaN: the column of the inports) and centre yc.
%   opts.Tags: dictionary source -> Goto tag, the source "block:port" or
%   "block" (its output 1); the connections from it are drawn as Goto /
%   From (rule E1: where a line would cross).
arguments
    sys
    S
    opts.Map = dictionary(string.empty, string.empty)
    opts.Place cell = cell(0, 3)
    opts.Tags = dictionary(string.empty, string.empty)
    opts.Keep (1,:) string = strings(1, 0)   % "src->dst": stays a line although src is in Tags
end
if ~isstruct(S), S = jsondecode(fileread(S, "Encoding", "UTF-8")); end
sys = string(getfullname(sys));
mapn = @(n) mapName(opts.Map, string(n));
E0 = signalEdges(sys);
rep = struct("missing", strings(0, 1), "unplaced", strings(0, 1), "autorouted", strings(0, 1));

% 1. clear: lines, Goto / From, areas
delete_line(find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "line"));
gf = [find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "Goto"); ...
      find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "From")];
if ~isempty(gf), delete_block(gf); end
A = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "annotation", "AnnotationType", "area_annotation");
for a = A(:)', delete(get_param(a, "Object")); end

% 2. blocks at their captured place
blocks = S.blocks(:);
placed = strings(0, 1);
for b = blocks'
    if any(b.type == ["Goto", "From"]), continue, end
    n = mapn(b.name);  full = sys + "/" + n;
    if getSimulinkBlockHandle(full) == -1, rep.missing(end+1) = n; continue, end
    % shape first: a round sum puts its ports elsewhere
    if b.type == "Sum" && get_param(full, "BlockType") == "Sum" && ...
            erase(string(b.inputs), "|") == erase(string(get_param(full, "Inputs")), "|")
        set_param(full, "IconShape", b.shape, "Inputs", b.inputs);
    end
    if b.type == string(get_param(full, "BlockType"))
        set_param(full, "Orientation", b.orient, "Position", b.pos(:)', "ShowName", b.showName);
    else
        % another kind of block in this place: its own size, same centre
        p = get_param(full, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
        c = [(b.pos(1) + b.pos(3)) / 2, (b.pos(2) + b.pos(4)) / 2];
        set_param(full, "Position", round([c(1) - w / 2, c(2) - h / 2, c(1) + w / 2, c(2) + h / 2]));
    end
    placed(end+1) = n; %#ok<AGROW>
end
% placed by hand (opts.Place)
inCol = arrayfun(@(b) b.type == "Inport", blocks);
for k = 1:size(opts.Place, 1)
    full = sys + "/" + opts.Place{k, 1};
    if getSimulinkBlockHandle(full) == -1, continue, end
    p = get_param(full, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
    x = opts.Place{k, 2};
    if isnan(x), x = median(cell2mat(arrayfun(@(b) b.pos(1), blocks(inCol), "UniformOutput", false))); end
    yc = opts.Place{k, 3};
    set_param(full, "Position", round([x, yc - h / 2, x + w, yc + h / 2]));
    placed = union(placed, string(opts.Place{k, 1}));
end
% columns of the in- and outports
isIn = arrayfun(@(b) b.type == "Inport", blocks);  isOut = arrayfun(@(b) b.type == "Outport", blocks);
inX = [];  outX = [];
if any(isIn), inX = median(cell2mat(arrayfun(@(b) b.pos(1), blocks(isIn), "UniformOutput", false))); end
if any(isOut), outX = median(cell2mat(arrayfun(@(b) b.pos(1), blocks(isOut), "UniformOutput", false))); end
% blocks without a captured place: next to a placed neighbour
here = string(find_system(sys, "SearchDepth", 1, "LookUnderMasks", "all", "FollowLinks", "on", "Type", "block"));
here = here(here ~= sys);
here = setdiff(string(get_param(here, "Name")), placed);
for n = here(:)'
    rep.unplaced(end+1) = n;
    full = sys + "/" + n;  p = get_param(full, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
    [s, d] = edgeEnds(E0, n);
    nb = intersect([s, d], placed, "stable");
    if isempty(nb), continue, end
    q = get_param(sys + "/" + nb(1), "Position");
    bt = get_param(full, "BlockType");
    if (bt == "Inport" && ~isempty(inX)) || (bt == "Outport" && ~isempty(outX))
        % a port in the column of the ports, level with its user / source,
        % moved down until it hits nothing (20 px for its name)
        if bt == "Inport", px = inX; else, px = outX; end
        yc = (q(2) + q(4)) / 2;
        r = round([px, yc - h / 2, px + w, yc + h / 2]);
        while hitsAny(sys, r, full), r = r + [0 10 0 10]; end
        set_param(full, "Position", r);
    elseif any(s == nb(1))      % right of its source
        set_param(full, "Position", [q(3) + 40, (q(2) + q(4)) / 2 - h / 2, q(3) + 40 + w, (q(2) + q(4)) / 2 + h / 2]);
    else                    % left of its user
        set_param(full, "Position", [q(1) - 40 - w, (q(2) + q(4)) / 2 - h / 2, q(1) - 40, (q(2) + q(4)) / 2 + h / 2]);
    end
    placed(end+1) = n; %#ok<AGROW>
end

% 3. captured Goto / From blocks (tags mapped)
for b = blocks'
    if ~any(b.type == ["Goto", "From"]), continue, end
    lib = "simulink/Signal Routing/" + b.type;
    add_block(lib, sys + "/" + b.name, "GotoTag", mapn(b.tag), "Position", b.pos(:)', "ShowName", b.showName);
end

% 4. captured lines, leaf by leaf; a branch starts on the line drawn before
trees = S.trees(:);
for t = 1:numel(trees)
    tr = trees(t);
    src = mapn(tr.src);
    if getSimulinkBlockHandle(sys + "/" + src) == -1, continue, end
    segs = tr.segs(:);
    made = false(numel(segs), 1);
    for k = 1:numel(segs)
        if segs(k).dst == "", continue, end
        dst = mapn(segs(k).dst);
        if getSimulinkBlockHandle(sys + "/" + dst) == -1, continue, end
        sp = portH(sys + "/" + src, "outport", tr.sp);
        dp = portH(sys + "/" + dst, segs(k).dt, segs(k).dp);
        if isempty(sp) || isempty(dp) || get_param(dp, "Line") > 0, continue, end
        chain = k;
        while segs(chain(1)).parent > 0 && ~made(segs(chain(1)).parent)
            chain = [segs(chain(1)).parent, chain]; %#ok<AGROW>
        end
        pts = zeros(0, 2);
        for c = chain
            P = segs(c).points;
            if ~isempty(pts) && isequal(pts(end, :), P(1, :)), P = P(2:end, :); end
            pts = [pts; P]; %#ok<AGROW>
        end
        ok = false;
        try
            add_line(sys, pts);
            l = get_param(dp, "Line");
            ok = l > 0 && get_param(l, "SrcPortHandle") == sp;
            if ~ok && l > 0, delete_line(l); end
        catch
        end
        if ~ok
            add_line(sys, sp, dp, "autorouting", "smart");
            rep.autorouted(end+1) = src + " -> " + dst;
        end
        made(chain) = true;
    end
end
% dangling pieces of routes (a point that hit no port) are removed
for l = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "line")'
    if ishandle(l) && (get_param(l, "SrcPortHandle") < 0 || all(get_param(l, "DstPortHandle") < 0))
        delete_line(l);
    end
end

% connections from the blocks in opts.Tags: Goto on the source port,
% From in front of each user
if numEntries(opts.Tags) > 0
    Ecur = signalEdges(sys);
    for e = setdiff(E0, Ecur)'
        [sn, spn, dn, dt, dpn] = parseEdge(e);
        % key "block:port" (one output) or "block" (its first output)
        if any(opts.Keep == sn + "->" + dn), continue, end
        if isKey(opts.Tags, sn + ":" + spn)
            t = opts.Tags(sn + ":" + spn);
        elseif isKey(opts.Tags, sn) && spn == 1
            t = opts.Tags(sn);
        else
            continue
        end
        sp = portH(sys + "/" + sn, "outport", spn);  dp = portH(sys + "/" + dn, dt, dpn);
        w = 7 * strlength(t) + 20;
        gx = find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "Goto", "GotoTag", t);
        if ~isempty(gx)
            % a captured Goto with this tag whose source is not here (e.g.
            % [dip] from Voltage_dip in the template): fed from this source
            ph = get_param(gx(1), "PortHandles");
            if get_param(ph.Inport, "Line") < 0
                add_line(sys, sp, ph.Inport, "autorouting", "smart");
            end
        else
            ps = get_param(sp, "Position");
            g = add_block("simulink/Signal Routing/Goto", sys + "/Goto " + t, "GotoTag", t, ...
                "ShowName", "off", "Position", round([ps(1) + 20, ps(2) - 8, ps(1) + 20 + w, ps(2) + 8]));
            ph = get_param(g, "PortHandles");  add_line(sys, sp, ph.Inport, "autorouting", "smart");
        end
        l = get_param(dp, "Line");
        if l > 0
            sb = get_param(get_param(l, "SrcPortHandle"), "Parent");
            if get_param(sb, "BlockType") == "From" && string(get_param(sb, "GotoTag")) == t
                continue                            % a captured From with this tag: right now
            end
            delete_line(l);                          % a wrong captured line
        end
        pd = get_param(dp, "Position");
        f = add_block("simulink/Signal Routing/From", sys + "/From " + t, "MakeNameUnique", "on", "GotoTag", t, ...
            "ShowName", "off", "Position", round([pd(1) - 20 - w, pd(2) - 8, pd(1) - 20, pd(2) + 8]));
        ph = get_param(f, "PortHandles");  add_line(sys, ph.Outport, dp, "autorouting", "smart");
    end
end

% 5. same signal connections: wrong ones removed, missing ones drawn
E1 = signalEdges(sys);
for e = setdiff(E1, E0)'
    [~, ~, dn, dt, dpn] = parseEdge(e);
    dp = portH(sys + "/" + dn, dt, dpn);
    l = get_param(dp, "Line");  if l > 0, delete_line(l); end
end
for e = setdiff(E0, signalEdges(sys))'
    [sn, spn, dn, dt, dpn] = parseEdge(e);
    add_line(sys, portH(sys + "/" + sn, "outport", spn), portH(sys + "/" + dn, dt, dpn), "autorouting", "smart");
    rep.autorouted(end+1) = sn + " -> " + dn;
end
% captured Goto / From blocks without a signal here (their source or
% user is not in this model) go
for g = [find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "Goto"); ...
         find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "From")]'
    ph = get_param(g, "PortHandles");
    p = [ph.Inport, ph.Outport];
    if get_param(p(1), "Line") < 0, delete_block(g); end
end
% Goto blocks nobody reads (local tags) go, except the captured ones
% (a label kept by hand, e.g. [f_pll])
used = string(get_param(find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "From"), "GotoTag"));
capG = string(arrayfun(@(b) b.name, blocks(arrayfun(@(b) b.type == "Goto", blocks)), "UniformOutput", false));
for g = find_system(sys, "SearchDepth", 1, "FindAll", "on", "BlockType", "Goto")'
    if any(capG == string(get_param(g, "Name"))), continue, end
    if get_param(g, "TagVisibility") == "local" && ~any(used == string(get_param(g, "GotoTag")))
        ph = get_param(g, "PortHandles");  l = get_param(ph.Inport, "Line");
        if l > 0, delete_line(l); end
        delete_block(g);
    end
end
E2 = signalEdges(sys);
if ~isequal(sort(E2), sort(E0))
    error("applyLayout:connections", "Signal connections changed in %s:\n  removed: %s\n  added: %s", sys, ...
        strjoin(setdiff(E0, E2), "; "), strjoin(setdiff(E2, E0), "; "));
end

% 6. areas
for a = S.areas(:)'
    h = add_block("built-in/Area", sys + "/" + mapn(a.title), "Position", a.pos(:)');
    set_param(h, "BackgroundColor", a.color);
end
end

% ---------------------------------------------------------------------
function tf = hitsAny(sys, r, self)
    % r (with 20 px below for the name) overlaps another block
    b = find_system(sys, "SearchDepth", 1, "LookUnderMasks", "all", "FollowLinks", "on", "Type", "block");
    b = string(b(~strcmp(b, sys) & ~strcmp(b, self)));
    tf = false;
    for k = 1:numel(b)
        p = get_param(b(k), "Position");
        if r(1) < p(3) + 5 && r(3) > p(1) - 5 && r(2) < p(4) + 22 && r(4) + 22 > p(2), tf = true; return, end
    end
end

function n = mapName(M, n)
    if numEntries(M) > 0 && isKey(M, n), n = M(n); end
end

function [s, d] = edgeEnds(E, n)
    % sources and users of block n
    s = strings(1, 0);  d = strings(1, 0);
    for e = E(:)'
        [sn, ~, dn] = parseEdge(e);
        if dn == n, s(end+1) = sn; end %#ok<AGROW>
        if sn == n, d(end+1) = dn; end %#ok<AGROW>
    end
end

function [sn, spn, dn, dt, dpn] = parseEdge(e)
    p = split(e, " -> ");  a = split(p(1), ":");  b = split(p(2), ":");   % signalEdges format
    sn = join(a(1:end-1), ":");  spn = double(a(end));
    dn = join(b(1:end-2), ":");  dt = b(end-1);  dpn = double(b(end));
end

function p = portH(blk, type, k)
    ph = get_param(blk, "PortHandles");
    switch string(type)
        case "outport",  f = ph.Outport;
        case "inport",   f = ph.Inport;
        case "enable",   f = ph.Enable;
        case "trigger",  f = ph.Trigger;
        case "ifaction", f = ph.Ifaction;
        case "reset",    f = ph.Reset;
        otherwise,       f = [];
    end
    if numel(f) >= k, p = f(k); else, p = []; end
    if any(string(type) == ["enable", "trigger", "ifaction", "reset"]) && ~isempty(f), p = f(1); end
end
