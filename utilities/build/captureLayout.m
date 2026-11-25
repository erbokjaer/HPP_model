function S = captureLayout(sys, file)
%CAPTURELAYOUT Store the layout of one system level (a hand layout).
%   S = captureLayout(sys) returns the positions of all blocks, the area
%   boxes, every line with its exact route (branches included), and the
%   Goto / From blocks. captureLayout(sys, file) also writes S as JSON.
%   applyLayout puts it back on a (re)generated model of the same or a
%   similar structure. Only this level: subsystems keep their insides.
sys = string(getfullname(sys));
blk = find_system(sys, "SearchDepth", 1, "LookUnderMasks", "all", "FollowLinks", "on", "Type", "block");
blk = string(blk(~strcmp(blk, sys)));
S.blocks = struct("name", {}, "type", {}, "pos", {}, "orient", {}, "showName", {}, "shape", {}, "inputs", {}, "tag", {});
for b = blk'
    t = string(get_param(b, "BlockType"));
    e = struct("name", string(get_param(b, "Name")), "type", t, "pos", get_param(b, "Position"), ...
        "orient", string(get_param(b, "Orientation")), "showName", string(get_param(b, "ShowName")), ...
        "shape", "", "inputs", "", "tag", "");
    if t == "Sum", e.shape = string(get_param(b, "IconShape")); e.inputs = string(get_param(b, "Inputs")); end
    if t == "Goto" || t == "From", e.tag = string(get_param(b, "GotoTag")); end
    S.blocks(end+1) = e;
end
A = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "annotation", "AnnotationType", "area_annotation");
S.areas = struct("title", {}, "pos", {}, "color", {});
for a = A(:)'
    S.areas(end+1) = struct("title", string(get_param(a, "Text")), "pos", get_param(a, "Position"), ...
        "color", string(get_param(a, "BackgroundColor")));
end
% line trees: the trunk (from the source port) and its branches; a leaf
% ends at a destination port
L = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "line");
S.trees = struct("src", {}, "sp", {}, "segs", {});
for l = L(:)'
    if get_param(l, "LineParent") ~= -1, continue, end
    sp = get_param(l, "SrcPortHandle");
    if sp < 0, continue, end
    segs = struct("id", {}, "parent", {}, "points", {}, "dst", {}, "dt", {}, "dp", {});
    segs = walk(l, 0, segs);
    S.trees(end+1) = struct("src", string(get_param(get_param(sp, "Parent"), "Name")), ...
        "sp", get_param(sp, "PortNumber"), "segs", segs);
end
S.system = sys;
if nargin > 1
    f = fopen(file, "w", "n", "UTF-8");  fprintf(f, "%s", jsonencode(S, "PrettyPrint", true));  fclose(f);
end
end

function segs = walk(l, parent, segs)
    id = numel(segs) + 1;
    ch = get_param(l, "LineChildren");
    dst = "";  dt = "";  dp = 0;
    if isempty(ch)
        d = get_param(l, "DstPortHandle");
        if ~isempty(d) && d(1) > 0
            dst = string(get_param(get_param(d(1), "Parent"), "Name"));
            dt = string(get_param(d(1), "PortType"));  dp = get_param(d(1), "PortNumber");
        end
    end
    segs(id) = struct("id", id, "parent", parent, "points", get_param(l, "Points"), "dst", dst, "dt", dt, "dp", dp);
    for c = ch(:)'
        segs = walk(c, id, segs);
    end
end
