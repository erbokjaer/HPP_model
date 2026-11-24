function r = countCrossings(sys)
%COUNTCROSSINGS Layout quality of one system level.
%   r.cross      crossings between lines of different signals
%                (a diagonal segment is not counted)
%   r.overBlock  line segments running through a block that is not theirs
%   r.froms      From blocks
%   r.lines      line objects
sys = string(getfullname(sys));
lines = find_system(sys, "SearchDepth", 1, "FindAll", "on", "Type", "line");
S = zeros(0, 6);
for k = 1:numel(lines)
    P = get_param(lines(k), "Points");
    src = get_param(lines(k), "SrcPortHandle");
    for q = 1:size(P, 1) - 1
        S(end+1, :) = [P(q, :) P(q + 1, :) k src]; %#ok<AGROW>
    end
end
r.cross = 0;
for i = 1:size(S, 1)
    for j = i + 1:size(S, 1)
        if S(i, 6) ~= S(j, 6) && crossHV(S(i, 1:4), S(j, 1:4)), r.cross = r.cross + 1; end
    end
end
blk = find_system(sys, "SearchDepth", 1, "LookUnderMasks", "all", "FollowLinks", "on", "Type", "block");
blk = blk(~strcmp(blk, sys));
R = zeros(numel(blk), 4);  H = zeros(numel(blk), 1);
for b = 1:numel(blk)
    R(b, :) = get_param(blk{b}, "Position");  H(b) = get_param(blk{b}, "Handle");
end
r.overBlock = 0;
for i = 1:size(S, 1)
    l = lines(S(i, 5));
    own = [get_param(l, "SrcBlockHandle"); get_param(l, "DstBlockHandle")];
    t = linspace(0, 1, max(2, ceil(norm(S(i, 3:4) - S(i, 1:2)) / 5)))';
    pts = S(i, 1:2) + t .* (S(i, 3:4) - S(i, 1:2));
    for b = 1:numel(blk)
        if any(own == H(b)), continue, end
        in = pts(:, 1) > R(b, 1) + 2 & pts(:, 1) < R(b, 3) - 2 & pts(:, 2) > R(b, 2) + 2 & pts(:, 2) < R(b, 4) - 2;
        if any(in), r.overBlock = r.overBlock + 1; break, end
    end
end
r.froms = numel(find_system(sys, "SearchDepth", 1, "BlockType", "From"));
r.lines = numel(lines);
end

function tf = crossHV(a, b)
tf = false;
ha = a(2) == a(4);  va = a(1) == a(3);
hb = b(2) == b(4);  vb = b(1) == b(3);
if ha && vb
elseif va && hb, [a, b] = deal(b, a);
else, return
end
x = b(1);  y = a(2);
tf = x > min(a([1 3])) + 1 && x < max(a([1 3])) - 1 && y > min(b([2 4])) + 1 && y < max(b([2 4])) - 1;
end
