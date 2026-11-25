%% ===================================================================
%  Loop diagrams (PLL, grid frequency): small helpers for hand-placed
%  blocks, so feedback is drawn as a loop (flipped delay blocks)
%  ===================================================================
function b = pb(sys, kind, name, xy, varargin)
    % standard block (nb) placed with its left edge at x, centred on y
    nb(sys, kind, name, varargin{:});
    b = sys + "/" + name;
    p = get_param(b, "Position");  w = p(3) - p(1);  h = p(4) - p(2);
    set_param(b, "Position", [xy(1), xy(2) - h/2, xy(1) + w, xy(2) + h/2]);
end
