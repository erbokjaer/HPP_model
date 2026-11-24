%% ===================================================================
%  Controller diagram layout (asset and plant controller templates)
%
%  Every controller is drawn in three columns of areas:
%    inputs (Inport -> Goto) | function areas, stacked | outputs (From -> Outport)
%  Inside a function area each signal path is one straight row, built by
%  rowChain: blocks left to right, each block's input aligned with the
%  previous block's output; other inputs come from local From blocks or
%  parameter constants placed directly left of their port; a row ends in
%  a Goto. Signals used in several rows are passed with Goto/From
%  (blue: references and orders, green: measurements, orange: internal).
%  ===================================================================
function st = stg(blk, ip, sides, outs)
    % One stage of a row: block blk with main input port ip (0: none).
    % sides = {port, src; ...}: src "@Tag" is a From, otherwise the name of
    % an existing block (a constant) placed left of the port.
    % outs = {port, tag; ...}: further outputs to Gotos.
    % blk "@Tag" creates a From (row start), ">Tag" a Goto (row end).
    if nargin < 2, ip = 1; end
    if nargin < 3, sides = {}; end
    if nargin < 4, outs = {}; end
    st = struct("blk", string(blk), "ip", ip, "sides", {sides}, "outs", {outs});
end
