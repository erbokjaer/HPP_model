function s = delaySignature(p)
%DELAYSIGNATURE  The compile-time part of the communication delays.
%   With p.comm.delays = "fixed" the delay lengths (p.comm.steps) are fixed
%   when HPP.slx is compiled; with "variable" only the buffer size
%   (p.comm.maxDelay) is. Used in the compile signatures of runCase and
%   rapidTarget. p.comm.delays must be resolved first (delayMode).
    if p.comm.delays == "variable"
        s = struct("mode", "variable", "maxDelay", p.comm.maxDelay);
    else
        s = struct("mode", string(p.comm.delays), "steps", p.comm.steps);
    end
end
