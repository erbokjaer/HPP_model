function restore = quietFastRestart()
%QUIETFASTRESTART  Silence the expected Fast Restart warnings for a while.
%
%   restore = quietFastRestart() switches off the warnings that p changed
%   in non-tunable parameters; they are expected, since p also holds the
%   compile-time values, which are equal to the compiled ones (checked by
%   compileSignature / checkTunable). The warnings come back when restore
%   is cleared (end of the calling function).

    ids = ["Simulink:Engine:NonTunableVarChangedInFastRestart", ...
           "Simulink:Engine:NonTunableVarChangedMaxWarnings"];
    old = arrayfun(@(i) warning("off", i), ids);
    restore = onCleanup(@() warning(old));
end
