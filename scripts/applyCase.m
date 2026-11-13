function p = applyCase(p, caseRow)
%APPLYCASE  Overwrite fields of p with the values in one row of a case table.
%   NaN means "keep the default", so a row of all NaN leaves p unchanged.
%   Each column must be an existing field path in p (catches typos).
%   Derived parameters (e.g. p.ctrl limits) are rebuilt afterwards so they
%   stay consistent with the overridden component values.

    for name = string(caseRow.Properties.VariableNames)
        v = caseRow.(name);
        if iscell(v)
            v = v{1};
        end
        if isnumeric(v) && isscalar(v) && isnan(v)
            continue
        end

        path = split(name, ".");
        if ~isFieldPath(p, path)
            error("applyCase:unknownField", ...
                  "Case column ""%s"" is not an existing field of p.", name);
        end
        p = setfield(p, path{:}, v);
    end
    p.ctrl = controllerParameters(p);
end

function tf = isFieldPath(s, path)
    tf = true;
    for k = 1:numel(path)
        if ~isstruct(s) || ~isfield(s, path{k})
            tf = false;
            return
        end
        s = s.(path{k});
    end
end
