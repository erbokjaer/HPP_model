function names = listCaseSets()
%LISTCASESETS  Names of the case sets in scripts/+caseSets.
%   With no output, prints each set with its number of cases.

    w = what("caseSets");
    files = vertcat(w.m);
    [~, n] = fileparts(string(files));
    n = sort(n(:));

    if nargout > 0
        names = n;
        return
    end
    for k = 1:numel(n)
        fprintf("  %-20s %3d case(s)\n", n(k), height(feval("caseSets." + n(k))));
    end
end
