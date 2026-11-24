function T = summarizeResults(source, caseId, opts)
%SUMMARIZERESULTS  Table of final, minimum and maximum value per signal and case.
%
%   T = summarizeResults("grid_Pref_Pbess")       all saved cases of a set
%   T = summarizeResults("handpicked", "noBess")  one saved case
%   T = summarizeResults(out, Cases=cases)        output of runCase / runSweep
%   T = summarizeResults(..., Signals=["SOC"])    choose the signals
%
%   One row per case (row names from caseLabels). For each signal the
%   columns <signal>_final, <signal>_min and <signal>_max. P in MW, Q in
%   Mvar, V and SOC in pu. Failed runs get NaN.

    arguments
        source
        caseId = []
        opts.Cases table = table()
        opts.Signals (1,:) string = ["P_poc", "P_bess_set", "SOC"]
    end

    [out, cases] = loadResults(source, caseId);
    if ~isempty(opts.Cases)
        cases = opts.Cases;
    end
    if height(cases) == numel(out)
        labels = caseLabels(cases);
    else
        labels = "case " + string(1:numel(out))';
    end

    n = numel(out);
    names = strings(1, 0);
    data = zeros(n, 0);
    for sig = opts.Signals
        vals = NaN(n, 3);
        for k = 1:n
            if ~isempty(out(k).ErrorMessage)
                continue
            end
            [~, d] = resultSignal(out(k), sig, true);   % in the units of signalUnit
            vals(k, :) = [d(end), min(d), max(d)];
        end
        names = [names, sig + ["_final", "_min", "_max"]]; %#ok<AGROW>
        data = [data, vals]; %#ok<AGROW>
    end

    T = array2table(data, "VariableNames", names, "RowNames", matlab.lang.makeUniqueStrings(labels));
end
