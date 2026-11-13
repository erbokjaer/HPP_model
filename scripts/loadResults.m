function [out, cases] = loadResults(source, caseId)
%LOADRESULTS  Load saved simulation results for a case set or one case.
%
%   [out, cases] = loadResults("handpicked")             all saved cases of a set
%   [out, cases] = loadResults("handpicked", "noBess")   one case, by name
%   [out, cases] = loadResults("grid_Pref_Pbess", 4)     one case, by number
%
%   out is a Simulink.SimulationOutput array and cases the matching rows of
%   the case table. For a whole set, results/<set>/sweep.mat is used if it
%   exists (written by runSweep); otherwise all case_NNN.mat files written
%   by runCase are collected.
%
%   A SimulationOutput passed as source is returned unchanged, with an
%   empty case table. plotResults and summarizeResults use this.

    arguments
        source
        caseId = []
    end

    if isa(source, "Simulink.SimulationOutput")
        out = source;
        cases = table();
        return
    end

    setName = string(source);
    folder = fullfile(currentProject().RootFolder, "results", setName);
    if ~isfolder(folder)
        error("loadResults:noResults", ...
              "No results for ""%s"". Run runSweep(""%s"") or runCase(""%s"", ...) first.", ...
              setName, setName, setName);
    end

    % One case
    if ~isempty(caseId)
        k = caseIndex(setName, caseId);
        f = fullfile(folder, sprintf("case_%03d.mat", k));
        if ~isfile(f)
            error("loadResults:noCase", "No saved result %s. Run runCase(""%s"", %d) first.", ...
                  f, setName, k);
        end
        L = load(f, "out", "caseInfo");
        out = L.out;
        cases = L.caseInfo.row;
        return
    end

    % Whole set: prefer the sweep file
    f = fullfile(folder, "sweep.mat");
    if isfile(f)
        L = load(f, "out", "cases");
        out = L.out;
        cases = L.cases;
        return
    end

    files = dir(fullfile(folder, "case_*.mat"));
    if isempty(files)
        error("loadResults:empty", "Folder %s has no saved results.", folder);
    end
    files = sort(string({files.name}));
    for i = numel(files):-1:1
        L = load(fullfile(folder, files(i)), "out", "caseInfo");
        out(i) = L.out;
        rows{i} = L.caseInfo.row;
    end
    cases = vertcat(rows{:});
end

function k = caseIndex(setName, caseId)
    if isnumeric(caseId)
        k = caseId;
        return
    end
    names = string(defineCases(setName).Properties.RowNames);
    k = find(names == string(caseId), 1);
    if isempty(k)
        error("loadResults:badName", "Case set ""%s"" has no case named ""%s"".", ...
              setName, string(caseId));
    end
end
