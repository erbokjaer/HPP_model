%% HPP_model - main: run and plot selected cases
%  Edit the settings below, then press Run (or F5).
%  listCaseSets() shows the available case sets; defineCases("<set>") shows
%  the cases in a set with their numbers and names.

%% Settings
setName  = "handpicked";              % case set in scripts/+caseSets
caseIds  = ["nominal", "noBess"];     % names or numbers, e.g. [1 4 7]; [] = all cases
rerun    = true;                      % false = reuse saved results where they exist
signals  = ["P_poc", "P_bess_set", "SOC"];
theme    = "auto";                    % "auto" (MATLAB theme), "light" or "dark"
exportTo = "";                        % e.g. "figures/handpicked.png"; "" = no export

%% Select cases
allCases = defineCases(setName);
if isempty(caseIds)
    caseIds = 1:height(allCases);
end
cases = allCases(caseIds, :);
fprintf("\n%s: %d case(s)\n", setName, height(cases));

%% Run (or load)
out = Simulink.SimulationOutput.empty;
for k = 1:height(cases)
    id = caseIds(k);
    if ~rerun
        try
            out(k) = loadResults(setName, id);
            fprintf("Loaded %s, case %s\n", setName, string(id));
            continue
        catch
            % no saved result for this case: run it below
        end
    end
    out(k) = runCase(setName, id);
end

%% Plot
fig = plotResults(out, Cases=cases, Signals=signals, Theme=theme, Title=setName);
if exportTo ~= ""
    folder = fileparts(exportTo);
    if folder ~= "" && ~isfolder(folder)
        mkdir(folder);
    end
    exportgraphics(fig, exportTo, Resolution=300);
    fprintf("Exported %s\n", exportTo);
end

%% Summary
summary = summarizeResults(out, Cases=cases, Signals=signals);
disp(summary)
