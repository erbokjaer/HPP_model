%% HPP_model - main: run and plot selected cases
%  Edit the settings below, then press Run (or F5).
%  listCaseSets() shows the available case sets; defineCases("<set>") shows
%  the cases in a set with their numbers and names.
tic
% Settings
setName  = "nominal";                 % case set in scripts/+caseSets, or "all": every set
                                      % (runSweep per set, then plotAllResults below)
caseIds  = [];                        % names or numbers, e.g. [1 4 7]; [] = all cases (ignored for "all")
rerun    = true;                      % false = reuse saved results where they exist
mode     = "rapid";                   % "rapid": compiled once, ~1.5-2 s per 60 s case (fastest);
                                      % "normal": for debugging with scopes inside the models
fastRestart = true;                   % normal mode only: keep the model compiled between runs
network  = "nodal";                   % "nodal" (fast) or "simscape" (Simscape Electrical, ~5x slower)
delays   = "fixed";                    % "fixed" (fastest), "variable" (delay sweeps without rebuilding)
                                      % or "auto" (variable if the case set changes a delay)
signals  = ["P_poc", "Q_poc", "V_poc", "P_wpp", "P_spp", "P_bess", "SOC"];
theme    = "auto";                    % "auto" (MATLAB theme), "light" or "dark"
exportTo = "";                        % e.g. "figures/handpicked.png"; "" = no export
% Plotting for setName = "all" (plotAllResults)
plotPurpose = "debug";                % "debug": every signal of every set (results/figures/debug);
                                      % "report": the figures of reportFigures.m (results/figures/report);
                                      % "both"; "none": no plots
plotMode    = "show";          % "save" (in the background), "showAndSave" or "show"

if setName == "all"
    %% Run every case set (or load), then plotAllResults
    sets = listCaseSets();
    root = currentProject().RootFolder;
    for s = sets(:)'
        if ~rerun && isfile(fullfile(resultsFolder(s), "sweep.mat"))
            fprintf("Kept saved results of %s\n", s);
            continue
        end
        try
            runSweep(s, Mode=mode, Network=network, Delays=delays, ...
                FastRestart=fastRestart && mode == "normal");
        catch err
            % e.g. a set that changes compile-time values cannot use Fast Restart
            fprintf(2, "%s: %s\n", s, err.message);
        end
    end
    toc
    purposes = string(plotPurpose);
    if purposes == "both", purposes = ["debug", "report"]; end
    if purposes ~= "none"
        for pp = purposes
            plotAllResults(Purpose=pp, Mode=plotMode);
        end
    end
else
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
        out(k) = runCase(setName, id, Network=network, Mode=mode, FastRestart=fastRestart, Delays=delays);
    end

    % Summary
    summary = summarizeResults(out, Cases=cases, Signals=signals);
    disp(summary)
    toc

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
end
