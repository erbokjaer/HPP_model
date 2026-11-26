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
network  = "nodal";                   % "nodal" (RMS, fastest) or "simscape" (Simscape Electrical)
simscapeMode = "phasor";              % Simscape network only: "phasor" or "emt" (time domain,
                                      % 50 us step, the first build takes several minutes)
converter = "source";                 % "source": current source (RMS); "averaged": current loop,
                                      % LCL filter and DC link (no rebuild needed)
window   = [];                        % [] = run from t = 0; [pre post] = event window: start pre s
                                      % before the set's first event from a saved operating point,
                                      % stop post s after it (Inf = to the stop time), e.g. [1 Inf]
delays   = "fixed";                   % "fixed" (fastest), "variable" (delay sweeps without rebuilding)
                                      % or "auto" (variable if the case set changes a delay)
signals  = ["P_poc", "Q_poc", "V_poc", "P_wpp", "P_spp", "P_bess", "SOC"];
theme    = "auto";                    % "auto" (MATLAB theme), "light" or "dark"
exportTo = "";                        % e.g. "figures/handpicked.png"; "" = no export
% Plotting for setName = "all" (plotAllResults)
plotPurpose = "debug";                % "debug": every signal of every set (results/figures/debug);
                                      % "report": the figures of reportFigures.m (results/figures/report);
                                      % "both"; "none": no plots
plotMode    = "show";                 % "save" (in the background), "showAndSave" or "show"

runOpts = {"Network", network, "SimscapeMode", simscapeMode, "Converter", converter, ...
           "Delays", delays, "Window", window};

if setName == "all"
    %% Run every case set (or load), then plotAllResults
    sets = listCaseSets();
    for s = sets(:)'
        if ~rerun && isfile(fullfile(resultsFolder(s), "sweep.mat"))
            fprintf("Kept saved results of %s\n", s);
            continue
        end
        try
            runSweep(s, runOpts{:}, Mode=mode, FastRestart=fastRestart && mode == "normal");
        catch err
            % e.g. a set that changes compile-time values cannot use Fast
            % Restart, or a set without an event cannot use the window
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
        out(k) = runCase(setName, id, runOpts{:}, Mode=mode, FastRestart=fastRestart);
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
