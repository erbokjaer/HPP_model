function files = plotAllResults(setNames, opts)
%PLOTALLRESULTS  Plot every saved case set, and save the figures to a folder.
%
%   plotAllResults()                          all sets in results/, saved in
%                                             the background (no windows)
%   plotAllResults(["handpicked","wt_frt"])   only these sets
%   plotAllResults(..., Mode="save")          background: invisible figures,
%                                             saved, then closed (default)
%   plotAllResults(..., Mode="showAndSave")   docked figures stay open, and
%                                             are saved too
%   plotAllResults(..., Mode="show")          docked figures, nothing saved
%   plotAllResults(..., Folder="figs")        where to save (default
%                                             results/figures)
%   plotAllResults(..., Format=["png","pdf"]) file types: png, pdf, svg, fig
%   plotAllResults(..., Signals=[...])        signals to plot (default below;
%                                             signals a set did not log are
%                                             left out of that set's figure)
%   plotAllResults(..., Theme="auto")         figure theme (default "light",
%                                             what a report wants)
%   plotAllResults(..., Size=[1000 800])      figure size in pixels for
%                                             background figures
%
%   A case set is any folder in results/ with a sweep.mat or case_*.mat
%   file. Each set gives one figure, drawn by plotResults, saved as
%   <Folder>/<set>.<format>. Returns the saved file paths (empty in "show"
%   mode). A set that fails to load or plot is skipped with a warning.
%
%   Example:
%       plotAllResults()                      % refresh results/figures
%       plotAllResults("freq_step", Mode="showAndSave", Format=["png","fig"])

    arguments
        setNames (1,:) string = string.empty
        opts.Mode (1,1) string {mustBeMember(opts.Mode, ["save","showAndSave","show"])} = "save"
        opts.Folder (1,1) string = ""
        opts.Format (1,:) string {mustBeMember(opts.Format, ["png","pdf","svg","fig"])} = "png"
        opts.Signals (1,:) string = ["P_poc", "Q_poc", "V_poc", "P_wpp", "P_spp", "P_bess", "SOC"]
        opts.Theme (1,1) string {mustBeMember(opts.Theme, ["auto","light","dark"])} = "light"
        opts.Size (1,2) double {mustBePositive} = [1000 900]
        opts.Resolution (1,1) double {mustBePositive} = 200
    end

    resultsFolder = fullfile(currentProject().RootFolder, "results");
    if isempty(setNames)
        setNames = savedSets(resultsFolder);
    end
    if isempty(setNames)
        fprintf("No saved results in %s.\n", resultsFolder);
        files = string.empty;
        return
    end

    doSave = opts.Mode ~= "show";
    folder = opts.Folder;
    if folder == ""
        folder = fullfile(resultsFolder, "figures");
    end
    if doSave && ~isfolder(folder)
        mkdir(folder);
    end

    files = string.empty;
    for s = 1:numel(setNames)
        name = setNames(s);
        try
            [out, cases] = loadResults(name);
            signals = loggedSignals(out, opts.Signals);
            if isempty(signals)
                warning("plotAllResults:noSignals", ...
                        "%s: none of the requested signals is logged, skipped.", name);
                continue
            end

            if opts.Mode == "save"
                % Background: an invisible figure that is never shown
                fig = figure("Visible", "off", "Units", "pixels", ...
                             "Position", [100 100 opts.Size]);
                cleanup = onCleanup(@() close(fig));
                plotResults(out, Cases=cases, Signals=signals, Theme=opts.Theme, ...
                            Title=name, Parent=fig);
            else
                % Docked figure, one per set (redrawn if it is already open)
                fig = plotResults(out, Cases=cases, Signals=signals, Theme=opts.Theme, ...
                                  Title=name);
            end

            if doSave
                for f = opts.Format
                    file = fullfile(folder, name + "." + f);
                    saveFigure(fig, file, f, opts.Resolution);
                    files(end+1, 1) = file; %#ok<AGROW>
                end
                fprintf("%-18s %d case(s) -> %s\n", name, numel(out), ...
                        strjoin(name + "." + opts.Format, ", "));
            else
                fprintf("%-18s %d case(s)\n", name, numel(out));
            end
            clear cleanup   % closes the background figure
        catch err
            warning("plotAllResults:failed", "%s: %s", name, err.message);
        end
    end
    if doSave
        fprintf("Saved %d file(s) in %s\n", numel(files), folder);
    end
end

function names = savedSets(resultsFolder)
    % Folders in results/ that hold saved runs (sweep.mat or case_*.mat).
    d = dir(resultsFolder);
    d = d([d.isdir] & ~startsWith({d.name}, "."));
    names = string.empty(1, 0);
    for k = 1:numel(d)
        f = fullfile(resultsFolder, d(k).name);
        if isfile(fullfile(f, "sweep.mat")) || ~isempty(dir(fullfile(f, "case_*.mat")))
            names(end+1) = string(d(k).name); %#ok<AGROW>
        end
    end
end

function signals = loggedSignals(out, wanted)
    % The wanted signals that every successful run logged, in order (runs
    % of one set saved at different times may log different signals).
    ok = find(arrayfun(@(o) isempty(o.ErrorMessage), out));
    if isempty(ok)
        error("plotAllResults:allFailed", "every run of the set failed");
    end
    signals = wanted;
    for k = ok(:)'
        signals = signals(ismember(signals, string(out(k).yout.getElementNames)));
    end
end

function saveFigure(fig, file, format, resolution)
    switch format
        case "fig"
            % A background figure is saved invisible; make it show when opened
            if fig.Visible == "off"
                fig.CreateFcn = "set(gcbo, 'Visible', 'on')";
            end
            savefig(fig, file);
        case "png"
            exportgraphics(fig, file, "Resolution", resolution);
        otherwise   % pdf, svg: vector
            exportgraphics(fig, file, "ContentType", "vector");
    end
end
