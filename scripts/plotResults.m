function fig = plotResults(source, caseId, opts)
%PLOTRESULTS  Time plots of the logged signals, one line per case.
%
%   plotResults("handpicked")                 all saved cases of a set
%   plotResults("handpicked", "noBess")       one saved case
%   plotResults(out)                          output of runCase / runSweep
%   plotResults(out, Cases=cases)             ... labelled with a case table
%   plotResults(..., Signals=["P_poc","SOC"]) choose the signals
%   plotResults(..., Parent=fig)              draw into an existing figure
%   plotResults(..., Theme="light")           light figure, e.g. for reports
%                                             (default "auto": MATLAB's theme)
%   plotResults(..., NewFigure=true)          keep the old figure, open a new one
%
%   One tile per signal, stacked with a shared time axis. Powers are shown
%   in MW. Lines are labelled with caseLabels and coloured with dtuColors
%   (the DTU design guide palette; lighter tints on a dark figure).
%   Returns the figure handle.
%
%   Figures are always docked. Plotting the same thing again redraws the
%   existing figure instead of opening a new one. "The same thing" is the
%   figure title: by default the case set name, so every plot of
%   "handpicked" lands in the figure named "handpicked". Give a different
%   Title to keep two plots of one set side by side.
%
%   Example:
%       out = runSweep("oat_Pref");
%       plotResults(out, Cases=defineCases("oat_Pref"))

    arguments
        source
        caseId = []
        opts.Cases table = table()
        opts.Signals (1,:) string = ["P_poc", "P_bess_set", "SOC"]
        opts.Parent = []
        opts.Title (1,1) string = ""
        opts.NewFigure (1,1) logical = false
        opts.Theme (1,1) string {mustBeMember(opts.Theme, ["auto","light","dark"])} = "auto"
    end

    [out, cases] = loadResults(source, caseId);
    if ~isempty(opts.Cases)
        cases = opts.Cases;
    end
    labels = runLabels(out, cases);

    key = figureKey(source, opts.Title);
    if isempty(opts.Parent)
        fig = reuseOrCreateFigure(key, opts.NewFigure);
    else
        fig = opts.Parent;
        clf(fig);
    end
    applyTheme(fig, opts.Theme);
    lineColors = dtuColors(figureStyle(fig));   % only the lines; background follows the theme
    t = tiledlayout(fig, numel(opts.Signals), 1, "TileSpacing", "compact", "Padding", "compact");

    ok = arrayfun(@(o) isempty(o.ErrorMessage), out);
    if any(~ok)
        warning("plotResults:failedRuns", "Skipping %d failed run(s): %s", ...
                nnz(~ok), strjoin(labels(~ok), ", "));
    end

    ax = gobjects(numel(opts.Signals), 1);
    for s = 1:numel(opts.Signals)
        sig = opts.Signals(s);
        ax(s) = nexttile(t);
        ax(s).ColorOrder = lineColors;             % DTU design guide colours
        ax(s).LineStyleOrder = ["-", "--", ":"];   % dashed once the colours run out
        hold(ax(s), "on");
        for k = find(ok(:))'
            [time, data] = getSignal(out(k), sig);
            plot(ax(s), time, data * unitScale(sig), "LineWidth", 1.2, ...
                 "DisplayName", labels(k));
        end
        hold(ax(s), "off");
        grid(ax(s), "on");
        box(ax(s), "on");
        ylabel(ax(s), sig + " [" + unitName(sig) + "]", "Interpreter", "none");
        if s < numel(opts.Signals)
            ax(s).XTickLabel = [];
        end
    end
    xlabel(ax(end), "Time [s]");
    linkaxes(ax, "x");

    if nnz(ok) > 1
        lg = legend(ax(1), "Interpreter", "none");
        if nnz(ok) > 6
            lg.Layout.Tile = "east";
        else
            lg.Location = "best";
        end
    end

    if key ~= ""
        title(t, key, "Interpreter", "none");
    end
    drawnow   % finish rendering so the figure can be exported right away
end

function key = figureKey(source, titleText)
    % What identifies "the same plot": the title, else the case set name.
    if titleText ~= ""
        key = titleText;
    elseif isstring(source) || ischar(source)
        key = string(source);
    else
        key = "";
    end
end

function fig = reuseOrCreateFigure(key, forceNew)
    tag = "plotResults:" + key;
    fig = gobjects(0);
    if ~forceNew
        fig = findobj(groot, "Type", "figure", "Tag", tag);
    end
    if isempty(fig)
        name = key;
        if name == ""
            name = "plotResults";
        end
        fig = figure("WindowStyle", "docked", "Tag", tag, ...
                     "Name", name, "NumberTitle", "off");
    else
        fig = fig(1);
        clf(fig);
        figure(fig);                 % bring its tab to the front
    end
end

function style = figureStyle(fig)
    % "light" or "dark", from the figure's current theme.
    style = "light";
    try
        if string(fig.Theme.BaseColorStyle) == "dark"
            style = "dark";
        end
    catch
        % no theme information: assume a light figure
    end
end

function applyTheme(fig, themeName)
    if themeName ~= "auto"
        theme(fig, themeName);
    else
        try
            theme(fig, "auto");      % undo a light/dark choice on a reused figure
        catch
            % older releases: a new figure already follows MATLAB's theme
        end
    end
end

function labels = runLabels(out, cases)
    if height(cases) == numel(out)
        labels = caseLabels(cases);
    else
        labels = "case " + string(1:numel(out))';
    end
end

function [time, data] = getSignal(o, sig)
    logged = string(o.yout.getElementNames);
    if ~ismember(sig, logged)
        error("plotResults:noSignal", "Signal ""%s"" is not logged. Logged signals: %s", ...
              sig, strjoin(logged, ", "));
    end
    v = o.yout.get(char(sig)).Values;
    time = v.Time;
    data = squeeze(v.Data);
end

function k = unitScale(sig)
    if startsWith(sig, "P_")
        k = 1e-6;
    else
        k = 1;
    end
end

function u = unitName(sig)
    if startsWith(sig, "P_")
        u = "MW";
    else
        u = "-";
    end
end
