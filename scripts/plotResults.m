function fig = plotResults(source, caseId, opts)
%PLOTRESULTS  Time plots of the logged signals, one line per case.
%
%   plotResults("handpicked")                 all saved cases of a set
%   plotResults("handpicked", "noBess")       one saved case
%   plotResults(out)                          output of runCase / runSweep
%   plotResults(out, Cases=cases)             ... labelled with a case table
%   plotResults(..., Signals=["P_poc","SOC"]) choose the signals
%   plotResults(..., Parent=fig)              draw into an existing figure
%   plotResults(..., Docked=false)            separate window (default: docked)
%   plotResults(..., Theme="light")           light figure, e.g. for reports
%                                             (default "auto": MATLAB's theme)
%
%   One tile per signal, stacked with a shared time axis. Powers are shown
%   in MW. Lines are labelled with caseLabels. Returns the figure handle.
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
        opts.Docked (1,1) logical = true
        opts.Theme (1,1) string {mustBeMember(opts.Theme, ["auto","light","dark"])} = "auto"
    end

    [out, cases] = loadResults(source, caseId);
    if ~isempty(opts.Cases)
        cases = opts.Cases;
    end
    labels = runLabels(out, cases);

    if isempty(opts.Parent)
        if opts.Docked
            fig = figure("WindowStyle", "docked");
        else
            fig = figure("WindowStyle", "normal");
        end
    else
        fig = opts.Parent;
        clf(fig);
    end
    if opts.Theme ~= "auto"
        theme(fig, opts.Theme);
    end
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

    if opts.Title ~= ""
        title(t, opts.Title, "Interpreter", "none");
    elseif isstring(source) || ischar(source)
        title(t, string(source), "Interpreter", "none");
    end
    drawnow   % finish rendering so the figure can be exported right away
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
