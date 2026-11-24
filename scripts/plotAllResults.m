function files = plotAllResults(setNames, opts)
%PLOTALLRESULTS  Make the figures of the saved case sets, for a report or for debugging.
%
%   plotAllResults()                           debug figures of every saved set
%   plotAllResults(Purpose="report")           report figures (reportFigures.m)
%   plotAllResults(["freq_step","wt_frt"], ...) only these case sets
%
%   Purpose="debug" (default): every logged signal of every case, one figure
%     per case set, full time plus a zoom on the event where the set has one
%     (<set>_zoom). Readability is secondary. Default folder
%     results/figures/debug, formats png and fig.
%   Purpose="report": one small figure per point a case set makes, as
%     defined in reportFigures.m: few cases and signals, zoomed on the event,
%     sized for one column of a two-column paper (8.8 cm, 8 pt Times).
%     Default folder results/figures/report, formats pdf (vector, for LaTeX)
%     and png (preview). In LaTeX include the PDF at its natural size
%     (width=\columnwidth) so the text stays 8 pt.
%
%   Options:
%     Mode="save"         background: invisible figures, saved, closed (default)
%     Mode="showAndSave"  figures stay open, and are saved
%     Mode="show"         figures open, nothing saved
%     Folder="..."        destination (default per purpose, see above)
%     Format=["pdf","png","svg","fig"]   file types (default per purpose)
%     Width=17.8          report figure width [cm] (default 8.8)
%     FontName, FontSize  report font (default Times New Roman, 8 pt)
%
%   Reads results/<set>/ (run the sets first, e.g. runSweep("freq_step",
%   FastRestart=true)). Returns the saved files.

    arguments
        setNames (1,:) string = string.empty
        opts.Purpose (1,1) string {mustBeMember(opts.Purpose, ["debug", "report"])} = "debug"
        opts.Mode (1,1) string {mustBeMember(opts.Mode, ["save", "showAndSave", "show"])} = "save"
        opts.Folder (1,1) string = ""
        opts.Format (1,:) string {mustBeMember(opts.Format, ["png", "pdf", "svg", "fig"])} = string.empty
        opts.Width (1,1) double {mustBePositive} = 8.8
        opts.FontName (1,1) string = "Times New Roman"
        opts.FontSize (1,1) double {mustBePositive} = 8
    end

    root = currentProject().RootFolder;
    opts.Sets = setNames;
    opts.Show = opts.Mode ~= "save";
    opts.Save = opts.Mode ~= "show";
    if isempty(opts.Format)
        if opts.Purpose == "report", opts.Format = ["pdf", "png"]; else, opts.Format = ["png", "fig"]; end
    end
    folder = opts.Folder;
    if folder == "", folder = fullfile(root, "results", "figures", opts.Purpose); end
    if opts.Save && ~isfolder(folder), mkdir(folder); end

    if opts.Purpose == "report"
        files = reportMode(opts, folder);
    else
        files = debugMode(opts, folder, root);
    end
    if opts.Save
        fprintf("Saved %d file(s) in %s\n", numel(files), folder);
    end
end

%% ---------------------------------------------------------------- report
function files = reportMode(opts, folder)
    F = reportFigures();
    if ~isempty(opts.Sets), F = F(ismember([F.set], opts.Sets)); end
    files = strings(0);
    cache = struct();
    for i = 1:numel(F)
        f = F(i);
        key = matlab.lang.makeValidName(f.set);
        if ~isfield(cache, key)
            [o, c] = loadResults(f.set);
            cache.(key) = struct("out", o, "cases", c);
        end
        R = cache.(key);
        names = caseLabels(R.cases);
        idx = zeros(1, numel(f.cases));
        for k = 1:numel(f.cases)
            j = find(names == f.cases(k), 1);
            if isempty(j)
                error("plotAllResults:noCase", "Figure %s: case ""%s"" is not in the saved results of %s. Rerun the set.", ...
                    f.name, f.cases(k), f.set);
            end
            idx(k) = j;
        end
        nAx = numel(f.axes);
        nLeg = numel(f.cases) + any(cellfun(@(s) any(isReference(s)), f.axes));
        if numel(f.cases) == 1, nLeg = sum(cellfun(@numel, f.axes)); end
        H = 1.2 + 2.5 * nAx + 0.35 * ceil(nLeg / 3) * (nLeg > 1);  % [cm]: margins, axes, legend rows
        fh = newFigure(opts, opts.Width, H, f.name);
        t = tiledlayout(fh, nAx, 1, "TileSpacing", "tight", "Padding", "compact");
        single = numel(idx) == 1;
        C = dtuColors("light");
        ax = gobjects(nAx, 1);
        hLeg = gobjects(0);                     % legend entries, in order
        nSig = 0;                               % signal counter (one case: colour per signal)
        hasRef = false;
        for a = 1:nAx
            ax(a) = nexttile(t);  hold(ax(a), "on");
            sigs = f.axes{a};
            for s = 1:numel(sigs)
                nSig = nSig + 1;
                for k = 1:numel(idx)
                    [tt, v] = resultSignal(R.out(idx(k)), sigs(s));
                    if f.delta
                        v = v - interp1(tt, v, f.t0);
                    end
                    [tt, v] = thin(tt, v, f.xlim);
                    if single
                        col = C(mod(nSig - 1, size(C, 1)) + 1, :);
                    else
                        col = C(mod(k - 1, size(C, 1)) + 1, :);
                    end
                    ref = isReference(sigs(s));
                    ls = "-";  lw = 1.0;
                    if ref, ls = "--";  lw = 0.8;  hasRef = true; end
                    h = plot(ax(a), tt, v, "Color", col, "LineStyle", ls, "LineWidth", lw);
                    if single
                        h.DisplayName = sigLabel(sigs(s));  hLeg(end+1) = h; %#ok<AGROW>
                    elseif ~ref && s == 1 && a == 1
                        h.DisplayName = f.labels(k);  hLeg(end+1) = h; %#ok<AGROW>
                    end
                end
            end
            hold(ax(a), "off");
            styleAxes(ax(a), opts);
            ylabel(ax(a), axisLabel(sigs, f.delta, single));
            if a < nAx, ax(a).XTickLabel = []; end
        end
        xlabel(ax(end), "Time [s]");
        linkaxes(ax, "x");  xlim(ax(1), f.xlim);
        nRows = ceil(numel(hLeg) / 3) * (numel(hLeg) > 1);
        for a = 1:nAx
            fitY(ax(a), (a == 1) * nRows);       % headroom for the legend in the top axis
        end
        % one legend at the top of the top axis, in the headroom left for it
        if ~single && hasRef
            hold(ax(1), "on");
            hLeg(end+1) = plot(ax(1), NaN, NaN, "k--", "LineWidth", 0.8, "DisplayName", "reference");
            hold(ax(1), "off");
        end
        if numel(hLeg) > 1
            lg = legend(ax(1), hLeg);
            set(lg, "FontSize", opts.FontSize - 1, "FontName", opts.FontName, "Box", "off", ...
                "NumColumns", min(numel(hLeg), 3), "ItemTokenSize", [14 8], "AutoUpdate", "off", ...
                "Location", "north");
        end
        files = [files, exportFigure(fh, folder, f.name, opts)]; %#ok<AGROW>
        fprintf("%-24s %s\n", f.name, f.what);
        showOrClose(fh, opts);
    end
end

%% ----------------------------------------------------------------- debug
function files = debugMode(opts, folder, root)
    sets = opts.Sets;
    if isempty(sets)
        d = dir(fullfile(root, "results"));
        d = d([d.isdir] & ~startsWith({d.name}, "."));
        sets = string({d.name});
        sets = sets(arrayfun(@(s) ~isempty(dir(fullfile(root, "results", s, "*.mat"))), sets));
    end
    % every logged signal; references dashed in the case colour
    groups = {["P_poc", "P_ref"], ["Q_poc", "Q_ref"], ["V_poc", "V_bus"], "f", ...
              ["P_wpp", "P_avail_wpp"], ["P_spp", "P_avail_spp"], ["P_bess", "P_bess_set"], "SOC", ...
              "Q_wpp", "Q_spp", "Q_bess"};
    files = strings(0);
    for s = sets
        try
            [out, cases] = loadResults(s);
        catch err
            warning("plotAllResults:skip", "%s: %s", s, err.message);  continue
        end
        labels = caseLabels(cases);
        zoom = eventWindow(s);
        views = {"", []};
        if ~isempty(zoom), views(end+1, :) = {"_zoom", zoom}; end
        for v = 1:size(views, 1)
            fh = newFigure(struct("Show", opts.Show, "FontName", "Helvetica", "FontSize", 9), 42, 26, s + views{v, 1});
            t = tiledlayout(fh, 3, 4, "TileSpacing", "compact", "Padding", "compact");
            C = dtuColors("light");
            ax = gobjects(numel(groups), 1);
            for g = 1:numel(groups)
                ax(g) = nexttile(t);  hold(ax(g), "on");
                for sig = groups{g}
                    for k = 1:numel(out)
                        if ~isempty(out(k).ErrorMessage), continue, end
                        [tt, y] = resultSignal(out(k), sig);
                        if isempty(tt), continue, end
                        win = views{v, 2};  if isempty(win), win = [tt(1) tt(end)]; end
                        [tt, y] = thin(tt, y, win);      % keeps peaks, saves graphics memory
                        col = C(mod(k - 1, size(C, 1)) + 1, :);
                        ls = "-";  if isReference(sig) || sig == "V_bus", ls = "--"; end
                        h = plot(ax(g), tt, y, "Color", col, "LineStyle", ls, "LineWidth", 0.9, "DisplayName", labels(k));
                        if sig ~= groups{g}(1), h.Annotation.LegendInformation.IconDisplayStyle = "off"; end
                    end
                end
                hold(ax(g), "off");  grid(ax(g), "on");  box(ax(g), "on");
                title(ax(g), strjoin(groups{g}, ", ") + " [" + signalUnit(groups{g}(1)) + "]" + ...
                    dashedNote(groups{g}), "Interpreter", "none", "FontWeight", "normal");
            end
            linkaxes(ax, "x");
            if ~isempty(views{v, 2}), xlim(ax(1), views{v, 2}); end
            lg = legend(ax(1), "Interpreter", "none");  lg.Layout.Tile = 12;
            title(t, s + views{v, 1} + "  (results/" + s + ")", "Interpreter", "none");
            files = [files, exportFigure(fh, folder, s + views{v, 1}, opts)]; %#ok<AGROW>
            showOrClose(fh, opts);
        end
        fprintf("%-16s %d case(s)\n", s, numel(out));
    end
end

function w = eventWindow(set)
    % zoom window of the debug figures, around the event of the set
    switch set
        case "freq_step",  w = [38 50];
        case "hppc_qv",    w = [38 45];
        case "wt_frt",     w = [44.5 47];
        case "hppc_p",     w = [35 100];
        otherwise,         w = [];
    end
end

function n = dashedNote(g)
    if numel(g) > 1, n = "  (dashed: " + strjoin(g(2:end), ", ") + ")"; else, n = ""; end
end

%% ---------------------------------------------------------------- helpers
function showOrClose(fh, opts)
    % the figure is drawn and saved invisible at its set size; afterwards it
    % is docked and shown (Show) or closed
    if opts.Show
        fh.WindowStyle = "docked";
        fh.Visible = "on";
    else
        close(fh);
    end
end

function fh = newFigure(opts, W, H, name)
    % always invisible and undocked while drawing and saving, so the saved
    % size is the set size (a docked figure takes the size of the dock);
    % showOrClose docks it afterwards
    fh = figure("Visible", "off", "Units", "centimeters", "Position", [2 2 W H], "Color", "w", ...
        "Name", name, "NumberTitle", "off", "WindowStyle", "normal");
    try, theme(fh, "light"); catch, end
    set(fh, "DefaultAxesFontName", opts.FontName, "DefaultAxesFontSize", opts.FontSize, ...
        "DefaultTextFontName", opts.FontName, "DefaultLegendFontName", opts.FontName);
end

function styleAxes(ax, opts)
    ax.FontName = opts.FontName;  ax.FontSize = opts.FontSize;
    ax.LineWidth = 0.5;  ax.TickLength = [0.015 0.015];
    grid(ax, "on");  ax.GridAlpha = 0.15;  box(ax, "on");
end

function fitY(ax, legendRows)
    % y range from the data inside the time window, with a margin, plus
    % headroom for legendRows rows of legend at the top
    if nargin < 2, legendRows = 0; end
    lo = inf;  hi = -inf;  xl = xlim(ax);
    for h = findobj(ax, "Type", "line")'
        m = h.XData >= xl(1) & h.XData <= xl(2);
        if any(m), lo = min(lo, min(h.YData(m)));  hi = max(hi, max(h.YData(m))); end
    end
    if isfinite(lo)
        d = max(hi - lo, 1e-3 * max(1, abs(hi)));
        top = hi + 0.10*d;
        if legendRows > 0
            % the legend takes about 0.3 cm per row of the axes height
            ax.Units = "centimeters";  hAx = ax.Position(4);  ax.Units = "normalized";
            f = min(0.6, (0.45 * legendRows + 0.15) / max(hAx, 0.5));
            top = top + f / (1 - f) * (top - (lo - 0.08*d));
        end
        ylim(ax, [lo - 0.08*d, top]);
    end
end

function [t, v] = thin(t, v, xl)
    % crop to the time window and keep at most ~1500 bins (min and max of
    % each bin, in time order), so vector files stay small and peaks stay
    t = t(:);  v = v(:);
    m = t >= xl(1) - 0.02*diff(xl) & t <= xl(2) + 0.02*diff(xl);
    t = t(m);  v = v(m);
    nb = 1500;
    if numel(t) <= 2*nb, return, end
    e = round(linspace(0, numel(t), nb + 1));
    T = zeros(2*nb, 1);  V = T;
    for b = 1:nb
        r = e(b)+1:e(b+1);
        [~, i1] = min(v(r));  [~, i2] = max(v(r));
        ii = sort([i1 i2]) + e(b);
        T(2*b-1:2*b) = t(ii);  V(2*b-1:2*b) = v(ii);
    end
    t = T;  v = V;
end

function files = exportFigure(fh, folder, name, opts)
    files = strings(0);
    if ~opts.Save, return, end
    drawnow
    for fmt = opts.Format
        f = fullfile(folder, name + "." + fmt);
        switch fmt
            case "pdf", exportgraphics(fh, f, "ContentType", "vector");
            case "png", exportgraphics(fh, f, "Resolution", 200);
            case "svg", exportgraphics(fh, f, "ContentType", "vector");
            case "fig"
                if fh.Visible == "off", fh.CreateFcn = "set(gcbo, 'Visible', 'on')"; end
                savefig(fh, f);
        end
        files(end+1) = f; %#ok<AGROW>
    end
end

function r = isReference(sig)
    r = ismember(sig, ["P_ref", "Q_ref", "P_avail_wpp", "P_avail_spp", "P_bess_set"]);
end

function s = sigLabel(sig)
    % TeX name of a signal
    m = dictionary(["P_poc","Q_poc","V_poc","V_bus","P_ref","Q_ref","f","P_wpp","P_spp","P_bess", ...
                    "P_avail_wpp","P_avail_spp","P_bess_set","Q_wpp","Q_spp","Q_bess","SOC","PF"], ...
        ["P_{POC}","Q_{POC}","V_{POC}","V_{bus}","P_{ref}","Q_{ref}","f","P_{wind}","P_{PV}","P_{BESS}", ...
         "P_{wind,avail}","P_{PV,avail}","P_{BESS,set}","Q_{wind}","Q_{PV}","Q_{BESS}","SOC","PF"]);
    if isKey(m, sig), s = m(sig); else, s = sig; end
end

function s = axisLabel(sigs, delta, single)
    % y label: the main signal (or "P" for several powers of one case) and the unit
    u = signalUnit(sigs(1));
    if single && numel(sigs) > 1
        s = extractBefore(sigs(1) + "_", "_") + " [" + u + "]";
    else
        s = sigLabel(sigs(1)) + " [" + u + "]";
    end
    if delta, s = "\Delta" + s; end
end
