function out = runCase(setName, caseId, opts)
%RUNCASE  Run a single case in batch mode (no GUI). HPC entry point.
%
%   runCase()                                 nominal case (project default)
%   runCase("handpicked", "noBess")           case by row name
%   runCase("grid_Pref_Pbess", 4)             case by row number
%   runCase("oat_Pref", 2, Save=false)        don't write a results file
%   runCase(..., Mode="rapid")                Rapid Accelerator, the fastest: the
%                                             model is compiled once into a
%                                             program (rapidTarget, ~25-50 s,
%                                             MinGW) and each run takes ~1.5-2 s
%                                             for 60 s. Parameters are tunable;
%                                             it is rebuilt only when the
%                                             layout, time step, delay mode
%                                             (and with fixed delays the
%                                             delays), network model or a model
%                                             file change. FastRestart does not
%                                             apply.
%                                             Falls back to normal mode with Fast
%                                             Restart (warning with the reason)
%                                             when there is no C compiler, when
%                                             checkModels finds block parameters
%                                             compiled code cannot use, or when
%                                             the build fails (see rapidTarget).
%   runCase(..., Network="nodal")             network model for this run:
%                                             "nodal" (fast nodal solver) or
%                                             "simscape" (Simscape Electrical);
%                                             default p.net.solverType
%   runCase(..., SimscapeMode="emt")          Simscape network mode: "phasor"
%                                             or "emt" (time domain, step
%                                             p.net.emtTs); default
%                                             p.net.simscapeMode. No effect
%                                             with the nodal network.
%   runCase(..., Converter="averaged")        converter model of all assets:
%                                             "source" (current source, RMS)
%                                             or "averaged" (current loop,
%                                             LCL filter, DC link); default
%                                             p.conv.type (tunable, no rebuild)
%   runCase(..., Delays="variable")           how the communication delays
%                                             are compiled: "fixed" (fastest),
%                                             "variable" (tunable, for delay
%                                             sweeps) or "auto" (variable if
%                                             the case set changes a delay);
%                                             default p.comm.delays ("auto"),
%                                             see commParameters
%   runCase(..., FastRestart=true)            keep the model compiled between
%                                             runs in this MATLAB session: a
%                                             rerun skips the compile (about
%                                             20-25 s). Parameter values are
%                                             tunable (controllers, grid and
%                                             network data, events); it
%                                             recompiles by itself only when
%                                             the layout, time step, delay
%                                             mode (with fixed delays: the
%                                             delays) or network model differ
%                                             from the previous run.
%
%   runCase(..., Window=[1 Inf])              event window: start 1 s before
%                                             the earliest event of the case
%                                             set from a saved operating point
%                                             and run to the stop time ([pre
%                                             post]: stop post s after the
%                                             event). The operating point is
%                                             made once (a run from 0) and
%                                             cached per build and pre-event
%                                             condition (operatingPoints,
%                                             eventWindow). Event times stay
%                                             absolute; the results start at
%                                             the window start.
%
%   Results go to results/<setName>/case_NNN.mat with out, p and caseInfo.
%   listCaseSets() shows the available case sets.
%
%   From a shell / HPC job:
%       matlab -batch "openProject('HPP_model.prj'); runCase('grid_Pref_Pbess', 3)"

    arguments
        setName {mustBeCaseSetName} = "nominal"
        caseId = []
        opts.Mode (1,1) string {mustBeMember(opts.Mode, ["normal","accelerator","rapid"])} = "normal"
        opts.Save (1,1) logical = true
        opts.Network (1,1) string {mustBeMember(opts.Network, ["", "nodal", "simscape"])} = ""
        opts.FastRestart (1,1) logical = false
        opts.Delays (1,1) string {mustBeMember(opts.Delays, ["", "auto", "fixed", "variable"])} = ""
        opts.Window double {mustBeWindow} = []
        opts.SimscapeMode (1,1) string {mustBeMember(opts.SimscapeMode, ["", "phasor", "emt"])} = ""
        opts.Converter (1,1) string {mustBeMember(opts.Converter, ["", "source", "averaged"])} = ""
    end
    persistent lastSignature     % compile-time values of the model kept compiled

    setName = string(setName);
    cases = defineCases(setName);
    [k, caseName] = selectCase(cases, caseId, setName);

    p = applyCase(HPP_parameters(), cases(k, :));
    if opts.Network ~= "", p.net.solverType = opts.Network; end
    if opts.SimscapeMode ~= "", p.net.simscapeMode = opts.SimscapeMode; end
    if opts.Converter ~= "", p.conv.type = find(opts.Converter == ["source", "averaged"]); end
    p = deriveParameters(p);
    [p, delayCode] = delayMode(p, cases, opts.Delays);
    checkModelBuild(p);
    fprintf("Running %s, case %d%s, %s, %s delays\n", setName, k, nameSuffix(caseName), ...
        networkLabel(p), p.comm.delays);

    % Rapid Accelerator if possible; otherwise (no compiler, checkModels
    % findings, failed build: rapidTarget warns why) normal mode
    % with Fast Restart
    mode = opts.Mode;  useFR = opts.FastRestart;
    if mode == "rapid"
        st = rapidTarget(p);
        if ~st.ok
            mode = "normal";  useFR = true;
            fprintf("Normal mode (Fast Restart) instead of Rapid Accelerator.\n");
        end
    end

    in = Simulink.SimulationInput("HPP");
    in = in.setVariable("p", p);
    in = in.setModelParameter("SimulationMode", modeName(mode), ...
                              "StopTime", num2str(p.sim.Tstop));
    if mode == "rapid"
        % compiled program, built once per layout / model change (rapidTarget);
        % the network model and other compile-time values are in the target
        in = in.setModelParameter("RapidAcceleratorUpToDateCheck", "off");
    else
        in = setVariants(in, p, delayCode);              % network, delays, PLL
    end

    if useFR && mode == "normal"
        load_system("HPP");
        sig = compileSignature(p);
        if ~isequal(sig, lastSignature)
            fastRestartOff();                            % compile-time values changed: recompile
        end
        set_param("HPP", "FastRestart", "on");
        lastSignature = sig;
        restore = quietFastRestart(); %#ok<NASGU>
    elseif mode ~= "rapid" && fastRestartOff()
        lastSignature = [];
    end

    win = [];
    if ~isempty(opts.Window)
        % event window: start from the operating point before the set's
        % earliest event (made and cached on first use)
        win = eventWindow(cases, HPP_parameters(), opts.Window);
        in = operatingPoints(in, p, win, mode, FastRestart=useFR && mode == "normal");
    end

    title = setName + " case " + k;  if ~isempty(win), title = title + " (window)"; end
    pr = Progress(title, 0);                            % estimate: this case last time
    if isempty(win)
        pr.step("simulating " + p.sim.Tstop + " s (" + modeName(mode) + ")");
    else
        pr.step("simulating " + win.tOp + "-" + min(win.tEvent + win.post, p.sim.Tstop) + ...
            " s from the operating point (" + modeName(mode) + ")");
    end
    out = sim(in);
    pr.done();

    if opts.Save
        folder = resultsFolder(setName, true);
        caseInfo = struct("set", setName, "index", k, "name", caseName, ...
                          "row", cases(k, :), "mode", mode, "window", win);
        f = fullfile(folder, sprintf("case_%03d.mat", k));
        save(f, "out", "p", "caseInfo");
        fprintf("Saved %s\n", f);
    end
end

function s = networkLabel(p)
    % "nodal network", "simscape phasor network" or "simscape emt network",
    % plus the converter model when it is not the current source
    s = string(p.net.solverType);
    if s == "simscape", s = s + " " + p.net.simscapeMode; end
    s = s + " network";
    if p.conv.type == 2, s = s + ", averaged converter"; end
end

function s = nameSuffix(caseName)
    if caseName == ""
        s = "";
    else
        s = " (" + caseName + ")";
    end
end

function mustBeCaseSetName(x)
    if ~((isstring(x) && isscalar(x)) || (ischar(x) && isrow(x)))
        error("runCase:setName", ...
              "The first argument is the case set name, e.g. runCase(""grid_Pref_Pbess"", 3). " + ...
              "See listCaseSets().");
    end
end

function mustBeWindow(w)
    if ~(isempty(w) || (isnumeric(w) && numel(w) == 2 && all(w >= 0)))
        error("runCase:window", "Window is [pre post] in seconds around the event, e.g. [1 Inf].");
    end
end

function m = modeName(mode)
    m = struct("normal","normal","accelerator","accelerator","rapid","rapid-accelerator").(mode);
end
