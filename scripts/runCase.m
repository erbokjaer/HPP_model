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
%                                             when there is no C compiler, with
%                                             the Simscape network, when
%                                             checkModels finds block parameters
%                                             compiled code cannot use, or when
%                                             the build fails (see rapidTarget).
%   runCase(..., Network="nodal")             network model for this run:
%                                             "nodal" (fast nodal solver) or
%                                             "simscape" (Simscape Electrical,
%                                             about 5 times slower) or "dq"
%                                             (state-space network with its
%                                             electromagnetic dynamics);
%                                             default p.net.solverType
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
        opts.Network (1,1) string {mustBeMember(opts.Network, ["", "nodal", "simscape", "dq"])} = ""
        opts.FastRestart (1,1) logical = false
        opts.Delays (1,1) string {mustBeMember(opts.Delays, ["", "auto", "fixed", "variable"])} = ""
    end
    persistent lastSignature     % compile-time values of the model kept compiled

    setName = string(setName);
    cases = defineCases(setName);
    [k, caseName] = selectCase(cases, caseId, setName);

    p = applyCase(HPP_parameters(), cases(k, :));
    if opts.Network ~= "", p.net.solverType = opts.Network; end
    [p, delayCode] = delayMode(p, cases, opts.Delays);
    checkModelBuild(p);
    fprintf("Running %s, case %d%s, %s network, %s delays\n", setName, k, nameSuffix(caseName), ...
        p.net.solverType, p.comm.delays);

    % Rapid Accelerator if possible; otherwise (no compiler, Simscape network,
    % checkModels findings, failed build: rapidTarget warns why) normal mode
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
        in = in.setVariable("HPP_network", networkCode(p.net.solverType), "Workspace", "HPP");
        in = in.setVariable("HPP_delays", delayCode, "Workspace", "HPP");
    end

    if useFR && mode == "normal"
        load_system("HPP");
        sig = compileSignature(p);
        if ~isequal(sig, lastSignature) && strcmp(get_param("HPP", "FastRestart"), "on")
            set_param("HPP", "FastRestart", "off");      % compile-time values changed: recompile
        end
        set_param("HPP", "FastRestart", "on");
        lastSignature = sig;
        % p also contains compile-time values; they are equal to the compiled
        % ones here (checked above), so these warnings are expected
        ids = ["Simulink:Engine:NonTunableVarChangedInFastRestart", ...
               "Simulink:Engine:NonTunableVarChangedMaxWarnings"];
        old = arrayfun(@(i) warning("off", i), ids);
        restore = onCleanup(@() warning(old));
    elseif mode ~= "rapid" && bdIsLoaded("HPP") && strcmp(get_param("HPP", "FastRestart"), "on")
        set_param("HPP", "FastRestart", "off");
        lastSignature = [];
    end

    out = sim(in);

    if opts.Save
        folder = fullfile(currentProject().RootFolder, "results", setName);
        if ~isfolder(folder)
            mkdir(folder);
        end
        caseInfo = struct("set", setName, "index", k, "name", caseName, ...
                          "row", cases(k, :), "mode", mode);
        f = fullfile(folder, sprintf("case_%03d.mat", k));
        save(f, "out", "p", "caseInfo");
        fprintf("Saved %s\n", f);
    end
end

function [k, caseName] = selectCase(cases, caseId, setName)
    rowNames = string(cases.Properties.RowNames);
    n = height(cases);

    if isempty(caseId)
        if n ~= 1
            error("runCase:noCase", ...
                  "Case set ""%s"" has %d cases. Give one, e.g. runCase(""%s"", 1).", ...
                  setName, n, setName);
        end
        k = 1;
    elseif isnumeric(caseId)
        if ~isscalar(caseId) || caseId ~= round(caseId) || caseId < 1 || caseId > n
            error("runCase:badIndex", "Case set ""%s"" has cases 1-%d.", setName, n);
        end
        k = caseId;
    else
        k = find(rowNames == string(caseId), 1);
        if isempty(k)
            error("runCase:badName", "Case set ""%s"" has no case named ""%s"". Names: %s", ...
                  setName, string(caseId), strjoin(rowNames, ", "));
        end
    end

    if isempty(rowNames)
        caseName = "";
    else
        caseName = rowNames(k);
    end
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

function m = modeName(mode)
    m = struct("normal","normal","accelerator","accelerator","rapid","rapid-accelerator").(mode);
end

function sig = compileSignature(p)
    % what is fixed when HPP.slx is compiled: the layout (number of units),
    % the time step, the delay mode with the delays (fixed) or the size of
    % the delay buffers (variable), and the network model. All other values
    % are tunable (network data, controllers, grid events).
    sig = struct("N", [p.wt.N, p.pv.N, p.bess.N], "Ts", p.sim.Ts, ...
        "delays", delaySignature(p), "network", p.net.solverType);
end
