function out = runSweep(setName, opts)
%RUNSWEEP  Run all cases of a case set, serially or with parsim.
%
%   out = runSweep()                              nominal set, serial
%   out = runSweep("grid_Pref_Pbess")             all cases of a set, serial
%   out = runSweep("grid_Pref_Pbess", UseParallel=true)   parsim, local pool
%   out = runSweep(..., Save=false)               don't write a results file
%   out = runSweep(..., Network="nodal")          network model: "nodal" (fast)
%                                                 or "simscape"; default
%                                                 p.net.solverType
%   out = runSweep(..., Mode="rapid")             Rapid Accelerator (fastest):
%                                                 the model is compiled once
%                                                 into a program (rapidTarget)
%                                                 and every case runs it with
%                                                 its own p, about 1.5 s per
%                                                 60 s case. Rebuilt only when
%                                                 the layout, time step, delays,
%                                                 network model or a model file
%                                                 change.
%   out = runSweep(..., FastRestart=true)         normal mode, compile once and
%                                                 reuse for all cases (about
%                                                 8-10 s per case). Every
%                                                 parameter is tunable except
%                                                 the layout (units per plant),
%                                                 the time step, the
%                                                 communication delays and the
%                                                 network model.
%
%   out(k) is the output of case k. Results go to results/<setName>/sweep.mat
%   with out, cases and setName. listCaseSets() shows the available sets.
%
%   Serial is the default. With Rapid Accelerator a parallel pool is slower,
%   not faster: each run takes ~1.5 s and parsim's overhead per run is larger
%   (16 cases: 24 s serial, 101 s on 3 workers). A pool only pays off for long
%   runs (Simscape network, long stop times). Close a pool after use:
%   delete(gcp('nocreate')).
%
%   For many cases across many nodes, use the HPC job array instead
%   (hpc/submit.sh <setName>), which calls runCase once per job.

    arguments
        setName (1,1) string = "nominal"
        opts.UseParallel (1,1) logical = false
        opts.Save (1,1) logical = true
        opts.Network (1,1) string {mustBeMember(opts.Network, ["", "nodal", "simscape"])} = ""
        opts.FastRestart (1,1) logical = false
        opts.Mode (1,1) string {mustBeMember(opts.Mode, ["normal", "rapid"])} = "normal"
    end
    rapid = opts.Mode == "rapid";
    if rapid && opts.FastRestart
        error("runSweep:mode", "Mode=""rapid"" and FastRestart exclude each other; use one of them.");
    end

    cases = defineCases(setName);
    if opts.FastRestart || (rapid && opts.UseParallel)
        checkTunable(cases, setName);
    end
    pBase = HPP_parameters();
    if opts.Network ~= "", pBase.net.solverType = opts.Network; end
    checkModelBuild(pBase);

    in(height(cases)) = Simulink.SimulationInput("HPP");
    ps = cell(height(cases), 1);
    for k = 1:height(cases)
        p = applyCase(pBase, cases(k, :));
        ps{k} = p;
        in(k) = Simulink.SimulationInput("HPP");
        in(k) = in(k).setVariable("p", p);
        in(k) = in(k).setModelParameter("StopTime", num2str(p.sim.Tstop));
        if rapid
            % network model and other compile-time values are in the target
            in(k) = in(k).setModelParameter("SimulationMode", "rapid-accelerator", ...
                "RapidAcceleratorUpToDateCheck", "off");
        else
            in(k) = in(k).setVariable("HPP_network", networkCode(p.net.solverType), "Workspace", "HPP");
        end
    end

    if rapid
        if opts.UseParallel
            rapidTarget(ps{1});                      % one target for all cases (checkTunable)
            out = parsim(in, "ShowProgress", "on", "TransferBaseWorkspaceVariables", "off");
        else
            % per case: rapidTarget rebuilds only if this case changes a
            % compile-time value (layout, time step, delays, network model)
            out = Simulink.SimulationOutput.empty;
            t = tic;
            for k = 1:numel(in)
                rapidTarget(ps{k});
                out(k) = sim(in(k), "ShowSimulationManager", "off");
                fprintf("[%s] case %d/%d done (%.0f s)\n", setName, k, numel(in), toc(t));
            end
        end
        saveSweep(opts.Save, out, cases, setName);
        return
    end

    if opts.FastRestart
        % the communication delays (whole steps) are fixed when the model is
        % compiled; they are equal in all cases (checkTunable), so the
        % warnings that p changed in non-tunable parameters are expected
        ids = ["Simulink:Engine:NonTunableVarChangedInFastRestart", ...
               "Simulink:Engine:NonTunableVarChangedMaxWarnings"];
        old = arrayfun(@(i) warning("off", i), ids);
        restore = onCleanup(@() warning(old));
    end
    if opts.UseParallel
        % parsim always uses a parallel pool (starts one if needed)
        out = parsim(in, "ShowProgress", "on", "UseFastRestart", onOff(opts.FastRestart), ...
                         "TransferBaseWorkspaceVariables", "off");
    else
        out = sim(in, "ShowProgress", "on", "UseFastRestart", onOff(opts.FastRestart));   % serial
    end

    saveSweep(opts.Save, out, cases, setName);
end

function saveSweep(doSave, out, cases, setName)
    if ~doSave, return, end
    folder = fullfile(currentProject().RootFolder, "results", setName);
    if ~isfolder(folder)
        mkdir(folder);
    end
    f = fullfile(folder, "sweep.mat");
    save(f, "out", "cases", "setName");
    fprintf("Saved %s\n", f);
end

function s = onOff(b)
    if b, s = "on"; else, s = "off"; end
end

function checkTunable(cases, setName)
    % Fast Restart keeps the compiled model: values that are fixed at
    % compile time (layout, time step, communication delays, network model)
    % would silently keep their first value. Refuse case sets that change them.
    exact  = ["net.solverType", "sim.Ts"];
    prefix = "comm.";
    suffix = [".nFeeders", ".nPerFeeder", ".N"];
    cols = string(cases.Properties.VariableNames);
    bad = cols(arrayfun(@(c) any(c == exact) || startsWith(c, prefix) || any(endsWith(c, suffix)), cols));
    if ~isempty(bad)
        error("runSweep:notTunable", "Case set ""%s"" changes %s, which is fixed when the model is " + ...
            "compiled. Run it without FastRestart.", setName, strjoin(bad, ", "));
    end
end
