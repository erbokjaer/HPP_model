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
%   out = runSweep(..., FastRestart=true)         compile once, reuse for all
%                                                 cases (saves the compile time
%                                                 per case). Every parameter is
%                                                 tunable except the layout
%                                                 (units per plant), the time
%                                                 step, the communication
%                                                 delays and the network model.
%
%   out(k) is the output of case k. Results go to results/<setName>/sweep.mat
%   with out, cases and setName. listCaseSets() shows the available sets.
%
%   Serial is the default because each pool worker uses ~1 GB of RAM, and on
%   a 16 GB laptop starting a pool next to the desktop session has crashed
%   MATLAB. Close the pool after use: delete(gcp('nocreate')).
%
%   For many cases across many nodes, use the HPC job array instead
%   (hpc/submit.sh <setName>), which calls runCase once per job.

    arguments
        setName (1,1) string = "nominal"
        opts.UseParallel (1,1) logical = false
        opts.Save (1,1) logical = true
        opts.Network (1,1) string {mustBeMember(opts.Network, ["", "nodal", "simscape"])} = ""
        opts.FastRestart (1,1) logical = false
    end

    cases = defineCases(setName);
    if opts.FastRestart
        checkTunable(cases, setName);
    end
    pBase = HPP_parameters();
    if opts.Network ~= "", pBase.net.solverType = opts.Network; end
    checkModelBuild(pBase);

    in(height(cases)) = Simulink.SimulationInput("HPP");
    for k = 1:height(cases)
        p = applyCase(pBase, cases(k, :));
        in(k) = Simulink.SimulationInput("HPP");
        in(k) = in(k).setVariable("p", p);
        in(k) = in(k).setVariable("HPP_network", networkCode(p.net.solverType), "Workspace", "HPP");
        in(k) = in(k).setModelParameter("StopTime", num2str(p.sim.Tstop));
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

    if opts.Save
        folder = fullfile(currentProject().RootFolder, "results", setName);
        if ~isfolder(folder)
            mkdir(folder);
        end
        f = fullfile(folder, "sweep.mat");
        save(f, "out", "cases", "setName");
        fprintf("Saved %s\n", f);
    end
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
