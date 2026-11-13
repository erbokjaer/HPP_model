function out = runSweep(setName, opts)
%RUNSWEEP  Run all cases of a case set, serially or with parsim.
%
%   out = runSweep()                              nominal set, serial
%   out = runSweep("grid_Pref_Pbess")             all cases of a set, serial
%   out = runSweep("grid_Pref_Pbess", UseParallel=true)   parsim, local pool
%   out = runSweep(..., Save=false)               don't write a results file
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
    end

    cases = defineCases(setName);
    pBase = HPP_parameters();

    in(height(cases)) = Simulink.SimulationInput("HPP");
    for k = 1:height(cases)
        p = applyCase(pBase, cases(k, :));
        in(k) = Simulink.SimulationInput("HPP");
        in(k) = in(k).setVariable("p", p);
        in(k) = in(k).setModelParameter("StopTime", num2str(p.sim.Tstop));
    end

    if opts.UseParallel
        % parsim always uses a parallel pool (starts one if needed)
        out = parsim(in, "ShowProgress", "on", ...
                         "TransferBaseWorkspaceVariables", "off");
    else
        out = sim(in, "ShowProgress", "on");   % serial, same output format
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
