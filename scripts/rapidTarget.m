function rapidTarget(p, opts)
%RAPIDTARGET  Make sure the Rapid Accelerator target of HPP.slx is up to date.
%
%   rapidTarget(p)              build it if needed, for the compile-time
%                               values in p
%   rapidTarget(p, Force=true)  always rebuild
%
%   The Rapid Accelerator target is HPP.slx with its referenced controller
%   and asset models compiled into one program (MinGW). It is built once and
%   then reused: every run passes its own p, and all parameter values are
%   tunable (controllers, grid and network data, events, stop time). It is
%   rebuilt only when something fixed at compile time changes: the layout
%   (units per plant), the time step, the communication delays, the network
%   model, or a model file in models/ (after buildHPP or an edit).
%   runCase and runSweep call this with Mode="rapid"; runs then use
%   RapidAcceleratorUpToDateCheck = "off", so they skip Simulink's own
%   (slow) check.

    arguments
        p (1,1) struct
        opts.Force (1,1) logical = false
    end
    persistent lastKey

    key = struct("sig", compileSignature(p), "models", modelStamp());
    folder = fullfile(Simulink.fileGenControl("get", "CacheFolder"), "slprj", "raccel", "HPP");
    if ~opts.Force && isequal(key, lastKey) && isfolder(folder)
        return
    end

    load_system("HPP");
    if strcmp(get_param("HPP", "FastRestart"), "on")
        set_param("HPP", "FastRestart", "off");          % Fast Restart and Rapid Accelerator exclude each other
    end
    % the build reads p from the base workspace and the network choice from
    % the model workspace; put them there for the build and restore afterwards
    hadP = evalin("base", "exist('p', 'var') == 1");
    pOld = [];
    if hadP, pOld = evalin("base", "p"); end
    assignin("base", "p", p);
    restoreP = onCleanup(@() restoreBase(hadP, pOld));
    ws = get_param("HPP", "ModelWorkspace");
    netOld = getVariable(ws, "HPP_network");
    assignin(ws, "HPP_network", networkCode(p.net.solverType));
    restoreNet = onCleanup(@() assignin(ws, "HPP_network", netOld));

    fprintf("Building the Rapid Accelerator target of HPP (once per layout / model change) ...\n");
    t = tic;
    Simulink.BlockDiagram.buildRapidAcceleratorTarget("HPP");
    fprintf("Rapid Accelerator target built in %.0f s\n", toc(t));
    lastKey = key;
end

function restoreBase(hadP, pOld)
    if hadP
        assignin("base", "p", pOld);
    else
        evalin("base", "clear p");
    end
end

function s = modelStamp()
    % newest modification time of the model files (top, controllers, assets)
    root = currentProject().RootFolder;
    d = dir(fullfile(root, "models", "**", "*.slx"));
    s = max([d.datenum]);
end

function sig = compileSignature(p)
    % same compile-time values as runCase's Fast Restart check
    sig = struct("N", [p.wt.N, p.pv.N, p.bess.N], "Ts", p.sim.Ts, ...
        "comm", p.comm, "network", p.net.solverType);
end
