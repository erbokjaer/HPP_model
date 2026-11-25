function buildHPP(opts)
%BUILDHPP  (Re)build the hierarchical HPP model from the parameters.
%
%   buildHPP()                    rebuild the top model HPP.slx for the
%                                 asset counts N in HPP_parameters()
%   buildHPP(All=true)            also create controller and asset models
%                                 that do not exist yet, from the templates;
%                                 existing ones are never touched
%   buildHPP(All=true, Overwrite=true, Only="HPPC")  regenerate the named
%                                 models from their templates: OVERWRITES
%                                 them, including all hand edits (controller
%                                 names or "<type>_asset"; without Only: all)
%   buildHPP(..., Strict=true)    stop if checkModels finds a problem in the
%                                 controller / asset models (default: warn)
%   Progress: one step per model (waitbar in the desktop, one line per
%   model with matlab -batch), with an estimate of the time left.
%   Force=true (older option) only works together with Overwrite=true.
%
%   Ownership: HPP.slx is always generated (its structure depends on N), so
%   never edit it by hand. The controller and asset models (models/ctrl,
%   models/assets) are the source: develop them in the Simulink editor. The
%   templates in this file only create models that are missing. Keep the
%   port names and order (modelInterfaces.m; HPP.slx connects by port), and
%   in block parameters that use par / ap only + - * / and field access
%   (compute anything else in params/controllerParameters.m). checkModels
%   checks both and runs at the start of buildHPP and of Rapid Accelerator
%   builds.
%
%   Structure (see docs/design.md):
%     models/HPP.slx                 top model: HPPC, plant controllers,
%                                    N asset instances per type, POC, delays
%     models/ctrl/<name>.slx         7 controllers, each a future DLL:
%                                    HPPC, WPPC, SPPC, BESSC, WT_ctrl,
%                                    PV_ctrl, BESS_ctrl
%     models/assets/<type>_asset.slx asset plant models; each contains one
%                                    asset controller instance
%
%   Each controller: inports -> a block diagram of its control structure
%   (areas per function; MATLAB Function blocks only for logic such as
%   dispatch and current limits) -> outports, all double, per unit on its
%   own base; tunable parameters are the model argument "par"; sample time
%   "Ts". Defaults for both come from HPP_parameters() via the model
%   workspace. Layouts: docs/layout_rules.md (templates in models/layout,
%   captureLayout / applyLayout, or coded in this file).
%
%   Changing N (p.wt.N, p.pv.N, p.bess.N) requires rerunning buildHPP.
%
%   The Network block holds both network models as variants (nodal solver
%   and Simscape Electrical); the active one is chosen per run, see runCase
%   (Network option) and p.net.solverType.
%
% Files (utilities/build): buildHPP.m is the entry point (options, order);
% each builder it calls is a file of its own with its local helpers
% (buildTop, buildPllModel, buildFreqMeterModel, buildHppcCtrl,
% buildPlantCtrlRepc, buildWTCtrlType4A, buildReecCtrl, buildAsset);
% helpers shared by several builders are in utilities/build/private; the
% layout tools (captureLayout, applyLayout, signalEdges, countCrossings)
% and modelInterfaces are next to them.
    arguments
        opts.All (1,1) logical = false
        opts.Force (1,1) logical = false       % older option: needs Overwrite=true
        opts.Overwrite (1,1) logical = false   % regenerate existing controller / asset models
        opts.Only (1,:) string = strings(1, 0) % with All: create / regenerate only these models
        opts.Strict (1,1) logical = false      % error on checkModels findings
    end
    if opts.Force && ~opts.Overwrite
        error("buildHPP:overwrite", "Force=true would regenerate the controller and asset models from the " + ...
            "templates and overwrite all hand edits. The models are the source now; to really overwrite, " + ...
            "say so: buildHPP(All=true, Overwrite=true, Only=[...]).");
    end
    if opts.Overwrite && ~opts.All
        error("buildHPP:overwriteAll", "Overwrite=true applies to the controller and asset models; use it " + ...
            "with All=true, e.g. buildHPP(All=true, Overwrite=true, Only=""HPPC"").");
    end

    % hand edits in the controller / asset models (expressions, ports, fields)
    findings = checkModels(Quiet=true);
    if ~isempty(findings)
        checkModels(Strict=opts.Strict);                % prints the report (and errors with Strict)
        warning("buildHPP:checkModels", "checkModels found %d problem(s) in the controller / asset models " + ...
            "(listed above). Rapid Accelerator will fall back to normal mode until they are fixed.", height(findings));
    end

    root = currentProject().RootFolder;
    dirs.ctrl   = fullfile(root, "models", "ctrl");
    dirs.assets = fullfile(root, "models", "assets");
    dirs.top    = fullfile(root, "models");
    for d = [dirs.ctrl, dirs.assets]
        if ~isfolder(d), mkdir(d); end
    end

    p = HPP_parameters();
    % rebuilding a model that exists on the path warns about "shadowing"
    ws0 = warning("off", "Simulink:Engine:MdlFileShadowing");
    restoreWarn = onCleanup(@() warning(ws0));

    % what to build, in order: PLL and FreqMeter first (referenced by the
    % asset controllers and the plant / HPP subsystems), controllers before
    % assets (assets reference the asset controllers), the top model last;
    % created if missing, then maintained in Simulink (Overwrite: rebuilt)
    jobs = strings(1, 0);       % a row: a for loop over an empty 0x1 column runs once
    if opts.All
        names = ["PLL", "FreqMeter", "HPPC", "WPPC", "SPPC", "BESSC", "WT_ctrl", "PV_ctrl", "BESS_ctrl", ...
                 "WT_asset", "PV_asset", "BESS_asset"];
        for nm = names
            if ~isempty(opts.Only) && ~any(opts.Only == nm), continue, end
            if endsWith(nm, "_asset"), f = fullfile(dirs.assets, nm + ".slx"); else, f = fullfile(dirs.ctrl, nm + ".slx"); end
            if opts.Overwrite || ~isfile(f)
                jobs(end+1) = nm; %#ok<AGROW>
            else
                fprintf("Kept existing %s.slx\n", nm);
            end
        end
    end
    % the number of models in the title: the estimate is kept per title
    pr = Progress(sprintf("Building %d HPP model(s)", numel(jobs) + 1), numel(jobs) + 1);
    for nm = jobs
        pr.step(nm);
        switch nm
            case "PLL",       buildPllModel(dirs.ctrl);
            case "FreqMeter", buildFreqMeterModel(dirs.ctrl);
            case "HPPC",      buildHppcCtrl(dirs.ctrl);          % closed-loop P, frequency response
            case "WT_ctrl",   buildWTCtrlType4A(dirs.ctrl);      % IEC 61400-27-1 Type 4A
            case {"WPPC", "SPPC", "BESSC"}
                buildPlantCtrlRepc(nm, dirs.ctrl);               % WECC REPC_A plant controllers
            case {"PV_ctrl", "BESS_ctrl"}
                buildReecCtrl(extractBefore(nm, "_"), dirs.ctrl);   % WECC REEC_B / REEC_C
            otherwise
                buildAsset(extractBefore(nm, "_asset"), dirs.assets);
        end
    end
    pr.step("HPP (top model)");
    buildTop(p, dirs.top);
    pr.done();
end
