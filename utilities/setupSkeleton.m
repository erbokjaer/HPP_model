function setupSkeleton()
%SETUPSKELETON  One-off: register project folders/path/startup and create
%   skeleton models HPP.slx and HPP_controller.slx. Safe to delete after use.
%   Refuses to overwrite existing models.
root = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(fullfile(root, "params"));
assert(~isfile(fullfile(root,"models","HPP.slx")) && ~isfile(fullfile(root,"models","HPP_controller.slx")), ...
    "Models already exist - not overwriting.");
proj = openProject(fullfile(root, "HPP_model.prj"));

% ---------- project folders, path, startup, cache ----------
fclose(fopen(fullfile(root, "results", ".gitkeep"), "w"));
for f = ["params","models","scripts","utilities","hpc"]
    addFolderIncludingChildFiles(proj, fullfile(root, f));
    if f ~= "hpc", addPath(proj, fullfile(root, f)); end
end
addFile(proj, fullfile(root, ".gitignore"));
addFile(proj, fullfile(root, "results"));
addFile(proj, fullfile(root, "results", ".gitkeep"));
addStartupFile(proj, fullfile(root, "utilities", "projectStartup.m"));
proj.SimulinkCacheFolder   = fullfile(root, "work");
proj.SimulinkCodeGenFolder = fullfile(root, "work");

p = HPP_parameters(); assignin("base", "p", p);
modelsDir = fullfile(root, "models");

% ---------- controller (referenced model, code-gen target) ----------
c = "HPP_controller";
if bdIsLoaded(c), close_system(c, 0); end
new_system(c);
set_param(c, "SolverType","Fixed-step", "Solver","FixedStepDiscrete", ...
    "FixedStep","p.ctrl.Ts", "StopTime","inf");
ins  = ["P_ref","P_wt_avail","P_pv_avail","SOC"];
outs = ["P_wt_set","P_pv_set","P_bess_set"];
for k = 1:numel(ins)
    add_block("simulink/Sources/In1", c+"/"+ins(k), "Port",num2str(k), ...
        "SampleTime","p.ctrl.Ts", "Position",[50 50+80*k 80 64+80*k]);
end
for k = 1:numel(outs)
    add_block("simulink/Sinks/Out1", c+"/"+outs(k), "Port",num2str(k), ...
        "Position",[600 50+80*k 630 64+80*k]);
end
add_block("simulink/Math Operations/Sum", c+"/P_res", "Inputs","++", "Position",[200 200 230 240]);
add_block("simulink/Math Operations/Sum", c+"/P_bess_req", "Inputs","+-", "Position",[320 120 350 160]);
add_block("simulink/Discontinuities/Saturation", c+"/BESS limit", ...
    "UpperLimit","p.ctrl.Pbess_max", "LowerLimit","-p.ctrl.Pbess_max", "Position",[420 125 470 155]);
add_block("simulink/Sinks/Terminator", c+"/SOC unused (TODO)", "Position",[200 370 220 390]);
add_line(c, "P_wt_avail/1", "P_res/1", "autorouting","on");
add_line(c, "P_pv_avail/1", "P_res/2", "autorouting","on");
add_line(c, "P_ref/1", "P_bess_req/1", "autorouting","on");
add_line(c, "P_res/1", "P_bess_req/2", "autorouting","on");
add_line(c, "P_bess_req/1", "BESS limit/1", "autorouting","on");
add_line(c, "BESS limit/1", "P_bess_set/1", "autorouting","on");
add_line(c, "P_wt_avail/1", "P_wt_set/1", "autorouting","on");
add_line(c, "P_pv_avail/1", "P_pv_set/1", "autorouting","on");
add_line(c, "SOC/1", "SOC unused (TODO)/1", "autorouting","on");
set_param(c, "Description", ...
    "HPP plant controller. Placeholder dispatch: BESS covers P_ref - (P_wt + P_pv). " + ...
    "Uses only p.ctrl.* parameters. Kept as a referenced model so it can be code-generated / built as an IEEE/CIGRE DLL.");
Simulink.BlockDiagram.arrangeSystem(c);
save_system(c, fullfile(modelsDir, c + ".slx")); close_system(c);

% ---------- top model ----------
m = "HPP";
if bdIsLoaded(m), close_system(m, 0); end
new_system(m);
set_param(m, "SolverType","Fixed-step", "Solver","ode4", "FixedStep","p.sim.Ts", ...
    "StopTime","p.sim.Tstop", "SaveOutput","on", "SignalLogging","on");
add_block("simulink/Sources/Constant", m+"/P_ref", "Value","p.grid.Pref", "Position",[50 60 120 90]);
add_block("simulink/Ports & Subsystems/Model", m+"/Controller", "ModelName", c, ...
    "SimulationMode","Normal", "Position",[220 40 400 220]);

% plant placeholder subsystem
pl = m + "/Plant";
add_block("simulink/Ports & Subsystems/Subsystem", pl, "Position",[520 40 700 220]);
delete_line(pl, "In1/1", "Out1/1"); delete_block(pl+"/In1"); delete_block(pl+"/Out1");
pin  = ["P_wt_set","P_pv_set","P_bess_set"];
pout = ["P_wt_avail","P_pv_avail","SOC","P_poc"];
for k = 1:3, add_block("simulink/Sources/In1",  pl+"/"+pin(k),  "Port",num2str(k)); end
for k = 1:4, add_block("simulink/Sinks/Out1",   pl+"/"+pout(k), "Port",num2str(k)); end
add_block("simulink/Sources/Step", pl+"/Wind available", "Time","p.wt.tStep", ...
    "Before","p.wt.Pavail0*p.wt.Prated", "After","p.wt.PavailStep*p.wt.Prated", "SampleTime","0");
add_block("simulink/Sources/Constant", pl+"/PV available", "Value","p.pv.Pavail*p.pv.Prated");
add_block("simulink/Math Operations/Gain", pl+"/SOC gain", "Gain","-1/p.bess.E");
add_block("simulink/Continuous/Integrator", pl+"/SOC integrator", "InitialCondition","p.bess.SOC0");
add_block("simulink/Math Operations/Sum", pl+"/P_poc sum", "Inputs","+++");
add_line(pl, "Wind available/1", "P_wt_avail/1", "autorouting","on");
add_line(pl, "PV available/1", "P_pv_avail/1", "autorouting","on");
add_line(pl, "P_bess_set/1", "SOC gain/1", "autorouting","on");
add_line(pl, "SOC gain/1", "SOC integrator/1", "autorouting","on");
add_line(pl, "SOC integrator/1", "SOC/1", "autorouting","on");
add_line(pl, "P_wt_set/1", "P_poc sum/1", "autorouting","on");
add_line(pl, "P_pv_set/1", "P_poc sum/2", "autorouting","on");
add_line(pl, "P_bess_set/1", "P_poc sum/3", "autorouting","on");
add_line(pl, "P_poc sum/1", "P_poc/1", "autorouting","on");
set_param(pl, "Description", "Placeholder plant. Replace with wind/PV/BESS/grid models.");
Simulink.BlockDiagram.arrangeSystem(pl);

% wiring top level
add_line(m, "P_ref/1", "Controller/1", "autorouting","on");
for k = 1:3, add_line(m, "Controller/"+k, "Plant/"+k, "autorouting","on"); end
add_line(m, "Plant/1", "Controller/2", "autorouting","on");
add_line(m, "Plant/2", "Controller/3", "autorouting","on");
add_line(m, "Plant/3", "Controller/4", "autorouting","on");
outNames = ["P_poc","SOC","P_bess_set"];
outSrc   = ["Plant/4","Plant/3","Controller/3"];
for k = 1:3
    add_block("simulink/Sinks/Out1", m+"/"+outNames(k), "Port",num2str(k));
    h = add_line(m, outSrc(k), outNames(k)+"/1", "autorouting","on");
    set_param(h, "Name", outNames(k));
end
Simulink.BlockDiagram.arrangeSystem(m);
save_system(m, fullfile(modelsDir, m + ".slx")); close_system(m);

addFolderIncludingChildFiles(proj, modelsDir);
fprintf("Skeleton built.\n");
end
