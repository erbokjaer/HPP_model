function plan = buildfile
%BUILDFILE  Build tasks of HPP_model, for the project's "Run Build" and buildtool.
%
%   buildtool                 default task: rapid (check -> model -> rapid)
%   buildtool check           check the hand-maintained controller / asset models
%   buildtool model           regenerate models/HPP.slx (after check)
%   buildtool rapid           compile the Rapid Accelerator program (after model)
%   buildtool sweeps          run the case sets used by the report figures
%   buildtool figures         make the report figures (after sweeps)
%   buildtool -tasks          list the tasks
%
%   Tasks run in order and the chain stops at the first failure. model,
%   sweeps and figures declare their input and output files, so buildtool
%   skips them when nothing they depend on has changed.

    plan = buildplan(localfunctions);
    plan.DefaultTasks = "rapid";

    plan("model").Dependencies = "check";
    plan("model").Inputs = ["params/*.m", "utilities/buildHPP.m", "utilities/modelInterfaces.m", ...
                            "models/ctrl/*.slx", "models/assets/*.slx"];
    plan("model").Outputs = "models/HPP.slx";

    plan("rapid").Dependencies = "model";

    plan("sweeps").Dependencies = "rapid";
    plan("sweeps").Inputs = ["params/*.m", "scripts/+caseSets/*.m", "models/**/*.slx"];
    plan("sweeps").Outputs = "results/" + reportSets() + "/sweep.mat";

    plan("figures").Dependencies = "sweeps";
    plan("figures").Inputs = ["results/" + reportSets() + "/sweep.mat", "scripts/reportFigures.m", ...
                              "scripts/plotAllResults.m"];
    plan("figures").Outputs = "results/figures/report";
end

function checkTask(~)
    % Check the controller and asset models (expressions, ports, fields)
    checkModels(Strict=true);
end

function modelTask(~)
    % Regenerate models/HPP.slx from the parameters (controller and asset models are kept)
    buildHPP();
end

function rapidTask(~)
    % Compile the Rapid Accelerator program for the default parameters
    st = rapidTarget(HPP_parameters());
    if ~st.ok
        error("buildfile:rapid", "Rapid Accelerator build not possible: %s", st.reason);
    end
end

function sweepsTask(~)
    % Run the case sets of the report figures in Rapid Accelerator
    for s = reportSets()
        runSweep(s, Mode="rapid");
    end
end

function figuresTask(~)
    % Make the report figures (results/figures/report)
    plotAllResults(Purpose="report");
end

function s = reportSets()
    % case sets that reportFigures.m uses
    s = ["nominal", "freq_step", "hppc_p", "hppc_qv", "wt_frt", "bess_soc"];
end
