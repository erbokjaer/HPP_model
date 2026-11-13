function out = runCase(setName, caseId, opts)
%RUNCASE  Run a single case in batch mode (no GUI). HPC entry point.
%
%   runCase()                                 nominal case (project default)
%   runCase("handpicked", "noBess")           case by row name
%   runCase("grid_Pref_Pbess", 4)             case by row number
%   runCase("oat_Pref", 2, Save=false)        don't write a results file
%   runCase(..., Mode="rapid")                Rapid Accelerator (needs C compiler)
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
    end

    setName = string(setName);
    cases = defineCases(setName);
    [k, caseName] = selectCase(cases, caseId, setName);

    p = applyCase(HPP_parameters(), cases(k, :));
    fprintf("Running %s, case %d%s\n", setName, k, nameSuffix(caseName));

    in = Simulink.SimulationInput("HPP");
    in = in.setVariable("p", p);
    in = in.setModelParameter("SimulationMode", modeName(opts.Mode), ...
                              "StopTime", num2str(p.sim.Tstop));

    out = sim(in);

    if opts.Save
        folder = fullfile(currentProject().RootFolder, "results", setName);
        if ~isfolder(folder)
            mkdir(folder);
        end
        caseInfo = struct("set", setName, "index", k, "name", caseName, ...
                          "row", cases(k, :));
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
