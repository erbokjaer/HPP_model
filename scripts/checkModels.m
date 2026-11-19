function T = checkModels(opts)
%CHECKMODELS  Check the hand-maintained controller and asset models before a build.
%
%   T = checkModels()                    check all controller and asset models
%   T = checkModels(Models=["HPPC"])     only these models
%   T = checkModels(Strict=true)         error if anything is found
%   T = checkModels(Root="C:\tmp\copy")  check models under <Root>/ctrl and
%                                        <Root>/assets instead of models/
%   T = checkModels(Quiet=true)          no printed report
%
%   The models in models/ctrl and models/assets are the source (buildHPP
%   does not overwrite them). This finds the hand edits that would break the
%   build or the simulation, without opening any window:
%   - expression: a block dialog parameter that uses the model argument
%     (par / ap) with anything other than + - * /, parentheses, numbers and
%     field access. Compiled simulation (Rapid Accelerator) and code
%     generation cannot keep such a parameter tunable and fail. Code inside
%     MATLAB Function blocks is not restricted and is not checked here.
%   - port: inport / outport names or order that differ from what HPP.slx
%     expects (modelInterfaces); HPP.slx connects by port number.
%   - field: par / ap fields used in the model (blocks or MATLAB Function
%     code) that HPP_parameters() does not define.
%   Returns a table with one row per finding: Model, Check, Block, Detail, Fix.
%   Models that do not exist yet are skipped (buildHPP creates them).

    arguments
        opts.Models (1,:) string = strings(1, 0)
        opts.Strict (1,1) logical = false
        opts.Root (1,1) string = ""
        opts.Quiet (1,1) logical = false
    end

    I = modelInterfaces();
    names = string(fieldnames(I))';
    if ~isempty(opts.Models), names = intersect(names, opts.Models, "stable"); end
    root = opts.Root;
    if root == "", root = fullfile(currentProject().RootFolder, "models"); end

    p = HPP_parameters();
    rows = cell(0, 5);
    for m = names
        f = fullfile(root, I.(m).folder, m + ".slx");
        if ~isfile(f), continue, end
        wasLoaded = bdIsLoaded(m);
        if wasLoaded && ~strcmpi(get_param(m, "FileName"), f)
            close_system(m, 0);  wasLoaded = false;      % another file of that name is loaded
        end
        ws = warning("off", "Simulink:Engine:MdlFileShadowing");
        load_system(f);
        warning(ws);
        cleanup = onCleanup(@() closeIfOpened(m, wasLoaded)); %#ok<NASGU>
        arg = I.(m).arg;
        def = eval("p." + I.(m).src);                    % default argument struct
        rows = [rows; portFindings(m, I.(m))]; %#ok<AGROW>
        [r, used] = blockFindings(m, arg);
        rows = [rows; r]; %#ok<AGROW>
        used = [used; codeFields(m, arg)]; %#ok<AGROW>
        rows = [rows; fieldFindings(m, arg, def, used)]; %#ok<AGROW>
        clear cleanup
    end
    % always string columns, also when empty (callers filter on T.Check)
    T = table(strings(0, 1), strings(0, 1), strings(0, 1), strings(0, 1), strings(0, 1), ...
        'VariableNames', ["Model", "Check", "Block", "Detail", "Fix"]);
    for k = 1:size(rows, 1)
        T(end+1, :) = cellfun(@string, rows(k, :), "UniformOutput", false); %#ok<AGROW>
    end

    if ~opts.Quiet
        if isempty(T)
            fprintf("checkModels: no problems in %d model(s).\n", numel(names));
        else
            fprintf("checkModels: %d finding(s):\n", height(T));
            for k = 1:height(T)
                fprintf("  [%s] %s  %s\n      %s\n      fix: %s\n", T.Check(k), T.Model(k), T.Block(k), T.Detail(k), T.Fix(k));
            end
        end
    end
    if opts.Strict && ~isempty(T)
        error("checkModels:findings", "checkModels found %d problem(s) in the controller / asset models (see above).", height(T));
    end
end

% ------------------------------------------------------------------ ports
function rows = portFindings(m, spec)
    rows = cell(0, 5);
    kinds = ["Inport", "Outport"];  expect = {spec.in, spec.out};
    for j = 1:2
        b = string(find_system(m, "MatchFilter", @Simulink.match.allVariants, "SearchDepth", 1, ...
            "BlockType", kinds(j)));
        if isempty(b), have = strings(1, 0);
        else
            num = str2double(string(get_param(b, "Port")));
            [~, o] = sort(num);  b = b(o);
            have = string(get_param(b, "Name"))';
        end
        want = expect{j};
        if isequal(have, want), continue, end
        n = max(numel(have), numel(want));
        for k = 1:n
            h = "";  w = "";
            if k <= numel(have), h = have(k); end
            if k <= numel(want), w = want(k); end
            if h ~= w
                if h == "", what = sprintf("%s %d missing: expected ""%s""", lower(kinds(j)), k, w);
                elseif w == "", what = sprintf("%s %d ""%s"" is not expected (HPP.slx has %d)", lower(kinds(j)), k, h, numel(want));
                else, what = sprintf("%s %d is ""%s"", expected ""%s""", lower(kinds(j)), k, h, w);
                end
                rows(end+1, :) = {m, "port", m + "/" + h, what, ...
                    "keep port names and order as in modelInterfaces.m (HPP.slx connects by number); " + ...
                    "if the change is intended, update modelInterfaces.m and buildHPP's wiring, then buildHPP()"}; %#ok<AGROW>
            end
        end
    end
end

% ------------------------------------------------------ block expressions
function [rows, used] = blockFindings(m, arg)
    rows = cell(0, 5);  used = strings(0, 1);
    % all blocks, also in inactive variant choices (they may become active)
    blks = string(find_system(m, "MatchFilter", @Simulink.match.allVariants, "LookUnderMasks", "all", ...
        "FollowLinks", "on", "Type", "block"));
    % skip the inside of MATLAB Function blocks (their code is checked for fields only)
    mfb = strings(0, 1);
    for b = blks'
        try
            if strcmp(get_param(b, "SFBlockType"), "MATLAB Function"), mfb(end+1, 1) = b; end %#ok<AGROW>
        catch
        end
    end
    pat = "(?<![\w.])" + arg + "(\.[A-Za-z_]\w*)+";
    for b = blks'
        if any(startsWith(b, mfb + "/")), continue, end
        dp = get_param(b, "DialogParameters");
        if ~isstruct(dp), continue, end
        for pn = string(fieldnames(dp))'
            try, v = get_param(b, pn); catch, continue, end
            if ~(ischar(v) || isstring(v)), continue, end
            v = string(v);
            if ~contains(v, arg + "."), continue, end
            refs = string(regexp(v, pat, "match"));
            if isempty(refs), continue, end
            used = [used; refs(:)]; %#ok<AGROW>
            bad = badTokens(v);
            if bad ~= ""
                rows(end+1, :) = {m, "expression", b, ...
                    sprintf("parameter %s = ""%s"" uses %s", pn, v, bad), ...
                    "compute the value in params/controllerParameters.m (e.g. a new field " + arg + ".<name>) " + ...
                    "and use only that field here; or move the calculation into a MATLAB Function block"}; %#ok<AGROW>
            end
        end
    end
end

function bad = badTokens(v)
    % anything other than numbers, (field-access) names, + - * / and ()
    s = regexprep(v, "\s", "");
    rest = s;
    found = strings(0);
    % names followed by "(" are function calls or indexing: not allowed
    calls = regexp(rest, "[A-Za-z_][\w.]*(?=\()", "match");
    if ~isempty(calls), found = [found, "function call / indexing " + join(unique(string(calls)), ", ")]; end
    rest = regexprep(rest, "[A-Za-z_]\w*(\.[A-Za-z_]\w*)*", "");          % names and field access
    rest = regexprep(rest, "(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?", "");       % numbers
    rest = regexprep(rest, "[+\-*/()]", "");                              % allowed operators
    if rest ~= ""
        found = [found, "operator(s) " + join(unique(string(cellstr(char(rest)')))', " ")];
    end
    if isempty(found), bad = ""; else, bad = join(found, "; "); end
end

% ----------------------------------------------------------------- fields
function used = codeFields(m, arg)
    % par / ap fields used in the code of the MATLAB Function blocks
    used = strings(0, 1);
    rt = sfroot;
    mdlObj = rt.find("-isa", "Simulink.BlockDiagram", "Name", m);
    if isempty(mdlObj), return, end
    charts = mdlObj.find("-isa", "Stateflow.EMChart");
    pat = "(?<![\w.])" + arg + "(\.[A-Za-z_]\w*)+";
    for c = charts'
        code = string(c.Script);
        code = regexprep(code, "%[^\n]*", "");           % ignore comments
        used = [used; string(regexp(code, pat, "match"))']; %#ok<AGROW>
    end
end

function rows = fieldFindings(m, arg, def, used)
    rows = cell(0, 5);
    used = unique(used);
    for u = used'
        parts = split(extractAfter(u, arg + "."), ".")';
        s = def;  ok = true;
        for k = 1:numel(parts)
            if isstruct(s) && isfield(s, parts(k)), s = s.(parts(k)); else, ok = false; break, end
        end
        if ~ok
            rows(end+1, :) = {m, "field", m, ...
                sprintf("%s is used in the model, but HPP_parameters() does not define it", u), ...
                "add the field to the parameters (params/*Parameters.m or controllerParameters.m), " + ...
                "or correct the name in the model"}; %#ok<AGROW>
        end
    end
end

function closeIfOpened(m, wasLoaded)
    if ~wasLoaded && bdIsLoaded(m), close_system(m, 0); end
end
