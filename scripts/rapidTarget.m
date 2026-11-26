function st = rapidTarget(p, opts)
%RAPIDTARGET  Make sure the Rapid Accelerator target of HPP.slx is up to date.
%
%   st = rapidTarget(p)              build it if needed, for the compile-time
%                                    values in p
%   st = rapidTarget(p, Force=true)  always rebuild
%   st = rapidTarget(p, Strict=true) error on checkModels findings
%   st = rapidTarget(p, Verbose=true) print Simulink's build log too
%
%   A build shows its progress (Progress: the duration of the last build
%   as an estimate; Simulink reports no progress while it compiles).
%
%   Returns a status struct: st.ok (true: run in Rapid Accelerator), st.reason
%   (why not, when st.ok is false) and st.built (true if built in this call).
%   It does not error: when Rapid Accelerator is not possible it warns with
%   the reason, and runCase / runSweep run the case in normal mode instead.
%
%   The Rapid Accelerator target is HPP.slx with its referenced controller
%   and asset models compiled into one program (MinGW). It is built once and
%   then reused: every run passes its own p, and all parameter values are
%   tunable (controllers, grid and network data, events, stop time). It is
%   rebuilt only when something fixed at compile time changes: the layout
%   (units per plant), the time step, the delay mode p.comm.delays (with
%   "fixed" delays also the delays; with "variable" delays comm.maxDelay,
%   the size of the delay buffers), the network model (with the Simscape
%   network: its mode, phasor or EMT, simscapeMode), a model file in
%   models/ (after buildHPP or an edit), or the structure of p (field
%   names, classes and sizes, e.g. after a new parameter: a target only
%   accepts a p of the type it was built with). Every build starts from an
%   empty target folder (an older target there would keep the old type of p). p.comm.delays = "auto" counts as
%   "fixed" here (runCase / runSweep resolve it per case set, delayMode). The
%   fingerprint of these is saved next to the target
%   (hpp_rapid_fingerprint.mat), so a new MATLAB session reuses an unchanged
%   target without rebuilding.
%
%   Not possible (normal mode instead), with the reason in the warning:
%   - no C compiler selected (mex -setup C);
%   - checkModels finds block expressions that compiled code cannot use;
%   - the build fails.
%   A failure is remembered for the same compile-time values and model files,
%   so a case set warns once.
%
%   Test hook: setenv("HPP_TEST_NO_COMPILER", "1") makes it behave as if no
%   C compiler were installed.

    arguments
        p (1,1) struct
        opts.Force (1,1) logical = false
        opts.Strict (1,1) logical = false
        opts.Verbose (1,1) logical = false
    end
    persistent lastKey lastStatus
    if p.comm.delays == "auto"
        p = delayMode(p, table());                        % single run: "auto" means fixed
    end

    % Rapid Accelerator cannot run while the model is in Fast Restart (e.g.
    % after a normal-mode run with FastRestart=true): switch it off first
    fastRestartOff();

    key = struct("sig", compileSignature(p), "models", modelStamp(), "ptype", typeSignature(p), ...
        "noCompilerHook", getenv("HPP_TEST_NO_COMPILER"));
    folder = fullfile(Simulink.fileGenControl("get", "CacheFolder"), "slprj", "raccel", "HPP");
    keyFile = fullfile(folder, "hpp_rapid_fingerprint.mat");
    if isempty(lastKey) && isfile(keyFile)
        % new MATLAB session: the fingerprint of the last successful build is
        % saved next to the target, so an unchanged target is reused
        S = load(keyFile, "key");
        lastKey = S.key;  lastStatus = struct("ok", true, "reason", "", "built", false);
    end
    if ~opts.Force && isequal(key, lastKey) && ~isempty(lastStatus) && (~lastStatus.ok || isfolder(folder))
        st = lastStatus;  st.built = false;
        return
    end

    st = struct("ok", false, "reason", "", "built", false);
    if getenv("HPP_TEST_NO_COMPILER") == "1"
        cc = [];                                          % test hook
    else
        cc = mex.getCompilerConfigurations("C", "Selected");
    end
    if isempty(cc)
        st.reason = "no C compiler is selected (run mex -setup C; MinGW-w64 is the supported free one)";
    else
        findings = checkModels(Quiet=true, Strict=false);
        expr = findings(findings.Check == "expression", :);
        if ~isempty(findings)
            checkModels(Strict=opts.Strict);              % prints the report (errors with Strict)
        end
        if ~isempty(expr)
            st.reason = sprintf("checkModels found %d block parameter(s) that compiled code cannot keep tunable " + ...
                "(first: %s)", height(expr), expr.Block(1));
        else
            try
                buildTarget(p, opts.Verbose);
                st.ok = true;  st.built = true;
                save(keyFile, "key");                     % fingerprint for later sessions
            catch err
                msg = strtrim(string(err.message));  msg = extractBefore(msg + newline, newline);
                st.reason = "the build failed: " + msg;
            end
        end
    end
    if ~st.ok
        warning("rapidTarget:fallback", "Rapid Accelerator is not possible: %s. Running in normal mode instead.", st.reason);
    end
    lastKey = key;  lastStatus = st;
end

function buildTarget(p, verbose)
    load_system("HPP");
    fastRestartOff();                                    % Fast Restart and Rapid Accelerator exclude each other
    % the build reads p from the base workspace and the variant choices
    % (variantVars) from the model workspace or as globals; put them there
    % for the build and restore afterwards
    hadP = evalin("base", "exist('p', 'var') == 1");
    pOld = [];
    if hadP, pOld = evalin("base", "p"); end
    assignin("base", "p", p);
    restoreP = onCleanup(@() restoreBase(hadP, pOld)); %#ok<NASGU>
    ws = get_param("HPP", "ModelWorkspace");
    [~, dc] = delayMode(p, table());
    restore = {};
    for v = variantVars(p, dc)
        if v.where == "global"
            had = evalin("base", "exist('" + v.name + "', 'var') == 1");  old = [];
            if had, old = evalin("base", v.name); end
            assignin("base", v.name, v.value);
            restore{end+1} = onCleanup(@() restoreBaseVar(v.name, had, old)); %#ok<AGROW>
        else
            old = getVariable(ws, v.name);
            assignin(ws, v.name, v.value);
            restore{end+1} = onCleanup(@() assignin(ws, v.name, old)); %#ok<AGROW>
        end
    end

    % start from an empty target: a build into a folder with an older target
    % keeps the old type of p (its fields and their classes), and the first
    % run with the new p then fails ("The tuned value of the variable 'p'
    % ... has a different type than the variable's default value"), forced
    % rebuild or not
    removeOldTarget();

    % one step: Simulink reports no progress while it compiles; the estimate
    % is last build's duration. Verbose: Simulink's own build log as well
    set_param("HPP", "AccelVerboseBuild", onOff(verbose));
    pr = Progress("Rapid Accelerator build of HPP", 0);
    pr.step("code generation and C compilation (once per layout / model change)");
    Simulink.BlockDiagram.buildRapidAcceleratorTarget("HPP");
    pr.done();
end

function restoreBase(hadP, pOld)
    restoreBaseVar("p", hadP, pOld);
end

function restoreBaseVar(name, had, old)
    if had
        assignin("base", name, old);
    else
        evalin("base", "clear " + name);
    end
end

function removeOldTarget()
    % the Rapid Accelerator target of HPP (slprj/raccel/HPP, with the
    % fingerprint) and HPP's Simulink cache file, which can hold a copy of it
    cache = Simulink.fileGenControl("get", "CacheFolder");
    folder = fullfile(cache, "slprj", "raccel", "HPP");
    if isfolder(folder), rmdir(folder, "s"); end
    slxc = fullfile(cache, "HPP.slxc");
    if isfile(slxc), delete(slxc); end
end

function s = typeSignature(x)
    % field names, classes and sizes of p, recursively: a Rapid Accelerator
    % target only accepts a p of the type it was built with, so a change in
    % p's structure (a new or removed field, string / char) needs a rebuild
    % even when compileSignature is unchanged
    if isstruct(x)
        f = string(fieldnames(x))';
        s = "struct" + mat2str(size(x)) + "{";
        if isscalar(x)
            for n = f, s = s + n + ":" + typeSignature(x.(n)) + ";"; end
        else
            s = s + strjoin(f, ",");
        end
        s = s + "}";
    else
        s = string(class(x)) + mat2str(size(x));
    end
end

function s = modelStamp()
    % newest modification time of the model files (top, controllers, assets)
    root = currentProject().RootFolder;
    d = dir(fullfile(root, "models", "**", "*.slx"));
    s = max([d.datenum]);
end

function s = onOff(b)
    if b, s = "on"; else, s = "off"; end
end
