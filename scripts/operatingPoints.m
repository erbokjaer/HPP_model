function [in, info] = operatingPoints(in, ps, win, mode, opts)
%OPERATINGPOINTS  Start runs from a saved operating point: the event window.
%
%   [in, info] = operatingPoints(in, ps, win, mode) for SimulationInputs
%   in(k) with parameters ps{k} (cell, or a struct for a single run), the
%   window win of the case set (eventWindow) and the simulation mode the
%   inputs will run in ("rapid", "normal" or "accelerator"). Returns the
%   inputs starting at win.tOp from the operating point
%   (in.setInitialState) and stopping at min(win.tEvent + win.post,
%   p.sim.Tstop), and info(k) with the cache file and whether it was made
%   in this call.
%   operatingPoints(..., FastRestart=true) runs the missing operating
%   points with Fast Restart (normal mode).
%
%   An operating point is the complete state of HPP.slx at tOp (controllers,
%   PLLs, delay buffers, filters, network states, the time), saved with
%   SaveOperatingPoint by running the case itself from t = 0 to tOp. It is
%   cached in <Simulink cache folder>/hpp_operating_points/<key>.mat and
%   reused only when both of these match:
%   - the build: simulation mode, compileSignature(p) (layout, time step,
%     delays, variants), the model files (newest models/**/*.slx) and the
%     MATLAB release; in Rapid Accelerator also the target itself (the
%     fingerprint file rapidTarget writes after each build), because an
%     operating point is only valid for the program that saved it;
%   - the pre-event condition: all of p except sim.Tstop and the event
%     groups (eventTimes) whose time is after tOp. Those act only from
%     their time on, so the state at tOp does not depend on them: the cases
%     of a set that differ only in the event (size, time after tOp) share
%     one operating point; any other difference (SCR, PLL bandwidth, FSM
%     on, SOC0, ...) gets its own.
%   Several missing operating points of one call run together (sim on an
%   array, with Fast Restart if asked). Delete the folder to clear the
%   cache; a stale entry is never used because the key changes with the
%   build.

    arguments
        in Simulink.SimulationInput
        ps
        win (1,1) struct
        mode (1,1) string {mustBeMember(mode, ["normal", "accelerator", "rapid"])}
        opts.FastRestart (1,1) logical = false
    end
    if isstruct(ps), ps = {ps}; end
    n = numel(in);
    folder = fullfile(Simulink.fileGenControl("get", "CacheFolder"), "hpp_operating_points");
    if ~isfolder(folder), mkdir(folder); end

    files = strings(n, 1);
    keys = cell(n, 1);
    for k = 1:n
        key = struct("build", buildKey(ps{k}, mode), "tOp", win.tOp, "p", preEvent(ps{k}, win.tOp));
        files(k) = fullfile(folder, hashKey(key) + ".mat");
        keys{k} = key;
    end

    % run the missing operating points (one run per distinct key)
    [uf, first] = unique(files, "stable");
    miss = first(~isfile(uf));
    made = false(n, 1);
    if ~isempty(miss)
        fprintf("Operating point at %g s: %d to make (%d case(s) share them), %d cached.\n", ...
            win.tOp, numel(miss), sum(ismember(files, files(miss))), n - sum(ismember(files, files(miss))));
        inOp = in(miss);
        for j = 1:numel(miss)
            inOp(j) = inOp(j).setModelParameter("StopTime", num2str(win.tOp, 17), ...
                "SaveFinalState", "on", "SaveOperatingPoint", "on", "FinalStateName", "xOp");
        end
        pr = Progress("Operating points", 0);
        pr.step(numel(miss) + " run(s) to " + win.tOp + " s");
        if numel(miss) == 1
            outOp = sim(inOp, "ShowSimulationManager", "off");
        else
            outOp = sim(inOp, "ShowProgress", "on", "UseFastRestart", onOff(opts.FastRestart));
        end
        pr.done();
        for j = 1:numel(miss)
            if ~isempty(outOp(j).ErrorMessage)
                error("operatingPoints:failed", "The run to the operating point failed: %s", outOp(j).ErrorMessage);
            end
            op = outOp(j).xOp;
            key = keys{miss(j)};
            tmp = files(miss(j)) + "." + feature("getpid") + ".tmp";
            save(tmp, "op", "key", "-v7");          % -v7 loads ~6x faster than -v7.3 (5 MB)
            movefile(tmp, files(miss(j)), "f");          % atomic for parallel sessions
            made(files == files(miss(j))) = true;
        end
    end

    info = struct("file", cellstr(files), "made", num2cell(made), "tOp", win.tOp);
    loaded = containers.Map();
    for k = 1:n
        f = char(files(k));
        if ~isKey(loaded, f)
            S = load(f, "op", "key");
            if ~isequaln(S.key, keys{k})                  % hash collision or corrupt file
                error("operatingPoints:key", "Cache file %s does not hold this key; delete it.", f);
            end
            loaded(f) = S.op;
        end
        tStop = min(win.tEvent + win.post, ps{k}.sim.Tstop);
        in(k) = in(k).setInitialState(loaded(f));
        in(k) = in(k).setModelParameter("StopTime", num2str(tStop, 17));
    end
end

function q = preEvent(p, tOp)
    % p without what cannot act before tOp: the stop time and the event
    % groups whose time is after tOp; also p.meta (creation time stamp,
    % not used by the model)
    q = rmfield(p, intersect("meta", fieldnames(p)));
    q.sim = rmfield(q.sim, "Tstop");
    for e = eventTimes(p)
        if e.t > tOp
            path = split(e.path, ".");
            q = setfield(q, path{:}, []);
        end
    end
end

function b = buildKey(p, mode)
    b = struct("mode", mode, "sig", compileSignature(p), "models", modelStamp(), "release", version);
    if mode == "rapid"
        % the Rapid Accelerator target that saves and restores the state
        % (rapidTarget saves this file after every successful build)
        f = fullfile(Simulink.fileGenControl("get", "CacheFolder"), "slprj", "raccel", "HPP", ...
            "hpp_rapid_fingerprint.mat");
        d = dir(f);
        if isempty(d)
            error("operatingPoints:noTarget", "No Rapid Accelerator target (%s); call rapidTarget(p) first.", f);
        end
        b.target = d.datenum;
    end
end

function s = modelStamp()
    % newest modification time of the model files (as rapidTarget)
    root = currentProject().RootFolder;
    d = dir(fullfile(root, "models", "**", "*.slx"));
    s = max([d.datenum]);
end

function h = hashKey(key)
    md = java.security.MessageDigest.getInstance("MD5");
    md.update(getByteStreamFromArray(key));
    h = string(lower(reshape(dec2hex(typecast(md.digest(), "uint8"))', 1, [])));
end

function s = onOff(b)
    if b, s = "on"; else, s = "off"; end
end
