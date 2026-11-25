classdef Progress < handle
%PROGRESS  Progress of the long jobs: building the models, the Rapid
%   Accelerator target, the runs of a case set.
%
%   pr = Progress("Building the HPP models", n)   a job of n steps
%   pr = Progress("Rapid Accelerator build", 0)   one step of unknown length
%   pr.step("WT_ctrl")                            the next step starts
%   pr.done()                                     the job is finished
%
%   In the MATLAB desktop a waitbar shows the step, the share done, the
%   elapsed time and an estimate of the time left; without the desktop
%   (matlab -batch) each step is one line in the Command Window. The
%   estimate comes from the steps done so far, or, before the first step
%   is done (and for a job of one step, such as the Rapid Accelerator
%   build), from the duration of the same job last time. Durations are kept
%   per job title in the Simulink cache folder (hpp_progress.mat).
%
%   Simulink has no progress callback during compilation and code
%   generation, so a single build step cannot move a bar on its own;
%   rapidTarget(..., Verbose=true) prints the build's own log as well.

    properties (SetAccess = private)
        Title (1,1) string
        N (1,1) double                 % number of steps (0: unknown)
        K (1,1) double = 0             % steps started
        Last (1,1) double = NaN        % duration of this job last time [s]
    end
    properties (Access = private)
        T0
        Bar = []
    end

    methods
        function obj = Progress(title, n)
            obj.Title = string(title);  obj.N = n;  obj.T0 = tic;
            obj.Last = Progress.lastDuration(obj.Title);
            msg = obj.Title;
            if ~isnan(obj.Last), msg = msg + " (last time " + Progress.fmt(obj.Last) + ")"; end
            if Progress.gui()
                obj.Bar = waitbar(0, char(msg), "Name", "HPP model");
                set(findall(obj.Bar, "Type", "text"), "Interpreter", "none");   % names with _ as typed
                drawnow;
            else
                fprintf("%s\n", msg);
            end
        end

        function step(obj, label)
            % the next step (label) starts
            obj.K = obj.K + 1;
            el = toc(obj.T0);
            if obj.N > 0
                frac = (obj.K - 1) / obj.N;
                if obj.K > 1
                    left = el / (obj.K - 1) * (obj.N - obj.K + 1);
                else
                    left = obj.Last;                      % NaN without an earlier run
                end
                info = sprintf("step %d of %d, %s elapsed", obj.K, obj.N, Progress.fmt(el));
            else
                frac = 0;  left = obj.Last - el;
                info = Progress.fmt(el) + " elapsed";
            end
            if ~isnan(left) && left > 0, info = info + ", about " + Progress.fmt(left) + " left"; end
            if ~isempty(obj.Bar) && isvalid(obj.Bar)
                % waitbar's text takes character vectors, not strings
                waitbar(frac, obj.Bar, {char(obj.Title + ": " + string(label)), char(info)});
                drawnow;
            else
                fprintf("  [%s] %s (%s)\n", Progress.barText(frac), string(label), info);
            end
        end

        function done(obj)
            % finished: close the bar, keep the duration for next time
            el = toc(obj.T0);
            Progress.saveDuration(obj.Title, el);
            obj.closeBar();
            fprintf("%s: done in %s\n", obj.Title, Progress.fmt(el));
        end

        function delete(obj)
            obj.closeBar();                               % also when the job stops with an error
        end
    end

    methods (Access = private)
        function closeBar(obj)
            if ~isempty(obj.Bar) && isvalid(obj.Bar), delete(obj.Bar); end
            obj.Bar = [];
        end
    end

    methods (Static, Access = private)
        function tf = gui()
            tf = usejava("desktop") && feature("ShowFigureWindows");
        end

        function s = fmt(t)
            t = max(round(t), 0);
            if t >= 3600
                s = sprintf("%d:%02d:%02d", floor(t / 3600), floor(mod(t, 3600) / 60), mod(t, 60));
            else
                s = sprintf("%d:%02d", floor(t / 60), mod(t, 60));
            end
        end

        function s = barText(frac)
            n = round(20 * frac);
            s = string(repmat('#', 1, n)) + string(repmat('-', 1, 20 - n));
        end

        function f = file()
            f = fullfile(Simulink.fileGenControl("get", "CacheFolder"), "hpp_progress.mat");
        end

        function t = lastDuration(title)
            t = NaN;
            try
                S = load(Progress.file(), "D");
                if isKey(S.D, title), t = S.D(title); end
            catch
            end
        end

        function saveDuration(title, t)
            try
                D = dictionary(string.empty, double.empty);
                if isfile(Progress.file())
                    S = load(Progress.file(), "D");  D = S.D;
                end
                D(title) = t;
                save(Progress.file(), "D");
            catch
                % no cache folder or no write access: no estimate next time
            end
        end
    end
end
