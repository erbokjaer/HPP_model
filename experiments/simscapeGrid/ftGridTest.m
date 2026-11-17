function res = ftGridTest(N, opts)
%FTGRIDTEST  Build, save and time a Simscape Electrical test grid solved in
%   frequency-and-time mode (phasors around 50 Hz), to judge whether
%   standard library blocks are fast enough for the HPP network.
%
%   Network: 132 kV grid (Voltage Source (Three-Phase), Ssc 1800 MVA,
%   X/R 10) -- POC power sensor -- 33/132 kV HPP transformer -- 33 kV bus --
%   N radial feeders. Each feeder: 2 km pi line, then either
%     unitTr = true : a 0.69/33 kV Two-Winding Transformer (Three-Phase)
%                     and the converter at 0.69 kV, or
%     unitTr = false: the same transformer impedance as a series R-L at
%                     33 kV and the converter at 33 kV (no flux states).
%   Each converter is a Controlled Current Source (Three-Phase) with
%   magnitude/phase input. It measures its terminal voltage, turns it into
%   a phasor (space phasor at a fixed 50 Hz) and injects a fixed rms
%   current in phase with it, as the HPP asset models do.
%
%   res = ftGridTest(5)                      unit transformers
%   res = ftGridTest(5, unitTr=false)        series R-L instead
%   res = ftGridTest(5, Tstop=12, save=true) also save the .slx here
%
%   The model is named ftGrid_N<N>_<T|RL>. Timing: compile, then two
%   runs (2 s and Tstop) so the cost per simulated second excludes the
%   fixed overhead. Solver: local Backward Euler, 1 ms, fixed cost, one
%   nonlinear iteration (the network is linear).
    arguments
        N (1,1) double {mustBeInteger, mustBePositive} = 5
        opts.Ptot (1,1) double = 150e6
        opts.Tstop (1,1) double = 12
        opts.unitTr (1,1) logical = true
        opts.save (1,1) logical = true
        opts.phOffset (1,1) double = pi/2
        opts.lineType (1,1) string {mustBeMember(opts.lineType, ["pi", "rl", "rlc"])} = "rlc"
    end
    folder = fileparts(mfilename("fullpath"));
    if opts.unitTr, tag = "T"; else, tag = "RL"; end
    m = char(sprintf("ftGrid_N%d_%s", N, tag));
    buildModel(m, N, opts);
    if opts.save
        save_system(m, fullfile(folder, m + ".slx"));
    end

    res = struct("model", string(m), "N", N, "unitTr", opts.unitTr, "ok", false);
    try
        t0 = tic; set_param(m, "SimulationCommand", "update"); res.tCompile = toc(t0);
        set_param(m, "StopTime", "2");
        t0 = tic; sim(m); t2 = toc(t0);
        set_param(m, "StopTime", num2str(opts.Tstop));
        t0 = tic; out = sim(m); tT = toc(t0);
        res.perSimSecond = (tT - t2) / (opts.Tstop - 2);
        res.overhead     = t2 - 2*res.perSimSecond;
        res.Ppoc = out.P_poc.Data(end);  res.Qpoc = out.Q_poc.Data(end);
        res.Pc1  = out.P_c1.Data(end);   res.Qc1  = out.Q_c1.Data(end);
        w = out.P_poc.Time > opts.Tstop - 1;                 % ripple over the last second
        res.ripplePpoc = max(out.P_poc.Data(w)) - min(out.P_poc.Data(w));
        res.ok = true;
    catch e
        res.err = string(e.message);
        for c = 1:numel(e.cause), res.err = res.err + " | " + string(e.cause{c}.message); end
    end
    close_system(m, 0);
end

% ---------------------------------------------------------------------
function buildModel(m, N, opts)
    load_system("ee_lib");
    all = string(strrep(find_system('ee_lib','LookUnderMasks','all','FollowLinks','on','BlockType','SimscapeBlock'), newline, ' '));
    raw = find_system('ee_lib','LookUnderMasks','all','FollowLinks','on','BlockType','SimscapeBlock');
    lib = @(n) raw{all == "ee_lib/" + n};
    ref = sprintf('ee_lib/Connectors &\nReferences/Electrical Reference');
    if bdIsLoaded(m), close_system(m, 0); end
    new_system(m);
    H = @(b) get_param([m '/' b], 'PortHandles');
    A = @(a, b) add_line(m, a, b, 'autorouting', 'on');
    trOpts = {'magnetizing_reactance_option', 'ee.enum.transformer_magnetizingReactance.include', 'pu_Xm', '500', ...
              'magnetizing_resistance_option', 'ee.enum.transformer_magnetizingResistance.include', 'pu_Rm', '500', ...
              'leakage_reactance_option', 'ee.enum.transformer_leakage.include', ...
              'Winding1Connection', 'ee.enum.windingconnection.Yg', 'Winding2Connection', 'ee.enum.windingconnection.Yg', ...
              'FRated', '50'};

    % ---- grid, POC sensor, HPP transformer, solver ----
    add_block(lib("Sources/Voltage Source (Three-Phase)"), [m '/grid'], 'vline_rms', '132e3', 'freq', '50', ...
        'impedance_option', '1', 'SShortCircuit', '1800e6', 'XR', '10', 'Position', [1300 100 1380 180]);
    add_block(ref, [m '/g0'], 'Position', [1330 230 1350 250]);
    add_block(lib("Sensors & Transducers/Power Sensor (Three-Phase)"), [m '/PQ poc'], 'f0', '50', 'Position', [1180 100 1240 180]);
    add_block(lib("Passive/Transformers/Two-Winding Transformer (Three-Phase)"), [m '/T hpp'], 'SRated', '250e6', ...
        'VRated1', '33e3', 'VRated2', '132e3', 'pu_Xl1', '0.06', 'pu_Xl2', '0.06', 'pu_Rw1', '0.0015', 'pu_Rw2', '0.0015', ...
        trOpts{:}, 'Position', [1040 100 1120 180]);
    add_block('nesl_utility/Solver Configuration', [m '/solver'], 'EquationFormulation', 'NE_FREQUENCY_TIME_EF', ...
        'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', 'LocalSolverSampleTime', '1e-3', ...
        'DoFixedCost', 'on', 'MaxNonlinIter', '1', 'Position', [960 230 1020 260]);
    A(H('grid').LConn(1), H('g0').LConn(1));
    A(H('grid').RConn(1), H('PQ poc').RConn(1));
    A(H('PQ poc').LConn(1), H('T hpp').RConn(1));
    bus = H('T hpp').LConn(1);
    A(H('solver').RConn(1), bus);
    logPQ(m, 'PQ poc', 'poc', [1180 30]);
    add_block('simulink/Sources/Clock', [m '/clock'], 'Position', [20 20 40 40]);

    % ---- feeders with one converter each ----
    Su = 1.1*opts.Ptot/N;                       % [VA] unit transformer rating
    if opts.unitTr
        Ik = opts.Ptot / N / (sqrt(3) * 690);   % [A rms] per converter at 0.69 kV
    else
        Ik = opts.Ptot / N / (sqrt(3) * 33e3);  % [A rms] per converter referred to 33 kV
        Zb = 33e3^2 / Su;                       % transformer series impedance referred to 33 kV
        Ru = 2*0.004 * Zb;                      %   R = Rw1 + Rw2, X = Xl1 + Xl2 (pu on Su)
        Lu = 2*0.03  * Zb / (2*pi*50);
    end
    for k = 1:N
        y = 100 + (k-1)*160;
        s = sprintf('%d', k);
        if opts.lineType == "pi"
            add_block(lib("Passive/Lines/Transmission Line (Three-Phase)"), [m '/line ' s], 'prm', ...
                'ee.enum.lines.parameterization3ph.posZeroSeqParam', 'length', '2', 'freq', '50', ...
                'Rpos', '0.10', 'Lpos', '0.38', 'Cpos', '0.25', 'Rzero', '0.3', 'Lzero', '1.2', 'Czero', '0.20', ...
                'Position', [880 y 940 y+50]);
        else            % same series impedance; "rlc": plus half the charging (2 km x 0.25 uF/km) at each end
            add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), [m '/line ' s], 'component_structure', ...
                'ee.enum.rlc.structure.SeriesRL', 'R', '0.2', 'L', '0.76e-3', 'Position', [880 y 940 y+50]);
            if opts.lineType == "rlc"
                for e = ["a", "b"]
                    c = [m '/C' char(e) ' ' s];
                    add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), c, 'component_structure', ...
                        'ee.enum.rlc.structure.C', 'C', '0.25e-6', 'Position', [880+40*(e=="b") y+70 910+40*(e=="b") y+100]);
                    add_block(lib("Connectors & References/Grounded Neutral (Three-Phase)"), [c ' gnd'], ...
                        'Position', [880+40*(e=="b") y+120 900+40*(e=="b") y+135]);
                    add_line(m, get_param(c,'PortHandles').RConn(1), get_param([c ' gnd'],'PortHandles').LConn(1), 'autorouting', 'on');
                end
            end
        end
        if opts.unitTr
            add_block(lib("Passive/Transformers/Two-Winding Transformer (Three-Phase)"), [m '/Tu ' s], ...
                'SRated', num2str(Su), 'VRated1', '690', 'VRated2', '33e3', 'pu_Xl1', '0.03', 'pu_Xl2', '0.03', ...
                'pu_Rw1', '0.004', 'pu_Rw2', '0.004', trOpts{:}, 'Position', [720 y 800 y+60]);
        else
            add_block(lib("Passive/RLC Assemblies/RLC (Three-Phase)"), [m '/Tu ' s], 'component_structure', ...
                'ee.enum.rlc.structure.SeriesRL', 'R', num2str(Ru), 'L', num2str(Lu), 'Position', [720 y 800 y+60]);
        end
        add_block(lib("Sources/Controlled Current Source (Three-Phase)"), [m '/conv ' s], 'input_option', ...
            'ee.enum.inputType.singleFreqAC', 'FRated', '50', 'Position', [460 y 540 y+60]);
        add_block(ref, [m '/gc ' s], 'Position', [480 y+90 500 y+110]);
        add_block(lib("Sensors & Transducers/Phase Voltage Sensor (Three-Phase)"), [m '/Vs ' s], 'Position', [600 y+80 640 y+120]);
        add_block(lib("Connectors & References/Grounded Neutral (Three-Phase)"), [m '/gv ' s], 'Position', [600 y+130 630 y+145]);
        A(bus, H(['line ' s]).RConn(1));
        A(H(['line ' s]).LConn(1), H(['Tu ' s]).RConn(1));
        if opts.lineType == "rlc"
            A(H(['Ca ' s]).LConn(1), H(['line ' s]).LConn(1));
            A(H(['Cb ' s]).LConn(1), H(['line ' s]).RConn(1));
        end
        term = H(['Tu ' s]).LConn(1);
        if k == 1   % converter 1: power sensor between converter and terminal
            add_block(lib("Sensors & Transducers/Power Sensor (Three-Phase)"), [m '/PQ c1'], 'f0', '50', 'Position', [580 y 620 y+50]);
            A(H(['conv ' s]).RConn(1), H('PQ c1').LConn(1));
            A(H('PQ c1').RConn(1), term);
            logPQ(m, 'PQ c1', 'c1', [580 y-60]);
        else
            A(H(['conv ' s]).RConn(1), term);
        end
        A(H(['conv ' s]).LConn(3), H(['gc ' s]).LConn(1));
        A(H(['Vs ' s]).LConn(1), term);
        A(H(['Vs ' s]).RConn(2), H(['gv ' s]).LConn(1));
        % Simulink side: voltage phasor -> current magnitude / phase aligned with it
        add_block('nesl_utility/PS-Simulink Converter', [m '/v ' s], 'Position', [300 y+80 320 y+100]);
        A(H(['Vs ' s]).RConn(1), H(['v ' s]).LConn(1));
        addPhasorFcn([m '/phasor ' s], [180 y 260 y+60]);
        add_line(m, ['v ' s '/1'], ['phasor ' s '/1'], 'autorouting', 'on');
        add_line(m, 'clock/1', ['phasor ' s '/2'], 'autorouting', 'on');
        add_block('simulink/Discrete/Memory', [m '/mem ' s], 'Position', [280 y+20 300 y+40]);
        add_block('simulink/Math Operations/Bias', [m '/off ' s], 'Bias', num2str(opts.phOffset), 'Position', [320 y+20 350 y+40]);
        add_block('nesl_utility/Simulink-PS Converter', [m '/ph ' s], 'Unit', 'rad', 'Position', [380 y+20 400 y+40]);
        add_block('simulink/Sources/Constant', [m '/Iset ' s], 'Value', num2str(Ik), 'Position', [320 y-20 380 y]);
        add_block('nesl_utility/Simulink-PS Converter', [m '/I ' s], 'Unit', 'A', 'Position', [400 y-20 420 y]);
        add_line(m, ['phasor ' s '/2'], ['mem ' s '/1']);
        add_line(m, ['mem ' s '/1'], ['off ' s '/1']);
        add_line(m, ['off ' s '/1'], ['ph ' s '/1']);
        add_line(m, ['Iset ' s '/1'], ['I ' s '/1']);
        A(H(['I ' s]).RConn(1), H(['conv ' s]).LConn(1));
        A(H(['ph ' s]).RConn(1), H(['conv ' s]).LConn(2));
        add_block('simulink/Sinks/Terminator', [m '/tm ' s], 'Position', [280 y-30 290 y-20]);
        add_line(m, ['phasor ' s '/1'], ['tm ' s '/1']);
    end
    set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1e-3', 'StopTime', num2str(opts.Tstop));
end

function logPQ(m, sensor, tag, xy)
    H = @(b) get_param([m '/' b], 'PortHandles');
    for q = ["P", "Q"]
        c = [char(q) ' ' tag ' out'];  w = [char(q) ' ' tag ' log'];
        dx = 60*(q == "Q");
        add_block('nesl_utility/PS-Simulink Converter', [m '/' c], 'Position', [xy(1)+dx xy(2) xy(1)+dx+20 xy(2)+20]);
        add_block('simulink/Sinks/To Workspace', [m '/' w], 'VariableName', char(q + "_" + tag), 'SaveFormat', 'Timeseries', ...
            'Position', [xy(1)+dx xy(2)-40 xy(1)+dx+50 xy(2)-20]);
        add_line(m, H(sensor).LConn(2 + (q == "Q")), H(c).LConn(1), 'autorouting', 'on');
        add_line(m, [c '/1'], [w '/1']);
    end
end

function addPhasorFcn(blk, pos)
    add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = sprintf(['function [vmag, vang] = phasor(v, t)\n' ...
        '%% space phasor of the phase voltages at a fixed 50 Hz: rms line voltage and angle\n' ...
        'a = complex(-0.5, sqrt(3)/2);\n' ...
        's = 2/3*(v(1) + a*v(2) + a^2*v(3)) * exp(complex(0, -2*pi*50*t));\n' ...
        'vmag = abs(s)*sqrt(3/2);\n' ...
        'vang = angle(s);\n' ...
        'end\n']);
end
