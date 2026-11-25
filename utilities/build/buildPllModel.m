function buildPllModel(folder)
    % models/ctrl/PLL.slx, argument pll (p.pll.asset or a meter's
    % p.pll.<controller>), shared by the asset controllers and FreqMeter.
    % In V_re, V_im; out th (the angle to inject along; pll.on = 0: the
    % angle of V), f [pu]. Both loops at the root, side by side, chosen per
    % run by Variant Source blocks on the outputs (the global variable
    % HPP_pll: runCase / runSweep from p.pll.type, projectStartup the
    % default; activation at update diagram; the inactive loop is not
    % compiled):
    %   MAF-PLL (HPP_pll == 1): SRF-PLL with moving-average prefilter
    %   Simple  (HPP_pll == 2): the minimal SRF-PLL
    % Created by buildHPP(All=true) if missing.
    mdl = "PLL";
    pllModelHeader(mdl, "pll", "pll.asset", "PLL, chosen per run (HPP_pll): MAF-PLL (1, SRF-PLL with " + ...
        "moving-average prefilter) or Simple (2, the minimal SRF-PLL). Argument pll: see params/pllParameters.m.");
    add_block("simulink/Sources/In1", mdl + "/V_re", "Port", "1", "Position", [30 133 60 147]);
    add_block("simulink/Sources/In1", mdl + "/V_im", "Port", "2", "Position", [30 173 60 187]);
    add_block("built-in/Subsystem", mdl + "/MAF-PLL");
    pllSubsystem(mdl + "/MAF-PLL", "pll", "ctrl");
    add_block("built-in/Subsystem", mdl + "/Simple");
    pllSimple(mdl + "/Simple", "pll");
    set_param(mdl + "/MAF-PLL", "Position", [160 40 300 120], "Description", ...
        "SRF-PLL with moving-average prefilter (HPP_pll = 1)");
    set_param(mdl + "/Simple", "Position", [160 200 300 280], "Description", "the minimal SRF-PLL (HPP_pll = 2)");
    for o = ["th", "f"]
        v = mdl + "/" + o + " variant";
        add_block("simulink/Signal Routing/Variant Source", v);
        set_param(v, "VariantControls", {'HPP_pll == 1', 'HPP_pll == 2'}, "VariantActivationTime", "update diagram");
    end
    set_param(mdl + "/th variant", "Position", [380 70 400 150]);
    set_param(mdl + "/f variant", "Position", [380 170 400 250]);
    add_block("simulink/Sinks/Out1", mdl + "/th", "Port", "1", "Position", [450 103 480 117]);
    add_block("simulink/Sinks/Out1", mdl + "/f", "Port", "2", "Position", [450 203 480 217]);
    ln(mdl, "V_re/1", "MAF-PLL/1");  ln(mdl, "V_im/1", "MAF-PLL/2");
    ln(mdl, "V_re/1", "Simple/1");   ln(mdl, "V_im/1", "Simple/2");
    ln(mdl, "MAF-PLL/1", "th variant/1");  ln(mdl, "Simple/1", "th variant/2");
    ln(mdl, "MAF-PLL/2", "f variant/1");   ln(mdl, "Simple/2", "f variant/2");
    ln(mdl, "th variant/1", "th/1", "th");  ln(mdl, "f variant/1", "f/1", "f");
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

%% ---------------------------------------------------------------- PLL
function pllSubsystem(blk, P, kind)
    % SRF-PLL as a loop diagram at the root of blk (the PLL model). P: the
    % parameter struct ("pll"). The arithmetic and its order are those of
    % the former MATLAB Function:
    %   vdq = V e^(-j th[k-1])                         rotate
    %   v   = moving average of vdq over Npre steps    MAF (prefilter, in the loop)
    %   e   = Im(v) / |v|, 0 below 0.05 pu             phase detector
    %   dw  = sat(kp e + sat(xi[k-1] + ki Ts e))       PI, +-5 Hz
    %   th  = wrap(th[k-1] + Ts dw)                    angle; th[k-1] starts at angle(V)
    %   f   = 1 + dw / w0
    % In V_re, V_im; out c, s (injection angle; on = 0: along V), f.
    % Layout: main path on y0 left to right, the angle feedback on y1 right to
    % left, start values below, the frequency branch on yf.
    y0 = 100;  y1 = 220;  yf = 300;  ys = 380;
    add_block("simulink/Sources/In1", blk + "/V_re", "Port", "1");  portAt(blk, "V_re", [20 y0-15]);
    add_block("simulink/Sources/In1", blk + "/V_im", "Port", "2");  portAt(blk, "V_im", [20 y0+15]);
    pb(blk, "ri2c", "V", [90 y0], "Input", "Real and imag");
    pb(blk, "prod", "rotate", [190 y0+10]);
    pllMaf(box(blk, "MAF", ["vdq", "v0"], "v", [270 y0-15 380 y0+45], ...
        "Prefilter: moving average of the voltage in the PLL frame over Npre steps (MAF-PLL)"), P);
    pllDetector(box(blk, "phase detector", "v", "e", [430 y0-15 540 y0+15], ...
        "e = Im(v) / |v| = sin(angle error); 0 below 0.05 pu (held in a deep dip)"));
    pllPi(box(blk, "PI", "e", "dw", [590 y0-15 700 y0+15], "dw = sat(kp e + xi), xi = sat(xi[k-1] + ki Ts e), +-5 Hz"), P);
    pllAngle(box(blk, "angle", ["dw", "th_prev"], "th", [770 y0-15 880 y0+45], "th = wrap(th[k-1] + Ts dw) to +-pi"), P);
    % feedback: th -> z^-1 (start: angle of V) -> e^(-j th) -> rotate
    pb(blk, "delay", "z^-1", [840 y1], "DelayLength", "1", "InitialConditionSource", "Input port");  flip(blk, "z^-1");
    pllExpj(box(blk, "e^(-j th)", "th", "e", [380 y1-15 480 y1+15], "unit phasor of the angle, conjugated"));
    flip(blk, "e^(-j th)");
    % start values: angle of V for z^-1, |V| for the prefilter
    pb(blk, "c2ma", "angle(V)", [500 ys], "Output", "Angle");
    pllStart(box(blk, "start |V|", "vin", "v0", [190 y0+60 260 y0+85], "start value of the prefilter: |V| (real)"));
    % frequency branch
    pk(blk, "w0", P + ".w0", [900 yf+20]);
    pb(blk, "div", "dw ÷ w0", [980 yf]);
    pb(blk, "bias", "1 +", [1050 yf], "Bias", "1");
    % PLL on or ideal: the PLL angle (pll.on = 1) or the angle of V (ideal, on = 0)
    pllInjection(box(blk, "PLL on or ideal", ["th", "V"], "th_inj", [960 y0-15 1080 y0+45], ...
        "the angle the converter injects along: the PLL angle; on = 0 (ideal): the angle of V"), P);
    add_block("simulink/Sinks/Out1", blk + "/th", "Port", "1");  portAt(blk, "th", [1140 y0]);
    add_block("simulink/Sinks/Out1", blk + "/f", "Port", "2");   portAt(blk, "f", [1140 yf]);
    ln(blk, "V_re/1", "V/1");  ln(blk, "V_im/1", "V/2");
    ln(blk, "V/1", "rotate/1", "V");
    ln(blk, "rotate/1", "MAF/1", "vdq");
    ln(blk, "V/1", "start |V|/1");  ln(blk, "start |V|/1", "MAF/2", "v0");
    ln(blk, "MAF/1", "phase detector/1", "v");
    ln(blk, "phase detector/1", "PI/1", "e");
    ln(blk, "PI/1", "angle/1", "dw");
    ln(blk, "angle/1", "z^-1/1", "th");
    ln(blk, "z^-1/1", "angle/2", "th[k-1]");
    ln(blk, "z^-1/1", "e^(-j th)/1");
    ln(blk, "e^(-j th)/1", "rotate/2");
    ln(blk, "V/1", "angle(V)/1");  ln(blk, "angle(V)/1", "z^-1/2", "angle(V)");
    ln(blk, "PI/1", "dw ÷ w0/1");  ln(blk, "w0/1", "dw ÷ w0/2");  ln(blk, "dw ÷ w0/1", "1 +/1");
    ln(blk, "angle/1", "PLL on or ideal/1");  ln(blk, "V/1", "PLL on or ideal/2");
    ln(blk, "PLL on or ideal/1", "th/1", "th");
    ln(blk, "1 +/1", "f/1", "f");
    % layout: the hand layout models/layout/pll.json
    f = fullfile(currentProject().RootFolder, "models", "layout", "pll.json");
    if isfile(f), applyLayout(blk, f); end
end

function pllMaf(s, P)
    % S = S[k-1] + vdq - vdq[k-Npre]; v = S / Npre; start values |V|
    portAt(s, "vdq", [20 60]);  portAt(s, "v0", [20 200]);  portAt(s, "v", [520 60]);
    pk(s, "Npre", P + ".Npre", [100 120]);  pk(s, "Npre 2", P + ".Npre", [380 90]);
    pb(s, "delay", "vdq[k-Npre]", [180 110], "DelayLengthSource", "Input port", "DelayLengthUpperLimit", "50", ...
        "InitialConditionSource", "Input port");
    pb(s, "sum", "S", [290 60], "Inputs", "++-");
    pb(s, "delay", "S[k-1]", [290 170], "DelayLength", "1", "InitialConditionSource", "Input port");  flip(s, "S[k-1]");
    pb(s, "gain", "Npre |V|", [180 220], "Gain", P + ".Npre");
    pb(s, "div", "S ÷ Npre", [430 60]);
    ln(s, "vdq/1", "S/2");  ln(s, "vdq/1", "vdq[k-Npre]/1");  ln(s, "Npre/1", "vdq[k-Npre]/2");  ln(s, "v0/1", "vdq[k-Npre]/3");
    ln(s, "vdq[k-Npre]/1", "S/3");
    ln(s, "S/1", "S ÷ Npre/1", "S");  ln(s, "Npre 2/1", "S ÷ Npre/2");  ln(s, "S ÷ Npre/1", "v/1");
    ln(s, "S/1", "S[k-1]/1");  ln(s, "S[k-1]/1", "S/1");
    ln(s, "v0/1", "Npre |V|/1");  ln(s, "Npre |V|/1", "S[k-1]/2");
end

function pllDetector(s)
    portAt(s, "v", [20 80]);  portAt(s, "e", [460 80]);
    pb(s, "c2ri", "Im", [110 80], "Output", "Imag");
    pb(s, "abs", "|v|", [110 150]);
    pb(s, "div", "Im ÷ |v|", [210 80]);
    pk(s, "0", "0", [270 190]);
    pb(s, "switch", "|v| > 0.05?", [330 100], "Criteria", "u2 > Threshold", "Threshold", "0.05");
    ln(s, "v/1", "Im/1");  ln(s, "v/1", "|v|/1");
    ln(s, "Im/1", "Im ÷ |v|/1");  ln(s, "|v|/1", "Im ÷ |v|/2");
    ln(s, "Im ÷ |v|/1", "|v| > 0.05?/1");  ln(s, "|v|/1", "|v| > 0.05?/2");  ln(s, "0/1", "|v| > 0.05?/3");
    ln(s, "|v| > 0.05?/1", "e/1");
end

function pllPi(s, P)
    % dw = sat(kp e + xi), xi = sat(xi[k-1] + ki Ts e)
    portAt(s, "e", [20 60]);  portAt(s, "dw", [520 60]);
    pb(s, "gain", "kp", [110 60], "Gain", P + ".kp");
    pb(s, "gain", "ki Ts", [110 150], "Gain", P + ".ki * " + P + ".Ts");
    pb(s, "sum", "xi[k-1] +", [220 160], "Inputs", "++");
    pb(s, "sat", "xi limit", [290 160], "UpperLimit", P + ".wlim", "LowerLimit", "-" + P + ".wlim");
    pb(s, "udelay", "xi[k-1]", [250 240], "InitialCondition", "0");  flip(s, "xi[k-1]");
    pb(s, "sum", "kp e + xi", [380 70], "Inputs", "++");
    pb(s, "sat", "dw limit", [440 60], "UpperLimit", P + ".wlim", "LowerLimit", "-" + P + ".wlim");
    ln(s, "e/1", "kp/1");  ln(s, "e/1", "ki Ts/1");
    ln(s, "ki Ts/1", "xi[k-1] +/2");  ln(s, "xi[k-1] +/1", "xi limit/1");
    ln(s, "xi limit/1", "xi[k-1]/1", "xi");  ln(s, "xi[k-1]/1", "xi[k-1] +/1");
    ln(s, "kp/1", "kp e + xi/1");  ln(s, "xi limit/1", "kp e + xi/2");
    ln(s, "kp e + xi/1", "dw limit/1");  ln(s, "dw limit/1", "dw/1");
end

function pllSimple(s, P)
    % simple SRF-PLL (the "Simple" choice of the PLL variant), the minimal
    % loop: vdq = V e^(-j th[k-1]), e = vq / max(|vdq|, 1e-3), dw = kp e +
    % xi (no limits), th = th[k-1] + Ts dw (not wrapped; start: the angle
    % of V), f = 1 + dw / w0; no prefilter, no dip hold. th_inj as in the
    % MAF-PLL (on = 0: along V). In V_re, V_im; out th, f.
    add_block("simulink/Sources/In1", s + "/V_re", "Port", "1");  portAt(s, "V_re", [20 45]);
    add_block("simulink/Sources/In1", s + "/V_im", "Port", "2");  portAt(s, "V_im", [20 75]);
    add_block("simulink/Sinks/Out1", s + "/th", "Port", "1");
    add_block("simulink/Sinks/Out1", s + "/f", "Port", "2");
    % layout as the textbook loop: dq transform -> PI -> integrator on one
    % row, theta[k-1] fed back below from the integrator to the transform;
    % frequency above, the start value (angle of V) below
    pb(s, "ri2c", "V", [80 60], "Input", "Real and imag");
    pllDq(box(s, "dq transform", ["V", "th_prev"], "e", [160 40 280 100], ...
        "vdq = V e^(-j th[k-1]); e = vq / max(|vdq|, 1e-3): the PI drives vq to zero"));
    pllPiPlain(box(s, "PI", "e", "dw", [340 55 440 85], "dw = kp e + xi, xi = xi[k-1] + ki Ts e (no limits)"), P);
    pllIntegrator(box(s, "integrator", ["dw", "th0"], ["th", "th_prev"], [500 40 620 100], ...
        "th = th[k-1] + Ts dw (not wrapped); th[k-1] starts at the angle of V"), P);
    pllInjection(box(s, "PLL on or ideal", ["th", "V"], "th_inj", [700 40 800 100], ...
        "the angle the converter injects along: the PLL angle; on = 0 (ideal): the angle of V"), P);
    portAt(s, "th", [860 55]);
    pb(s, "c2ma", "angle(V)", [400 160], "Output", "Angle");
    pk(s, "w0", P + ".w0", [480 -20]);
    pb(s, "div", "dw ÷ w0", [560 -30]);
    pb(s, "bias", "1 +", [640 -30], "Bias", "1");
    portAt(s, "f", [860 -30]);
    ln(s, "V_re/1", "V/1");  ln(s, "V_im/1", "V/2");
    ln(s, "V/1", "dq transform/1", "V");  ln(s, "dq transform/1", "PI/1", "e");
    ln(s, "PI/1", "integrator/1", "dw");  ln(s, "integrator/1", "PLL on or ideal/1", "th");
    ln(s, "V/1", "PLL on or ideal/2");  ln(s, "PLL on or ideal/1", "th/1", "th");
    ln(s, "integrator/2", "dq transform/2", "th[k-1]");          % the feedback
    ln(s, "V/1", "angle(V)/1");  ln(s, "angle(V)/1", "integrator/2", "angle(V)");
    ln(s, "PI/1", "dw ÷ w0/1");  ln(s, "w0/1", "dw ÷ w0/2");  ln(s, "dw ÷ w0/1", "1 +/1");  ln(s, "1 +/1", "f/1", "f");
end

function pllDq(s)
    % vdq = V e^(-j th[k-1]); e = vq / max(|vdq|, 1e-3) (the guard only
    % against division by zero)
    portAt(s, "V", [20 60]);  portAt(s, "th_prev", [20 160]);  portAt(s, "e", [520 80]);
    pb(s, "ma2c", "e^(j th)", [90 160], "Input", "Angle", "ConstantPart", "1");
    pb(s, "math", "conj", [170 160], "Operator", "conj");
    pb(s, "prod", "rotate", [250 70]);
    pb(s, "c2ri", "vq", [330 70], "Output", "Imag");
    pb(s, "abs", "|vdq|", [300 140]);
    pk(s, "1e-3", "1e-3", [300 190]);
    pb(s, "minmax", "max", [370 150], "Function", "max", "Inputs", "2");
    pb(s, "div", "vq ÷ |vdq|", [430 80]);
    ln(s, "th_prev/1", "e^(j th)/1");  ln(s, "e^(j th)/1", "conj/1");
    ln(s, "V/1", "rotate/1");  ln(s, "conj/1", "rotate/2");
    ln(s, "rotate/1", "vq/1", "vdq");  ln(s, "rotate/1", "|vdq|/1");
    ln(s, "|vdq|/1", "max/1");  ln(s, "1e-3/1", "max/2");
    ln(s, "vq/1", "vq ÷ |vdq|/1", "vq");  ln(s, "max/1", "vq ÷ |vdq|/2");
    ln(s, "vq ÷ |vdq|/1", "e/1");
end

function pllPiPlain(s, P)
    % dw = kp e + xi, xi = xi[k-1] + ki Ts e (no limits)
    portAt(s, "e", [20 60]);  portAt(s, "dw", [420 70]);
    pb(s, "gain", "kp", [110 60], "Gain", P + ".kp");
    pb(s, "gain", "ki Ts", [110 150], "Gain", P + ".ki * " + P + ".Ts");
    pb(s, "sum", "xi[k-1] +", [220 160], "Inputs", "++");
    pb(s, "udelay", "xi[k-1]", [220 240], "InitialCondition", "0");  flip(s, "xi[k-1]");
    pb(s, "sum", "kp e + xi", [330 70], "Inputs", "++");
    ln(s, "e/1", "kp/1");  ln(s, "e/1", "ki Ts/1");
    ln(s, "ki Ts/1", "xi[k-1] +/2");  ln(s, "xi[k-1] +/1", "xi[k-1]/1", "xi");  ln(s, "xi[k-1]/1", "xi[k-1] +/1");
    ln(s, "kp/1", "kp e + xi/1");  ln(s, "xi[k-1] +/1", "kp e + xi/2");  ln(s, "kp e + xi/1", "dw/1");
end

function pllIntegrator(s, P)
    % th = th[k-1] + Ts dw; th[k-1] = z^-1 th, start th0 (not wrapped:
    % cos / sin do not need it)
    portAt(s, "dw", [20 60]);  portAt(s, "th0", [20 180]);
    portAt(s, "th", [400 70]);  portAt(s, "th_prev", [400 160]);
    pb(s, "gain", "Ts", [90 60], "Gain", P + ".Ts");
    pb(s, "sum", "th[k-1] + Ts dw", [200 70], "Inputs", "++");
    pb(s, "delay", "z^-1", [260 160], "DelayLength", "1", "InitialConditionSource", "Input port");  flip(s, "z^-1");
    ln(s, "dw/1", "Ts/1");  ln(s, "Ts/1", "th[k-1] + Ts dw/2");
    ln(s, "th[k-1] + Ts dw/1", "th/1");  ln(s, "th[k-1] + Ts dw/1", "z^-1/1");  ln(s, "th0/1", "z^-1/2");
    ln(s, "z^-1/1", "th[k-1] + Ts dw/1", "th[k-1]");  ln(s, "z^-1/1", "th_prev/1");
end

function pllAngle(s, P)
    % th = mod(th[k-1] + Ts dw + pi, 2 pi) - pi
    portAt(s, "dw", [20 60]);  portAt(s, "th_prev", [20 130]);  portAt(s, "th", [520 70]);
    pb(s, "gain", "Ts", [90 60], "Gain", P + ".Ts");
    pb(s, "sum", "th[k-1] + Ts dw", [190 70], "Inputs", "++");
    pb(s, "bias", "+ pi", [250 70], "Bias", "pi");
    pk(s, "2 pi", "2*pi", [270 140]);
    pb(s, "math", "mod 2 pi", [330 80], "Operator", "mod");
    pb(s, "bias", "- pi", [420 70], "Bias", "-pi");
    ln(s, "dw/1", "Ts/1");  ln(s, "th_prev/1", "th[k-1] + Ts dw/1");  ln(s, "Ts/1", "th[k-1] + Ts dw/2");
    ln(s, "th[k-1] + Ts dw/1", "+ pi/1");  ln(s, "+ pi/1", "mod 2 pi/1");  ln(s, "2 pi/1", "mod 2 pi/2");
    ln(s, "mod 2 pi/1", "- pi/1");  ln(s, "- pi/1", "th/1");
end

function pllExpj(s)
    portAt(s, "th", [20 50]);  portAt(s, "e", [300 50]);
    pb(s, "ma2c", "e^(j th)", [90 50], "Input", "Angle", "ConstantPart", "1");
    pb(s, "math", "conj", [180 50], "Operator", "conj");
    ln(s, "th/1", "e^(j th)/1");  ln(s, "e^(j th)/1", "conj/1");  ln(s, "conj/1", "e/1");
end

function pllStart(s)
    portAt(s, "vin", [20 50]);  portAt(s, "v0", [300 50]);
    pb(s, "abs", "|V|", [90 50]);  pk(s, "0", "0", [100 100]);
    pb(s, "ri2c", "|V| + j0", [180 60], "Input", "Real and imag");
    ln(s, "vin/1", "|V|/1");  ln(s, "|V|/1", "|V| + j0/1");  ln(s, "0/1", "|V| + j0/2");  ln(s, "|V| + j0/1", "v0/1");
end

function pllInjection(s, P)
    % th_inj = th (PLL on), the angle of V (on = 0: ideal, along V)
    portAt(s, "th", [20 50]);  portAt(s, "V", [20 120]);  portAt(s, "th_inj", [400 60]);
    pb(s, "c2ma", "angle(V)", [110 120], "Output", "Angle");
    pk(s, "on", P + ".on", [180 80]);
    pb(s, "switch", "PLL on?", [290 60], "Criteria", "u2 > Threshold", "Threshold", "0.5");
    ln(s, "th/1", "PLL on?/1");  ln(s, "on/1", "PLL on?/2");  ln(s, "V/1", "angle(V)/1");
    ln(s, "angle(V)/1", "PLL on?/3");  ln(s, "PLL on?/1", "th_inj/1");
end
