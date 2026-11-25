function buildFreqMeterModel(folder)
    % models/ctrl/FreqMeter.slx: frequency meter of a controller: the PLL model
    % (Model block, argument pll = m) and the one-cycle average of its
    % frequency and RoCoF (pllAverage). Argument m (p.pll.<controller>).
    % In V_re, V_im, f_sys (used with m.on = 0); out f [pu], RoCoF [Hz/s].
    mdl = "FreqMeter";
    pllModelHeader(mdl, "m", "pll.hppc", "Frequency meter: PLL, frequency averaged over Navg steps " + ...
        "(m.on = 0: f_sys), RoCoF over Nroc steps. Argument m: see params/pllParameters.m.");
    y = 100;
    add_block("simulink/Sources/In1", mdl + "/V_re", "Port", "1");   portAt(mdl, "V_re", [20 y-20]);
    add_block("simulink/Sources/In1", mdl + "/V_im", "Port", "2");   portAt(mdl, "V_im", [20 y+20]);
    add_block("simulink/Sources/In1", mdl + "/f_sys", "Port", "3");  portAt(mdl, "f_sys", [20 y+110]);
    add_block("simulink/Ports & Subsystems/Model", mdl + "/PLL", "ModelName", "PLL", "Position", [120 y-45 240 y+45]);
    Simulink.ModelReference.refresh(mdl + "/PLL");
    setArg(mdl + "/PLL", "pll", "m");
    add_block("simulink/Sinks/Terminator", mdl + "/th unused", "Position", [290 y-40 310 y-20], "ShowName", "off");
    pllAverage(box(mdl, "f average, RoCoF", ["fp", "f_sys"], ["f", "RoCoF"], [360 y+10 490 y+90], ...
        "f = moving average of the PLL frequency over Navg steps (on = 0: f_sys); RoCoF over Nroc steps"), "m");
    add_block("simulink/Sinks/Out1", mdl + "/f", "Port", "1");      portAt(mdl, "f", [560 y+30]);
    add_block("simulink/Sinks/Out1", mdl + "/RoCoF", "Port", "2");  portAt(mdl, "RoCoF", [560 y+70]);
    ln(mdl, "V_re/1", "PLL/1");  ln(mdl, "V_im/1", "PLL/2");
    ln(mdl, "PLL/1", "th unused/1");
    ln(mdl, "PLL/2", "f average, RoCoF/1", "fp");  ln(mdl, "f_sys/1", "f average, RoCoF/2", "f_sys");
    ln(mdl, "f average, RoCoF/1", "f/1", "f");  ln(mdl, "f average, RoCoF/2", "RoCoF/1", "RoCoF");
    save_system(mdl, fullfile(folder, mdl + ".slx"));
    close_system(mdl);
    registerInProject(fullfile(folder, mdl + ".slx"));
end

function pllAverage(s, P)
    % F = F[k-1] + fp - fp[k-Navg] (start fp Navg), f = F / Navg (on = 0: f_sys);
    % RoCoF = (f - f[k-Nroc]) f0 / (Nroc Ts) (start f_sys)
    portAt(s, "fp", [20 60]);  portAt(s, "f_sys", [20 300]);  portAt(s, "f", [640 70]);  portAt(s, "RoCoF", [640 330]);
    pk(s, "Navg", P + ".Navg", [100 120]);  pk(s, "Navg 2", P + ".Navg", [380 100]);
    pb(s, "delay", "fp[k-Navg]", [180 110], "DelayLengthSource", "Input port", "DelayLengthUpperLimit", "50", ...
        "InitialConditionSource", "Input port");
    % F = fp - fp[k-Navg] + F[k-1]: a round sum, fp from above, the delayed
    % sample from the left, the state from below (hand layout)
    pb(s, "sum", "F", [290 60], "Inputs", "+-+", "IconShape", "round");
    pb(s, "delay", "F[k-1]", [290 170], "DelayLength", "1", "InitialConditionSource", "Input port");  flip(s, "F[k-1]");
    pb(s, "gain", "Navg fp", [180 220], "Gain", P + ".Navg");
    pb(s, "div", "F ÷ Navg", [430 70]);
    pk(s, "on", P + ".on", [450 140]);
    pb(s, "switch", "PLL on?", [530 80], "Criteria", "u2 > Threshold", "Threshold", "0.5");
    pk(s, "Nroc", P + ".Nroc", [300 360]);  pk(s, "Nroc Ts", P + ".Nroc * " + P + ".Ts", [460 380]);
    pb(s, "delay", "f[k-Nroc]", [380 340], "DelayLengthSource", "Input port", "DelayLengthUpperLimit", "1000", ...
        "InitialConditionSource", "Input port");
    pb(s, "sum", "df", [460 320], "Inputs", "+-");
    pb(s, "gain", "f0", [510 320], "Gain", P + ".f0");
    pb(s, "div", "÷ Nroc Ts", [570 330]);
    ln(s, "fp/1", "F/1");  ln(s, "fp/1", "fp[k-Navg]/1");  ln(s, "Navg/1", "fp[k-Navg]/2");  ln(s, "fp/1", "fp[k-Navg]/3");
    ln(s, "fp[k-Navg]/1", "F/2");  ln(s, "F/1", "F[k-1]/1");  ln(s, "F[k-1]/1", "F/3");
    ln(s, "fp/1", "Navg fp/1");  ln(s, "Navg fp/1", "F[k-1]/2");
    ln(s, "F/1", "F ÷ Navg/1", "F");  ln(s, "Navg 2/1", "F ÷ Navg/2");
    ln(s, "F ÷ Navg/1", "PLL on?/1");  ln(s, "on/1", "PLL on?/2");  ln(s, "f_sys/1", "PLL on?/3");
    ln(s, "PLL on?/1", "f/1", "f");
    ln(s, "PLL on?/1", "f[k-Nroc]/1");  ln(s, "Nroc/1", "f[k-Nroc]/2");  ln(s, "f_sys/1", "f[k-Nroc]/3");
    ln(s, "PLL on?/1", "df/1");  ln(s, "f[k-Nroc]/1", "df/2");
    ln(s, "df/1", "f0/1");  ln(s, "f0/1", "÷ Nroc Ts/1");  ln(s, "Nroc Ts/1", "÷ Nroc Ts/2");  ln(s, "÷ Nroc Ts/1", "RoCoF/1");
    % layout: the hand layout models/layout/rocof.json
    f = fullfile(currentProject().RootFolder, "models", "layout", "rocof.json");
    if isfile(f), applyLayout(s, f); end
end
