function cases = sys_freq()
%SYS_FREQ  System frequency with the swing-equation grid (p.grid.sys, stage 6):
%   loss of generation (or load drop) of 3.5 % of a 2 GW Nordic-like system
%   at 40 s, with the HPP's frequency response off, LFSM only (default),
%   FSM, and lower inertia. 90 s runs.

    on = {"grid.sys.swing", 1, "sim.Tstop", 90};
    loss = [on, {"grid.sys.dist.dP", 0.035}];
    cases = caseList({
        "swing, no event"               on
        "loss 3.5 %, HPP no response"   [loss, {"hppc.Frqflag", 0}]
        "loss 3.5 %, LFSM"              loss
        "loss 3.5 %, FSM"               [loss, {"hppc.FSMflag", 1}]
        "loss 3.5 %, FSM, H 2 s"        [loss, {"hppc.FSMflag", 1, "grid.sys.H", 2}]
        "load drop 3.5 %, LFSM"         [on, {"grid.sys.dist.dP", -0.035}]
    });
end
