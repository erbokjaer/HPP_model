function cases = dq_network()
%DQ_NETWORK  Validation of the dq network (stage 8) against the nodal one.
%   Run the set twice, runSweep("dq_network", Network="nodal") and
%   runSweep("dq_network", Network="dq"), and compare: in steady state the
%   two are the same network; in transients the dq network adds the
%   electromagnetic dynamics of cables and transformers. Events at 40 s,
%   after the plant has settled; all cases 60 s, at SCR 10 and SCR 3.

    pstep = {"grid.Pstep.dP", -30e6};
    qstep = {"grid.Qstep.dQ", 30e6};
    dip   = {"grid.dip.depth", 0.1, "grid.dip.t0", 40, "grid.dip.dur", 0.15};
    s3    = {"grid.SCR", 3};
    cases = caseList({
        "nominal"           {}
        "P_ref -30 MW"      pstep
        "Q_ref +30 Mvar"    qstep
        "dip 10 %"          dip
        "nominal, SCR3"     s3
        "P_ref -30 MW, SCR3" [pstep, s3]
        "Q_ref +30 Mvar, SCR3" [qstep, s3]
        "dip 10 %, SCR3"    [dip, s3]
    });
    cases.("sim.Tstop")(:) = 60;
end
