function cases = comm_delay()
%COMM_DELAY  Communication delays: a +30 Mvar Q_ref step at 40 s with different
%   delays between the control levels (p.comm). The set changes comm.*
%   values, so p.comm.delays = "auto" compiles variable-length delays (up to
%   p.comm.maxDelay = 0.5 s) and the whole set runs on one compiled model
%   (Fast Restart or Rapid Accelerator).

    q = {"grid.Qstep.dQ", 30e6};
    cases = caseList({
        "HPPC-plant 50 ms (default)"   q
        "HPPC-plant 0 ms"              [q, {"comm.hppc_to_plant", 0}]
        "HPPC-plant 100 ms"            [q, {"comm.hppc_to_plant", 0.100}]
        "HPPC-plant 200 ms"            [q, {"comm.hppc_to_plant", 0.200}]
        "all delays x2"                [q, {"comm.tso", 0.400, "comm.meas_hppc", 0.040, "comm.hppc_to_plant", 0.100, ...
                                            "comm.meas_plant", 0.040, "comm.plant_to_asset", 0.040}]
        "no delays"                    [q, {"comm.tso", 0, "comm.meas_hppc", 0, "comm.hppc_to_plant", 0, ...
                                            "comm.meas_plant", 0, "comm.plant_to_asset", 0}]
    });
end
