# HPP_model: architecture of the hierarchical plant model

Status: **stages 1–4 built** (2026-11-16): structure, N instances, communication delays, parameter structure, the phasor network, the IEC 61400-27-1 Type 4A wind turbine, the WECC REEC_B / REEC_C + REGC_A models for PV and BESS, closed-loop Q control in every asset controller, and REPC_A plant controllers (WPPC, SPPC, BESSC) with their own P, frequency and Q/V loops. Since 2026-11-17 also: two network models chosen per run (nodal and Simscape Electrical, section 5) and faster runs with Fast Restart, where almost all parameters change without recompiling (section 11). Stage 5 started (2026-11-17): the HPPC closes the P loop at the POC with the frequency response of the plant (central control) and, since 2026-11-18, a Q, voltage or power factor loop at the POC after the Danish grid code, with voltage control as a cascade over the plant controllers (section 8).
Values marked *typical* are starting points to be replaced by project data.

## 1. Decisions

| Topic | Decision |
|---|---|
| Simulation type in Simulink | **Three tiers behind one asset–network interface** (section 14): **RMS** (positive-sequence phasor, today's model, for sweeps), **dq** (network as a state-space model in a 50 Hz dq frame, for frequency, PLL, GFM and weak-grid studies; planned) and **EMT** (Simscape time mode or PSCAD with the controller DLLs, for validation). |
| Which controllers become DLLs | **All controllers**: HPP controller, the three plant controllers and the three asset controllers. They are our own models, not vendor DLLs. |
| Number of assets | **Per plant: `nFeeders` × `nPerFeeder` units**, each its own asset instance (converter model and controller) at its own network node, so N = nFeeders · nPerFeeder. Default layout: wind 1 × 2, PV 2 × 1, BESS 1 × 1 (N = 2 / 2 / 1). |
| Asset model standards | Wind: **IEC 61400-27-1 Ed. 2, Type 4** (full converter). PV and BESS: **WECC second-generation generic models** (REGC / REEC / REPC). |
| Collector system and grid | Typical values (section 5). Two network models with the same results, chosen per run: an own nodal phasor solver in plain Simulink (default, fast) and a Simscape Electrical model in frequency-and-time mode (component-wise, slower). Specialized Power Systems is not used. |
| Communication delays | Included from the start, typical values (section 6). |

## 2. Hierarchy

```
 TSO / market:  P_ref, Q_ref / V_ref, frequency response mode
        │
 HPPC  (HPP controller)                                   Ts = 100 ms
        │  P/Q set-points per plant         ▲ plant P, Q, availability, SOC
   ┌────┴──────────────┬───────────────────────┐
 WPPC               SPPC                     BESSC (plant controller / EMS)   Ts = 20 ms
 (WECC REPC_A;       (WECC REPC_A)            (WECC REPC_A + SOC management)
  IEC WPP control
  later)
   │ P/Q per asset     │                        │
 WT controller ×N    PV controller ×N         BESS controller ×N            Ts = sim step
 (IEC 61400-27-1     (WECC REEC_B)            (WECC REEC_C: incl. SOC / BMS limits)
  Type 4 control)
   │ i_p, i_q cmd      │                        │
 ── plant side (not DLLs) ─────────────────────────────────────────────────────────────
 WT generator ×N     PV generator ×N          BESS generator + battery ×N
 (IEC Type 4 gen.     (WECC REGC_A)            (WECC REGC_A + energy / SOC model)
  system, opt. 4B mechanics)
   └──── unit transformers ── collector feeders ── 33 kV busbar ── main transformer ── POC ── grid
```

**Why the battery has its own asset controller.** The BESS plant controller (EMS) decides *how much* power the battery delivers and manages SOC over minutes to hours. The asset controller decides *how* it is delivered: current and power control, converter limits and the BMS limits for SOC and current. Different time scales and functions, and it keeps the three asset types structurally the same. In the WECC set this split is REPC (plant) and REEC_C (asset).

**Controller and plant side.** Every controller is a separate referenced model and a future DLL. Everything below the dashed line is the "plant" in Simulink; in PSCAD it is replaced by PSCAD's own EMT models, so it is never code-generated.

## 3. Model files

| File | Contents | DLL |
|---|---|---|
| `models/HPP.slx` | Top model: HPPC, the three plants, network, grid, logging | – |
| `models/ctrl/HPPC.slx` | HPP controller | yes |
| `models/ctrl/WPPC.slx`, `SPPC.slx`, `BESSC.slx` | Plant controllers; vector ports of width N | yes |
| `models/ctrl/WT_ctrl.slx`, `PV_ctrl.slx`, `BESS_ctrl.slx` | Asset controllers; one instance per asset | yes |
| `models/assets/WT_asset.slx`, `PV_asset.slx`, `BESS_asset.slx` | Asset plant models: controller instance + generator/converter (+ mechanics / battery) | – |
| `Network` block inside `models/HPP.slx` | Phasor network: Variant Subsystem with the nodal and the Simscape model (section 5) | – |
| "link" blocks inside `models/HPP.slx` | Unit conversion, measurement memory and communication delay (discrete, whole steps) per signal path | – |

**N assets (decided in stage 1).** Each asset type is one parameterised file, instantiated N times by the build script `utilities/build/buildHPP.m`, which generates `HPP.slx` with N Model blocks per type and wires them. Instance k gets the model argument `ap = p.<type>.ap(k)` (asset parameters including its controller's `par`). `HPP.slx` stores the N it was built for, and `runCase` / `runSweep` stop if the parameters differ. A For Each subsystem was not used: the build script is simpler to debug, and every instance stays a normal Model block, which code generation and PSCAD handle the same way.

The plant controllers use fixed-width vector ports (width N, from their model workspace), so a DLL is built for one N. Changing N means rerunning `buildHPP(All=true)`, which is acceptable.

**Ownership (2026-11-19).** `HPP.slx` is always generated by `buildHPP` and never edited by hand. The controller and asset models (`models/ctrl`, `models/assets`) are the source and are developed in the Simulink editor; the templates in `buildHPP.m` only create missing models. `buildHPP` never overwrites an existing one unless asked explicitly (`buildHPP(All=true, Overwrite=true, Only=[...])`; `Force=true` alone errors). The interface `HPP.slx` expects (port names and order, model argument and its default) is defined once in `utilities/build/modelInterfaces.m`, used by the HPPC template and wiring and by `scripts/checkModels.m`. `checkModels` loads the models without UI and reports, with block path and suggested fix: block parameters that use `par` / `ap` with anything but + − × ÷, parentheses, numbers and field access (not tunable in compiled code; MATLAB Function code is exempt), ports that differ from `modelInterfaces`, and `par` / `ap` fields that `HPP_parameters()` does not define. It runs at the start of `buildHPP` and of a Rapid Accelerator build (warning; error with `Strict=true`).

**Diagram layout (2026-11-16).** `HPP.slx` is organised as six subsystems (TSO, HPP controller, Wind / PV / BESS plant, Network, Logging) connected by virtual buses in SI units. Each subsystem converts to its controller's per-unit base and applies the communication delays of the links it receives. Controller models keep flat scalar ports for the DLL interface; Bus Selectors and Bus Creators sit around them. Forward signals are lines; feedback and logging signals are colour-coded Goto/From pairs (orange plant measurements, green network, grey logging). Each signal path between levels is one masked "link" block (gain, memory, discrete delay). The network is a masked subsystem inside `HPP.slx` rather than a separate `models/network/` file, since its size depends on N and it is not a DLL. The reorganisation changed no results (largest relative difference 6e-13 on all logged signals). HPPC inports are ordered per source (TSO, POC, wind, PV, BESS).

**Aggregation.** One unit can stand for `nAgg` identical physical units at the same place (its unit transformer is scaled with `nAgg`), and one modelled feeder for `cable.nPar` identical parallel feeders (`p.net.<type>.cable`). The default layout keeps the plant sizes with few converter models: wind 2 units × 15 turbines of 5 MW on one feeder that stands for 15 identical feeders (nPar = 15), PV 2 feeders with one unit of 5 inverter stations each, one 30 MW BESS. A full layout (e.g. wind 15 feeders × 2 turbines, PV 2 feeders × 5 stations, all `nAgg` and `nPar` = 1: 41 converter models, 85 nodes) gives practically the same steady state (P_poc 149.46 against 149.40 MW in a static check with all units at the same power).

## 4. Interfaces

Every controller follows the same pattern:
- per unit on **its own base**: HPPC on the POC base, plant controllers on the plant rating, asset controllers on the asset rating;
- all ports scalar or fixed-width vectors of `double`;
- tunable parameters as one struct `par`, passed as a model argument;
- fixed sample time `Ts`; base and `Ts` not tunable.
The W/var to pu conversions sit outside the controllers: in `HPP.slx` in Simulink, and in the PSCAD case in PSCAD.

Signs: generator convention, + P = export / discharge, + Q = export (capacitive). SOC in 0–1.

### 4.1 HPPC  (base: POC rating, `p.grid.Pbase`)

| In | Unit | Out | Unit |
|---|---|---|---|
| `P_ref`, `Q_ref` or `V_ref` from TSO | pu | `P_set_wpp`, `Q_set_wpp` | pu |
| `f_meas` | pu (of 50 Hz) | `P_set_spp`, `Q_set_spp` | pu |
| `P_poc`, `Q_poc`, `V_poc` (measured) | pu | `P_set_bess`, `Q_set_bess` | pu |
| `P_avail_wpp`, `P_avail_spp` | pu | | |
| `P_chg_max_bess`, `P_dis_max_bess`, `SOC_bess` | pu, – | | |
| `P_wpp`, `Q_wpp`, `P_spp`, `Q_spp`, `P_bess`, `Q_bess` | pu | | |
| `Q_sat_wpp`, `Q_sat_spp`, `Q_sat_bess` (plant Q order at its limit) | +1 / 0 / −1 | | |

Functions, stage 5: P control at POC with ramp limits, Q / V / PF control at POC, frequency response (FSM / LFSM-O/U), dispatch between plants, POC limits. Built so far (section 8, stage 5): set-point ramp limit, LFSM-O / LFSM-U / FSM, export limit, closed-loop P, SOC-aware P dispatch, Q / voltage (slope) / power factor control, after Energinet's DK1 requirements (V_ref and PF are parameters, `p.hppc.Vref` / `PF`, not TSO signals yet); voltage control as a cascade over the plant controllers, POC Q limit, anti-windup over the plant limits (stage 5 step 4). `f_meas` is measured locally (no communication delay). Port order (20 inputs): TSO (P_ref, Q_ref, f_meas), POC (P, Q, V), then per plant its measurements with `Q_sat` last (wind: P_avail, P, Q, Q_sat; PV likewise; BESS: P_chg_max, P_dis_max, SOC, P, Q, Q_sat).

### 4.2 Plant controllers  (base: plant rating; vectors of width N)

| In | Unit | Out | Unit |
|---|---|---|---|
| `P_set`, `Q_set` from HPPC | pu | `P_set_k`, `Q_set_k` (N) | pu |
| `P_meas`, `Q_meas`, `V_meas` at plant connection | pu | `P_avail`, `P_plant`, `Q_plant` to HPPC | pu |
| `P_avail_k` (N) from assets | pu | BESSC only: `P_chg_max`, `P_dis_max`, `SOC` | pu, – |
| BESSC only: `SOC_k` (N) | – | `Q_sat` to HPPC (last output): +1 / −1 when the Q order is at its upper / lower limit | – |
| `f_meas` grid frequency (last input) | pu | | |

### 4.3 Asset controllers  (base: asset rating)

| In | Unit | Out | Unit |
|---|---|---|---|
| `P_set`, `Q_set` from plant controller | pu | `ip_cmd`, `iq_cmd` to generator model | pu |
| `V_term` (magnitude), `P_meas`, `Q_meas` | pu | `P_avail` | pu |
| WT only: `P_wind_avail` (from wind input), `omega_gen` (4B) | pu | | |
| BESS only: `SOC` from battery model | – | | |

## 5. Network and grid  (typical values)

Layout (revised 2026-11-16, stage 4b): every unit has its own unit transformer and is tapped off a radial feeder; the feeders of a plant end on its collector bus.

```
wind:  turbine 0.69 kV ─ 0.69/10 kV ─ tap ┐ 10 kV feeders ─ 10 kV wind bus ─ 10/33 kV ─┐
PV:    station 0.6 kV  ─ 0.6/33 kV  ─ tap ┘ 33 kV feeders ─────────────────────────────┤ 33 kV HPP busbar ─ 33/132 kV ─ POC ─ grid
BESS:  unit 0.69 kV    ─ 0.69/33 kV ─ short 33 kV cable ───────────────────────────────┘
```

| Item | Typical value (`networkParameters`) |
|---|---|
| Wind unit transformer | 0.69 / 10 kV, 5.6 MVA per turbine, u_k = 7 %, X/R = 7 |
| Wind feeders | 10 kV, 630 mm² Al: R = 0.05 Ω/km, X = 0.095 Ω/km, C = 0.5 µF/km; 2 km to the first turbine, 1 km between turbines (about 10 MW per feeder) |
| Wind transformer | 10 / 33 kV, 170 MVA, u_k = 10 %, X/R = 30 |
| PV unit transformer | 0.6 / 33 kV, 5.5 MVA per inverter station, u_k = 6 %, X/R = 7 |
| PV feeders | 33 kV, 240 mm² Al: R = 0.10 Ω/km, X = 0.12 Ω/km, C = 0.25 µF/km; 2 km to the first station, 0.5 km between stations |
| BESS | 0.69 / 33 kV, 33 MVA, u_k = 8 %; 0.3 km 33 kV cable to the HPP busbar |
| HPP transformer | 33 / 132 kV, 250 MVA, u_k = 12 %, X/R = 40 (tap changer: later) |
| POC | 132 kV, export limit `p.grid.Pmax_poc` = 180 MW |
| Grid | Thevenin source, **SCR = 10** (strong), X/R = 10. Weak-grid case set: SCR = 3 |
| Frequency | 50 Hz; grid frequency as an input signal (events and ramps from case sets) |

Network solution (RMS): per unit on the system base 180 MVA, ideal transformer ratios. `networkSolverData` builds the nodal admittance matrix Y from the layout (asset terminals, feeder taps, plant collector buses, HPP busbar, POC; 13 nodes in the default layout) and stores Z = Y⁻¹ once; each step the masked `Network` subsystem computes `V = Z * (I_inj + I_grid)` in a MATLAB Function block. Each plant's P and Q are the flows of its branches into the 33 kV HPP busbar (wind: its 10/33 kV transformer; PV and BESS: their feeders, including the busbar-end cable charging): these are the plant measurement points. It also returns the HPP busbar voltage and P, Q, V at the POC. Transformer taps and reactive compensation (STATCOM, shunt reactors) can be added later as extra nodes or branches.

**Two network models, chosen per run (2026-11-17).** The `Network` block in `HPP.slx` is a Variant Subsystem with two choices built from the same layout parameters and with the same ports: `Nodal` (the solver above, fast) and `Simscape` (standard Simscape Electrical three-phase blocks solved in **frequency-and-time mode**: phasors around 50 Hz, one Solver Configuration, local Backward Euler at `p.sim.Ts`, fixed cost with one iteration). The variant control variable `HPP_network` (1 nodal, 2 Simscape, from `networkCode`) is set per run by `runCase(..., Network="nodal" | "simscape")`, `runSweep(..., Network=...)` and the `network` setting in `main.m`; the default is `p.net.solverType` ("nodal"). The inactive choice is not compiled. No rebuild is needed to switch.

- Grid: Controlled Voltage Source (Three-Phase), magnitude E·V_poc as line rms with the dips of `p.grid.dip`, behind a series R-L Z_g from SCR and X/R.
- HPP transformer 33/132 kV and wind transformer 10/33 kV: Two-Winding Transformer (Three-Phase), Yg/Yg, with a magnetizing branch (`Rm`, `Xm` in `p.net.main` and `p.net.wt.plant`; 1000 pu and 500 pu: 0.1 % no-load loss, 0.2 % magnetizing current). The Simscape model needs a finite branch in this mode; the nodal model has the same branch as a shunt at the 33 kV side (HPP transformer) and the collector side (wind transformer), so both describe the same transformers.
- Feeder segments: π model from RLC (Three-Phase) blocks (series R-L, half the charging capacitance to ground at each end). The library Transmission Line (Three-Phase) block was not used: in this mode it produced a constant, non-physical 100 Hz power ripple (0.6 MW at 147 MW) even with constant converter currents; the equivalent R-L-C π has none.
- Unit transformers: series R-L at the collector voltage (u_k and X/R on the rating × nAgg). With real transformer blocks the network was 3.5 times slower.
- Converter interface (subsystem per asset): Controlled Current Source (Three-Phase) with magnitude/phase inputs from the asset's current phasor; the terminal voltage comes from a phase voltage sensor turned into a phasor by the space-phasor transform at the nominal frequency (sampled with a Digital Clock), one step old as before. No PLL is needed because the mode returns exact 50 Hz samples.
- Measurements: Power Sensor (Three-Phase) between each plant node and the HPP busbar (plant measurement points) and at the POC; voltages from the same phasor transform.

Results (default layout), Simscape against nodal, with the same magnetizing branch in both: nominal P_poc 146.526 against 146.525 MW, Q_poc −10.900 against −10.883 Mvar, P_wpp 132.133 against 132.131 MW; dip 0.5 V_poc 0.564 pu in both, Q_wpp 51.33 against 51.35 Mvar, P_poc during the dip 85.55 against 85.50 MW, largest P_poc difference over the dip 0.10 MW (one step at fault clearing), median 0.001 MW. (Before the branch was added to the nodal model, P_poc differed by 0.84 MW: the magnetizing loss with 500 pu.) +0.2 Hz and BESS at SOCmin also gave the same results. No ripple (below 1e-10 MW); plant Q loops settle to 0. Run time: section 11. Layout of the `Network` subsystem: current injections (left), one row per converter in each plant area (converter interface, unit impedance, feeder segment), plant transformer and meter, then a grid column (grid voltage with dips; HPP transformer, POC meter, Z_g and E; bus and POC voltages with the Solver Configuration) and the output buses (right). Electrical nodes are joined by Connection Labels (`wpp_collector`, `tap_1`, `HPP_bus`, `POC`), signals by local Goto/From. Standalone test models: `experiments/simscapeGrid/ftGridTest.m`.

Generator interface (wind: behind the IEC Type 4A generator system; PV and BESS: behind the WECC REGC_A converter model): current source aligned with the terminal voltage, I = (i_p − j·i_q)·V/|V|, so P = |V|·i_p and Q = |V|·i_q. The voltage used is that of the previous step (as a PLL would), which also avoids an algebraic loop through the network.

Current results (default layout, nodal network, transformers with magnetizing branch, plant Q loops closed at their measurement points): nominal P_poc 146.525 MW, Q_poc −10.9 Mvar, V_poc 0.999 pu. History: with the stage-4b layout before the magnetizing branch P_poc was 146.95 MW and Q_poc −10.4 Mvar (146.47 MW with the stage-2 network); dip 0.5: V_poc 0.56 pu, P_poc back at 146.7 MW after 1 s. Run time: section 11.

## 6. Communication delays and rates  (typical values)

| Path | Delay | Controller rate |
|---|---|---|
| TSO → HPPC set-points | 200 ms | |
| POC measurement → HPPC | 20 ms | HPPC: 100 ms |
| HPPC → plant controllers | 50 ms | plant controllers: 20 ms |
| Plant measurement → plant controller | 20 ms | |
| Plant controller → asset controllers | 20 ms | asset controllers: 1 ms (= sim step) |
| Asset measurements → asset controller | 0 (local) | |

Delays are parameters (`p.comm.*`) so case sets can sweep them. Implemented as a discrete delay of whole simulation steps, `p.comm.steps.<path> = round(delay / p.sim.Ts)` (`deriveParameters`). Each delay in a link block is a Variant Subsystem with two choices, selected per run by `p.comm.delays` through the variant control variable `HPP_delays` (`delayMode`; activation at update diagram, so only the active choice is compiled): **Fixed** (`HPP_delays = 1`): the length is a block parameter, fixed at compile time, fastest. **Variable** (`HPP_delays = 2`): the length is a tunable constant input, so the delays change between runs (Fast Restart and Rapid Accelerator) up to `p.comm.maxDelay` (0.5 s, the buffer size); about 0.6–1 s slower per 60 s run in Rapid Accelerator (2.2–2.6 s instead of 1.5–1.7 s). `"auto"` (default) takes Variable when the case set changes a `comm.*` value, otherwise Fixed; `runCase`, `runSweep` and `main.m` can override it (option `Delays`). The case set `comm_delay` sweeps the delays. The receiving controller samples it at its own rate. Every measurement path into a controller also has a one-step memory (1 ms): without it, the loops through the controllers' direct feedthrough are algebraic loops when a delay is 0. The TSO delays start at the initial set-point, so the HPPC does not see a zero reference at start-up.

## 7. Parameters

```
p.sim            solver step 1 ms, stop time, logging decimation (every 10 ms)
p.grid           POC, base, grid SCR, X/R, frequency
p.net            transformers (incl. magnetizing branch), cables, lengths per feeder; solverType = default network model
p.comm           communication delays
p.wt  / p.pv / p.bess   nFeeders, nPerFeeder, nAgg, Pn (N, Prated derived), asset-model parameters
p.hppc           HPPC settings (P: Pflag, PI, Tref, Vfrz / Thld, ramp limit; frequency: Frqflag, LFSM-O / LFSM-U, FSM; SOCband;
                 Q / V / PF: Qflag, QawFlag, RefFlag, Vref, Vslope, PF, PI, SCRset;
                 voltage cascade: Vlocal, KpVt / KiVt, KcBusMin, cascadeSep), copied into p.ctrl.hppc.par
p.pc             plant controller (REPC_A) settings incl. voltage-control gains KpV / KiV, copied into
                 p.ctrl.wppc / sppc / bessc .par (droop Kc, dVdQ and, in voltage control, RefFlag / Vref
                 set there by controllerParameters)
p.conv           converter part of the assets: type (1 current source, 2 averaged converter), LCL filter,
                 current loop, DC link (14.10); gains and the discretized filter derived, passed as ap.conv
p.ctrl.hppc      Ts, Pbase, par
p.ctrl.wppc / sppc / bessc      Ts, Pbase, par
p.ctrl.wt / pv / bess           Ts, Pbase, par   (one set, shared by all N instances;
                                                  per-instance values later if needed)
```
Each standard model's parameter list is kept with the standard's own names, so values can be checked against the IEC / WECC tables.

## 8. Stages

Each stage ends with a regression check against the previous stage, as for the per-unit change.

1. **Skeleton. Done (2026-11-16).** New files and folders, N-instance mechanism, communication delays, parameter structure. Placeholder logic in each controller. With delays set to 0, P_poc, P_bess_set and SOC match the earlier single-controller model, except that the HPPC reacts one sample (0.1 s) later at start-up and at the 20 s wind step (one-step measurement memories). A 60 s run takes about 15–20 s, against about 1 s before.
2. **Network. Done (2026-11-16).** Phasor network with the typical values; Q and V appear; measurements come from the network; the HPPC splits Q_ref over the plants by rating (open loop). Checked against a hand-calculated load flow (identical P_poc, Q_poc, V_poc, V_bus). New case set `grid_strength` (SCR 10 / 5 / 3). A 60 s run takes about 20 s.
3. **Wind asset. Done (2026-11-16), except the WPPC.** `WT_ctrl.slx` is now an IEC 61400-27-1 Type 4A control as a block diagram (template `buildWTCtrlType4A` in `buildHPP.m`): measurement filters (U, P, Q), voltage-dip detection (U < `udip`), P control type 4A (P_ref limited to the available power, ramp `dpmaxp4A`, lag `Tpordp4A`, i_p = p/U), Q control (open loop i_q = q/U, plus dip current `Kqv·(uref0 − U)` during a dip, limited to `iqH1`), and current limitation (|i| ≤ `imax`; P priority normally, Q priority in a dip). `WT_asset.slx` adds the generator system: rate limits `dipmax` / `diqmax` and lag `Tg` on each current. Parameters in `p.wt.iec` (`ctrl` → `p.ctrl.wt.par`, `gen` → `p.wt.ap(k).gen`). The grid source voltage can now dip (`p.grid.dip`: start, duration, depth); new case set `wt_frt` (no dip, dip 0.5, dip 0.8, dip 0.5 in SCR 3).
   Checks: steady state identical to stage 2 (P_poc 146.358 MW, Q_poc −21.3 Mvar). Dip 0.5 (150 ms): V_poc ≈ 0.53 pu, the wind plant injects ≈ 50 Mvar, P_poc ≈ 92 MW; after clearing, P ramps back at `dipmax` = 1 pu/s and is back at 146.4 MW within about 0.6 s. Dip 0.8: V_poc ≈ 0.26 pu, Q ≈ 51 Mvar, P ≈ 30 MW. SCR 3, dip 0.5: V_poc ≈ 0.62 pu, Q ≈ 53 Mvar, P ≈ 62 MW. Figure: `results/stage3_wt_frt.png`.
   Known limits: (a) parameter values are placeholders and block details (filter forms, dip logic with hysteresis/post-fault hold, `Kqv` reference, current-limit options, the `uqdip`/`Mqfrt` modes) still need checking against the standard text; (b) during the first ≈ 30 ms of a dip and at clearing the instantaneous network with one-step-delayed current sources rings for a few steps (single-step spikes up to ≈ 180 Mvar); a smaller step or a network voltage filter would remove this; (c) the WPPC from the IEC WPP control model is not done yet, the WPPC is still the open-loop placeholder.
4. **PV and BESS assets and the plant controllers. Done (2026-11-16).** `PV_ctrl.slx` (WECC REEC_B) and `BESS_ctrl.slx` (REEC_C) are block diagrams from the template `buildReecCtrl` in `buildHPP.m`: voltage filter `Trv`, dip flag (V < `Vdip` or V > `Vup`), P path (limits — PV: 0 … available power, BESS: ±`Pmax` — ramp `dPmax`/`dPmin`, lag `Tpord`, Ipcmd = Pord / V), Q path, at first with Qflag = 0 only (Iq = Q_set / V, lag `Tiq`, plus Iqinj = `Kqv`·deadband(`Vref0` − V) limited to `Iqh1`/`Iql1` during a dip), and current limit logic (|I| ≤ `Imax`, `PQflag` 0 = Q priority, 1 = P priority; PV cannot absorb; BESS: no discharge at SOC ≤ `SOCmin`, no charge at SOC ≥ `SOCmax`). `PV_asset` / `BESS_asset` add the REGC_A converter: Ip recovery ramp `Rrpwr`, Iq ramps `Iqrmax`/`Iqrmin`, lag `Tg`, LVPL (Ip limit from 0 at `Zerox` to `Lvpl1` at `Brkpt`) and LVACM (Ip scaled to 0 between `Lvpnt1` and `Lvpnt0`). Parameters in `p.pv.wecc` / `p.bess.wecc` (`ctrl` → `p.ctrl.<type>.par`, `gen` → `p.<type>.ap(k).gen`). New case set `bess_soc` (SOC0 0.5 / at SOCmin / at SOCmax).
   Checks: steady state identical to stage 3 (P_poc 146.358 MW, Q_poc −21.28 Mvar at 55 s). Dip 0.5: V_poc ≈ 0.56 pu (0.53 in stage 3, the PV and BESS now support the voltage), Q: wind ≈ 47, PV ≈ 17, BESS ≈ 11 Mvar; PV P drops to ≈ 13 MW (LVACM); all back to pre-fault values within ≈ 0.1 s. Dip 0.8: V_poc ≈ 0.32 pu, PV P ≈ 0.8 MW. At SOCmin the BESS gives 0 MW against a 30 MW discharge set-point and still charges afterwards; at SOCmax it discharges, and charging stops when SOC reaches 0.9. The converter lag Tg = 20 ms also removed most of the ringing at the dip start seen in stage 3; the single-step spike at fault clearing remains (Q_poc ≈ 125 Mvar for one step). Figure: `results/stage4_frt_dip05.png`.
   Simplifications (to check against the WECC specifications): no voltage-dependent current limits (VDL1/VDL2), no state freeze and no post-dip hold `Thld`/`Thld2` during a dip, no Pflag (P·ω), closed-loop Q added afterwards (see below: Qflag = 1 is now the default), no high-voltage reactive current management and no voltage filter `Tfltr` in REGC_A, LVPL static on the unfiltered voltage, REGC_A rate limits applied before the lag instead of on its state. Parameter values are typical example values, not from a specific inverter.
   **Closed loops at every level (decided 2026-11-16).** As in state-of-the-art HPPs, where vendors supply the assets together with their own plant controller, every level closes its own loops; the HPPC (stage 5) then has to coordinate closed-loop, vendor-style plant controllers. This interaction is deliberate (it is the problem the PhD project addresses) and should stay visible in the model.
   - *Asset controllers:* closed-loop reactive power control. Wind: IEC mode `MqG` = 1 (PI `kpq`, `kiq` on q_ref − q_filt, added to the open-loop current q_ref / U; `MqG` = 2 open loop). PV / BESS: REEC `Qflag` = 1 (PI `Kqp`, `Kqi` on Q_set − Q_gen; `Qflag` = 0 open loop). The PI is frozen during a voltage dip. Active power stays open loop in both standards (the converter current control, `Tg`, tracks it); there is no P loop at asset level.
   - *Plant controllers* (`WPPC`, `SPPC`, `BESSC`, template `buildPlantCtrlRepc`, parameters `p.pc`, file `params/plantControlParameters.m`): WECC REPC_A structure on the plant feeder into the 33 kV busbar. P path: frequency droop (`Ddn` over-frequency, `Dup` under-frequency, deadband `fdbd1`/`fdbd2`) on P_set, PI (`Kpg`, `Kig`) on that target − P_filt, limits from availability (BESS: SOC-aware charge/discharge limits), lag `Tlag`; `Frqflag` = 0 passes P_set on unchanged. Q path: `RefFlag` = 0 tracks Q_set, 1 controls the busbar voltage to `Vref` with reactive droop `Kc`; deadband, error limits `emax`/`emin`, PI (`Kp`, `Ki`) frozen below `Vfrz`, limits `Qmax`/`Qmin`, lag `Tfv`. Deviation from REPC_A: P_set (incl. droop) and Q_set are fed forward around the PI controllers, as in many vendor plant controllers, so the PI only corrects losses and asset deviations; without it, REPC_A example gains would take tens of seconds to reach the set-point. Then dispatch over the N assets (P by availability, Q by rating share; BESS: only assets inside their SOC range). Wind and PV run at available power, so `Dup` = 0 for them. The WPPC uses the REPC_A structure for now; the IEC 61400-27-1 WPP control model has the same functions and is a later refinement.
   - *Grid frequency:* `p.grid.fev` (step at `t0` by `df`, pu) drives a new `f` element of the network bus (plant controllers) and `f_meas` on the TSO bus (HPPC). Case set `freq_step` (+0.2 Hz, −0.2 Hz, +0.2 Hz with `Frqflag` = 0 in all plants; since stage 5 the plant droop is the `decentral` case, see stage 5).
   Checks (nominal case): each plant now meets its own set-point at its busbar feeder (Q_wpp, Q_spp, Q_bess → 0 within about 5 s; P_bess = −5.00 MW exactly); wind and PV stay at their available power, which they cannot exceed to cover losses. Q_poc improves from −21.3 to −10.4 Mvar (the rest is the 132 kV transformer and grid side, for the HPPC); P_poc 146.47 MW. Frequency +0.2 Hz: each plant reduces P by 5 % droop × (0.2 Hz − 20 mHz) = 7.2 % of its rating relative to its set-point (wind 135 → 124.2 MW, PV 20 → 16.4 MW, BESS −5 → −7.16 MW; P_poc 146.5 → 133.2 MW); −0.2 Hz: only the BESS responds (−5 → −2.84 MW); `Frqflag` = 0: no response. Dip 0.5: V_poc ≈ 0.56 pu, Q wind ≈ 56, PV ≈ 17, BESS ≈ 10 Mvar; P_poc back at 146.1 MW 0.35 s after clearing. Dip 0.8: Q wind ≈ 67 Mvar, P_poc back to 146.5 MW within 3 s. Since the HPPC is still open loop, nothing at the POC counteracts the plants' frequency response yet; with the stage-5 HPPC frequency response this becomes the first coordination case (double response, competing integrators).
5. **HPPC functions. In progress.** P and Q/V control at the POC, frequency response, dispatch, ramp limits, SOC management.
   **Central control (decided 2026-11-17, step 1 done).** The HPPC closes the P loop at the POC and gives the frequency response of the whole plant; the plant controllers do not respond to frequency themselves and pass their P set-points on (`p.pc.<plant>.Frqflag` = 0, which in REPC_A also opens the plant P loop). Their Q / V loops stay closed. This replaces the stage-4 default for P and frequency; the stage-4 setup stays available as the `decentral` cases (`hppc.Pflag` = 0, `hppc.Frqflag` = 0, plant `Frqflag` = 1).
   `HPPC.slx` (template `buildHppcCtrl` in `buildHPP.m`, parameters `p.hppc` in `params/hppcParameters.m`), per unit on 180 MW, 0.1 s:
   - *Frequency response* (`Frqflag`): P_target = P_ref + droop, with REPC_A-style droop on the POC base (`Ddn` / `Dup` = 20, i.e. 5 %; deadband `fdbd` ±20 mHz), then the export limit `Ppoc_max`. Under-frequency headroom comes from the BESS (wind and PV run at available power). (Replaced in step 3 by LFSM-O / LFSM-U and optional FSM after the grid code.)
   - *P control* (`Pflag`): P_ord = P_target + PI(P_target filtered by `Tref` − P_poc filtered by `Tp`), PI `Kpg` 0.1, `Kig` 0.5 /s with clamping anti-windup, error limits `femax` / `femin`, output limits from the plants' availability (P_avail_wpp + P_avail_spp + P_dis_max_bess, and −P_chg_max_bess). The reference filter `Tref` (0.4 s) keeps the PI from integrating the response time of a step (measurement filter, delays, plant lags): with it the +20 MW step overshoots 0.8 MW instead of 3 MW. The PI is frozen while V_poc < `Vfrz` (0.9) and for `Thld` (1 s) after it; without the freeze the PI wound up during a dip (P_poc overshoot 3–13 MW, recovery 2–4 s instead of 0.1–0.5 s).
   - *Dispatch:* wind and PV at available power, the BESS balances (P_ord − available renewable power, within its charge / discharge limits); wind and PV are curtailed in proportion to availability only when the BESS cannot absorb the surplus. Q_ref is still split by rating (open loop). The POC Q / V and plant P / Q measurements and SOC are inputs but not used yet.
   - The TSO P_ref can step (`p.grid.Pstep`: time, change); new case set `hppc_p`.
   Checks (default layout, nodal network, Fast Restart):
   - Nominal: P_poc 150.00 MW for P_ref 150 MW (146.52 MW open loop); the BESS covers the losses (−1.51 instead of −5.01 MW). Before the wind step (t < 20 s) the plant cannot reach 150 MW: P_poc 137.8 MW with the BESS at its discharge limit.
   - `hppc_p`, P_ref steps at 40 s: −30 MW → 120.00 MW (BESS at its charge limit −30.3 MW, wind curtailed 132.1 → 131.0 MW; undershoot 1.6 MW, within 1 % after 1.0 s); +20 MW → 170.00 MW (overshoot 0.8 MW, 0.5 s); +40 MW → export limit 180.00 MW (0.5 s). Open loop: 146.52 → 116.52 MW.
   - `freq_step` (step-1 droop, since replaced): ±0.2 Hz → P_poc −12.96 / +12.96 MW (5 % × 0.18 Hz × 180 MW, symmetric because the BESS gives the under-frequency response; it absorbs the over-frequency response instead of curtailing wind), within 5 % after 0.3–0.7 s. Decentral (stage 4): −13.55 / +2.15 MW (only the BESS can respond to under-frequency, and only on its own 30 MW base), 1.4–2.0 s, P_poc offset by the uncompensated losses.
   - `wt_frt`: P_poc back at 150 MW 0.10 s (dip 0.5), 0.46 s (dip 0.8) and 0.45 s (dip 0.5, SCR 3) after clearing, no overshoot (max 150.2 MW apart from the known one-step spike at clearing).
   - `grid_strength` and `bess_soc`: P_poc 150.00 MW in all cases (at SOCmin before the wind step 108 MW: no BESS discharge).
   **Q / V control at the POC (step 2 done, 2026-11-18).** Same HPPC, parameters in `p.hppc`; the plant controllers keep their own Q loops (REPC_A `RefFlag` = 0, tracking their Q set-point at the busbar feeder), so the HPPC is an outer loop over them.
   - *Q control* (`RefFlag` = 0, default): Q_ord = Q_ref + PI(Q_ref filtered by `TrefQ` − Q_poc filtered by `Tq`), PI `Kpq` 0.1, `Kiq` 0.5 /s. *Voltage control* (`RefFlag` = 1): Q_ord = PI(`Vref` − V_poc − `Kc`·Q_poc), PI `KpV` 2, `KiV` 10 /s, no feed-forward. Error limits `eqmax` / `eqmin`, Q_ord limited to ±0.33 pu (about 60 Mvar, power factor 0.95 at 180 MW); clamping anti-windup; frozen with the P loop during a dip. `Qflag` = 0 gives the open-loop split of Q_ref. The Q order is split over the plants by rating (`qshare`).
   - The TSO Q_ref can step (`p.grid.Qref`, `p.grid.Qstep`); a sustained grid voltage change uses `p.grid.dip` with a long duration. New case set `hppc_qv`.
   Checks (default layout, nodal network):
   - Q control: Q_poc 0.00 Mvar for Q_ref 0 (−11.39 Mvar open loop, the 33/132 kV transformer and grid side); V_poc 1.0048 instead of 0.9985. Q_ref ±30 Mvar at 40 s: ±30.00 Mvar, within 5 % after 0.8–0.9 s, overshoot 5.8 / 8.8 Mvar. Most of the overshoot is from the plant and asset Q loops (feed-forward plus PI at two levels): the same step with the HPPC loop open overshoots 4.7 Mvar. A longer `TrefQ` (0.8 s) or a smaller `Kiq` hardly changes it.
   - Voltage control, grid source −2 % at 40 s: V_poc back at 1.0000 pu, Q_poc −8.7 → +27.3 Mvar at SCR 10 (2.9 s, no overshoot) and 6.0 → 17.0 Mvar at SCR 3 (1.0 s, 2 Mvar overshoot). The loop gain is dV/dQ ≈ 1/SCR, so one setting is a compromise: `KiV` 5 took 6 s at SCR 10, `KiV` 20 oscillated at SCR 3 (10 Mvar overshoot, 3.6 s). (Replaced in step 3: slope and normalised gains.) Q control in the same event keeps Q_poc at 0 and V_poc drops to 0.985. `Vref` 1.01: Q_poc 9.4 Mvar.
   - P loop unchanged (`hppc_p`, `freq_step`: same P results; open loop now 146.56 MW because Q_poc = 0 changes the losses). `wt_frt`: P_poc back within 1 % 0.13 s (dip 0.5), 0.52 s (dip 0.8), 0.29 s (dip 0.5 SCR 3) after clearing, max 150.2 MW.

   **Grid code: ramp limit, LFSM / FSM, PF control, SOC-aware dispatch (step 3 done, 2026-11-18).** The HPPC functions follow Energinet's requirements for DK1, from the Danish RfG requirements (*Krav jf. RfG, bilag 1*, power park modules type D; the copy used is rev. 2 of 22-07-2022, marked historic, so check against the current revision) and Technical Regulation 3.3.1 for energy storage facilities (rev. 6). Pn = 180 MW (POC base).

   | Requirement (DK1) | Source | Model (`p.hppc`) |
   |---|---|---|
   | Ramp rate constraint up and down, 1–20 % Pn/min, at most 60 MW/min (PPM) or 1 MW/s (storage); maximum by default; not for frequency response | RfG 15(6)(e); TR 3.3.1 §32 | `rampUp` / `rampDn` 0.20 pu/min = 36 MW/min, on the set-point and availability changes; frequency response added after it |
   | Priority: protection, synthetic inertia, frequency control, power constraint, power gradient constraint | RfG 14(5)(c); TR 3.3.1 §26 | frequency response after the ramp limiter, before the export limit |
   | LFSM-O: threshold 50.2–50.5 Hz (DK1 50.2), droop 2–12 % (DK1 5 %), start ≤ 2 s, full ≤ 30 s, prevails over other set-points | RfG 13(2); TR 3.3.1 §23 | `fO` 50.2 Hz, `sO` 5 %, always on |
   | LFSM-U: threshold 49.5–49.8 Hz (DK1 49.8), droop 2–12 % (DK1 5 %), up to maximum capacity | RfG 15(2)(c); TR 3.3.1 §52 | `fU` 49.8 Hz, `sU` 5 %, always on (headroom from the BESS) |
   | FSM: droop 2–12 %, deadband 0–500 mHz, insensitivity ≤ 10 mHz, range ≥ 1.5–10 % Pn, full activation ≤ 30 s | RfG 15(2)(d); TR 3.3.1 §53 | `FSMflag` (off unless delivering FCR), `sFSM` 5 %, `fsmDb` 10 mHz, `fsmRange` 10 %; the BESS keeps `fsmRange` as upward reserve |
   | Voltage control: set-point 0.95–1.05 pu, slope 2–7 %, 90 % of the Q change within t1 = 1 s, settled within t2 = 5 s (5 % of Qmax) | RfG 21(3)(d)(ii–iv) | `RefFlag` 1, `Vref`, `Vslope` 4 % |
   | Q, voltage and PF control modes; PF: start ≤ 2 s, complete ≤ 30 s | RfG 21(3)(d)(i, vi) | `RefFlag` 0 / 1 / 2, `PF`, `PFsign` |
   | P set-point accuracy 2 % Pn over 1 min | RfG 15(2)(a) | closed P loop at the POC |

   Changes in `HPPC.slx` (template `buildHppcCtrl`):
   - *Ramp limiter* (MATLAB Function, state starts at the set-point): P_base follows min(P_ref, P_avail − FSM reserve) at `rampUp` / `rampDn`. Availability rises (the wind step at 20 s) are therefore ramp-limited too; the BESS absorbs the renewable surplus meanwhile, and covers a fast drop of availability up to its limits.
   - *Frequency response* (MATLAB Function): LFSM-O − max(f − fO, 0)/sO, LFSM-U + max(fU − f, 0)/sU, and with `FSMflag` the FSM droop with deadband, limited to ±`fsmRange`. This replaces the step-1 symmetric droop with ±20 mHz deadband, which was effectively FSM. `Frqflag` 0 switches all of it off (decentral cases).
   - *SOC-aware BESS limits:* discharge (charge) derated linearly to zero over `SOCband` (0.1) above SOCmin (below SOCmax); wind and PV are curtailed when the BESS cannot absorb more. The dispatch uses these limits.
   - *Voltage control with slope:* PI on Vref − V − (`Vslope`/`Qmax`)·Q, so in steady state Q = (Vref − V)/`Vslope` · Qmax. The PI gains are normalised by the loop gain (`Vslope`/`Qmax` + dV/dQ), dV/dQ = 1/`SCRset` (the grid strength set at commissioning; default the actual `p.grid.SCR`): `KpV` 0.5, `KiV` 2.5 /s.
   - *PF control* (`RefFlag` 2): Q reference P_poc · `PFsign` · tan(acos(`PF`)), then the Q loop.
   - `runSweep` with Fast Restart no longer refuses `sim.Tstop` (it matched `sim.Ts` as a prefix).
   Checks (default layout, nodal network):
   - `freq_step`: +0.2 Hz (at the LFSM-O threshold) → no response; +0.5 Hz LFSM-O → −21.60 MW (12 % of Pn = 0.3 Hz / 50 Hz / 5 %); −0.5 Hz LFSM-U → +21.60 MW (BESS); FSM ±0.2 Hz → ∓13.68 MW (7.6 %); FSM + LFSM-O at +0.5 Hz → −39.6 MW (10 % + 12 %; BESS at its charge limit, wind curtailed). Start 0.1 s, 90 % after 0.3 s (requirement: ≤ 2 s, ≤ 30 s).
   - `hppc_p` (120 s): P_ref −30 / +20 / +40 MW at 40 s reached after 48 / 31 / 47 s (1 % band; 36 MW/min); 10 %/min: 18 MW/min. Largest change over any 60 s window: 36.46 MW (limit 36) and 18.04 MW (limit 18): the 0.5 MW excess is the PI taking up the losses after the wind step, within the 2 % Pn (3.6 MW) accuracy. Without the ramp limit: 1 s.
   - `bess_soc`: SOC0 0.15 → BESS discharge derated to 14.6 MW (29.7 at SOC 0.5); at SOCmin 0 MW; SOC0 0.15 with +20 MW: BESS 13.5 MW, P_poc 165.0 instead of 170 MW; near SOCmax charging −0.8 MW and wind / PV curtailed, P_poc 150 MW.
   - `hppc_qv`, voltage control after a 2 % grid voltage drop: Q follows the slope (4 %: 12.3 Mvar at V 0.9917); t90 0.49–0.59 s, settled within 5 % of Qmax after 0.34–0.52 s for slopes 2 % and 7 % at SCR 10 and 3 (t1 = 1 s, t2 = 5 s met). With `SCRset` 10 on an SCR 3 grid: t90 0.31 s, 50–90 % overshoot, settled after 0.8–1.5 s; with `SCRset` 3 on an SCR 10 grid: t90 1.4–2.1 s, which misses t1. PF 0.95: 0.953 export (47.7 Mvar), 0.950 import (−49.3 Mvar). Q steps unchanged.
   - `wt_frt`: P_poc relative to the no-dip case back within 1.5 MW 0.13 / 0.51 / 0.29 s after clearing, no overshoot.
   - Not modelled: the island-detection delay of 500 ms (distribution-connected storage only), synthetic inertia, FRT current requirements at the POC (asset level only), the U-Q/Pmax profile and Q capability per P, the LFSM-O behaviour at minimum regulating level.

   **Voltage control as a cascade, POC Q limit, anti-windup (step 4 done, 2026-11-18).** Developed partly in a parallel session and reviewed here.
   - *Why:* in practice voltage is controlled in layers: each plant controller regulates the voltage at its own point with a droop (fast), and a master controller slowly corrects the set-points so the requirement holds at the POC (as primary and secondary voltage control in transmission grids). The requirement itself (slope, t1 / t2, Q range) is defined at the POC, so that part stays central. It also fits the research setting: vendor plant controllers that close their own loops under a coordinating HPPC.
   - *Plant controllers* (`hppc.RefFlag` 1 with `hppc.Vlocal` 1, the default): each controls the 33 kV busbar voltage on a V-Q droop line Q = Q_set + (Vref − V)/Kc; Q_set, sent by the HPPC, is the offset of the line and is fed forward in both modes. The droop of all plants together is the POC slope minus the 33/132 kV transformer reactance (0.121 − 0.086 = 0.035 pu on 180 MVA for 4 %), at least `KcBusMin` 0.02 (a negative droop makes parallel plants push Q between each other); every plant gets the same droop on its own rating, so the Q split follows the ratings. The PI gains (`KpV` 0.5, `KiV` 2.5) are normalised by the busbar loop gain Kc + dV/dQ (dV/dQ = 1/SCR + transformer reactance, from the commissioned SCR).
   - *HPPC:* trims the plants' Q offsets slowly (integral only, `KiVt` 0.3, about 3 s) so V and Q at the POC lie on the required slope. Only a share Kc/(Kc + dV/dQ) of an offset change becomes real Q (0.16 at SCR 10), so the order range is widened to ±Qmax/share. `Vlocal` 0 keeps the direct POC voltage control of step 3.
   - *Cascade timing:* every level at least `cascadeSep` (3) times slower than the one below, estimated as (1 + Kp)/Ki; `controllerParameters` warns otherwise (`p.ctrl.hppc.cascade`). Asset Q PIs 5 → 10 /s (0.11 s), plant Q 1.1 s, plant voltage 0.6 s, HPPC Q / PF 0.5 → 0.2 /s (5.5 s), HPPC voltage trim 3.3 s.
   - *POC Q limit:* with the plants doing the voltage control, the HPPC's ±0.33 pu order limit no longer limits Q at the POC. The HPPC now caps its voltage error by the Q margin, e = min(max(e, (Qmin − Q)·k), (Qmax − Q)·k) with k the loop gain (block "POC Q limit"), so the loop settles at the limit. Tighter plant order limits were tried and dropped: they limit the Q at the asset terminals, and the wind plant loses up to about 40 % of it in its transformers and cables (wind stopped at 28 instead of 49 Mvar).
   - *Anti-windup over the plant limits:* the plant controllers report `Q_sat` (+1 / −1 when their Q order is at its limit; new last output); the HPPC holds its Q loop (error 0) in a direction where all plants are saturated (`QawFlag`, conditional integration). Its own output limits already have clamping in the PI block. This is part of the controller interfaces (DLL ports): HPPC 20 inputs, plant controllers one more output.
   Checks (default layout, nodal network):
   - Voltage control after a 2 % grid voltage drop: t90 0.40–0.59 s, settled within 0.52 s, overshoot 5–22 % for slopes 2–7 % and SCR 3–10 (t1 1 s, t2 5 s met). Set for SCR 10 on an SCR 3 grid: t90 0.12 s, 19 % overshoot (was 50–90 % with direct POC control). At a 2 % slope the needed busbar droop is negative, so `KcBusMin` applies and the trim pulls the slope right within its ~3 s.
   - Q limit: grid −10 % / +8 % → Q_poc +59.40 / −59.40 Mvar; the plants' droop overshoots to +69 / −72 Mvar first, back within 0.3 Mvar of the limit after ~20 s.
   - Anti-windup: plant Q limits 0.2 pu, grid −10 % from 40 to 60 s: Q back within 3 Mvar 0.9 s after the voltage returns; without anti-windup Q stays at +15.8 Mvar (V_poc 1.013) for 9.5 s.
   - Controller speed and grid strength (SCR 3 / 10, set right and wrong): halving the plant voltage gains lowers the overshoot to 2–12 %, a quarter to 0–1 %, but t90 then reaches 1.23 / 2.75 s and misses t1. Final Q does not depend on the setting. The current gains are the compromise; an online estimate of dV/dQ would remove the dependence on the commissioned SCR.
   - Q steps (overshoot 6–10 Mvar, settled 0.65 s), P control, frequency response, ramps, SOC limits and fault ride-through unchanged.

Afterwards: code generation of every controller (needs the Coder products and a compiler), the IEEE/CIGRE wrapper, and PSCAD validation.

## 9. Decided after review (2026-11-16)

- **Plant sizes:** 150 MW wind (30 × 5 MW turbines since stage 4b, 10 kV collector), 50 MW PV (10 × 5 MW stations), 30 MW / 60 MWh BESS, 180 MW POC export limit.
- **POC voltage:** 132 kV.
- **Grid code:** Energinet's DK1 requirements (Danish RfG requirements for power park modules type D; Technical Regulation 3.3.1 for the BESS). Since 2026-11-18 the HPPC functions follow them (section 8, stage 5 step 3); compliance is checked in simulation only.
- **Wind model:** IEC 61400-27-1 **Type 4A** (no drive-train model) for now.
- **Per-instance parameters:** not planned, but kept possible. Every asset instance k already gets its own `p.<type>.ap(k)`, including its controller's `par`, so different turbine types in one plant only need different values in `ap(k)`; the model does not change.

## 10. Open points
- **Stage 5, HPPC functions** (section 8): done: closed-loop P and frequency response at the POC (central control), P dispatch, Q or V control at the POC. Done since: ramp limits, LFSM-O / LFSM-U / FSM and PF control after the Danish grid code, SOC-aware dispatch (step 3); voltage control as a cascade over the plant controllers, POC Q limit and anti-windup (step 4). Next: V_ref and PF from the TSO, Q dispatch by available Q capability instead of rating, the U-Q/Pmax profile, SOC restoration (towards a target SOC), the ramp-limit overshoot after availability steps (0.5 MW), checking the requirements against the current RfG revision. Open question: how the central HPPC loops interact with plant controllers that keep their own P and frequency loops (decentral) — the coordination the research addresses.
- **Dispatch on availability:** at the wind step (20 s) P_poc dips by about 13 MW for 0.3 s: the HPPC moves the BESS at once on the jump in wind availability, while the wind output follows only within about 0.5 s. Dispatching the BESS on the measured wind output (P_wpp is an HPPC input) would avoid it.
- **Voltage control and grid strength:** the plant and HPPC voltage gains use the commissioned SCR (`SCRset`); an online estimate of dV/dQ would make them independent of it.
- **WPPC:** uses the WECC REPC_A structure; the IEC 61400-27-1 WPP control model is a later refinement.
- **Checks against the standard texts:** IEC Type 4A block details (filter forms, dip logic with hysteresis / post-fault hold, `Kqv` reference, current-limit options, `uqdip` / `Mqfrt` modes) and the WECC simplifications listed in stage 4 (VDL1/VDL2, state freeze and `Thld` during a dip, Pflag, high-voltage reactive current management, `Tfltr` in REGC_A, rate limits on the state). All parameter values are typical, not from a specific turbine or inverter.
- **Spike at fault clearing:** the network with one-step-delayed current sources gives a single-step spike when a dip clears (Q_poc ≈ 125 Mvar for one step). A smaller step or a filter on the network voltage would remove it.
- **Steady-state initialisation** at start-up (controllers currently start from zero measurements). More important with the dq tier (section 14), where settling time costs 20–50 times more.
- **Frequency, PLL, GFM and the dq / EMT tiers:** planned in section 14 (stages 6–9), with the questions still open listed there.
- **Tap changer and reactive compensation** (STATCOM, shunt reactors): not modelled yet (section 5).
- **Simulation speed:** solved for the nodal network with Rapid Accelerator (section 13). The Simscape network does not work in Rapid Accelerator (parameter changes ignored) and runs in normal mode, at 107–127 s per 60 s case with the default layout; the reason it is slower than the ~40 s measured earlier (layout, machine load) is not measured yet.
- **Readability of the generated diagrams (Goto / From):** the builder (`ctrlSection` / `rowChain`) passes most signals through local Goto / From tags. That avoids crossing lines, but it is also used where a direct line would be clearer, and it hides feedback: no loop is visible as a loop (compare the PLL and the grid frequency, now drawn as loop diagrams with flipped delay blocks in the feedback path). Task: go through the controllers (HPPC, plant controllers, asset controllers), the asset models and the plant / HPP subsystems; draw each control loop and each signal path inside a function area with direct lines, keep Goto / From only for signals that cross areas or would cross many lines (measurements fanning out, bus elements); place blocks so the main path runs left to right and feedback runs below it right to left. Check with exported pictures of every system; behaviour must stay identical (layout only).
- **Code generation and PSCAD** (after stage 5): MATLAB Coder, Embedded Coder and MinGW installed 2026-11-19; Simulink Coder (needed to generate code from Simulink models) is being added. The controller blocks already use only tunable block expressions (section 13).

## 11. Simulation and run time  (2026-11-17)

Measured on the laptop (16 GB RAM), nominal 60 s case.

**Where the time goes.** Without Fast Restart most of a run is compiling, not simulating. `sldiagnostics` on the compile: 56 s in total, of which 23.5 s "finding and refreshing Model blocks" (loading the referenced controller and asset models; 0.6 s when they are already loaded) and 25.5 s "compiling the referenced models in normal mode". The simulation itself takes about 3–16 s with the nodal network (fewer units and more free memory are faster) and about 40 s with the Simscape network.

**Settings of the top model.**
- Solver ode1 (one evaluation per step) instead of ode4, single-tasking. The asset models keep their continuous current lags (`Tg`) and SOC integrator; making them discrete gave no speed gain and moved P during a dip by 2–3 MW, so it was not kept.
- Communication delays: discrete delays of whole steps (fixed or variable length) instead of continuous transport delays.
- Logging every 10 ms (`p.sim.logDecimation`).
Effect: about 15 % faster per run; steady state unchanged, dip values within 0.7 MW.

**Fast Restart.** `runCase(..., FastRestart=true)` (the `fastRestart` setting in `main.m`, on by default) keeps `HPP.slx` compiled in the MATLAB session; `runSweep(..., FastRestart=true)` compiles once per case set. Almost all parameters change between runs without recompiling:
- controller parameters (`par` model arguments), set-points, events (dip, frequency step);
- nodal network: the numbers (impedance matrix, grid impedance, branch impedances and charging, E) enter the solver through tunable Constant blocks; only the node structure is fixed;
- Simscape network: R, L, C and transformer data are run-time block parameters.
Fixed at compile time: the layout (units per plant), the time step, the delay mode with the delays (fixed) or the buffer size `p.comm.maxDelay` (variable), and the network model. `runCase` remembers these for the compiled model and recompiles by itself when one changes; `runSweep` with Fast Restart refuses case sets that change them. Tried and dropped: the whole network data as one tunable parameter structure (it made every run 0.5–0.75 s per simulated second slower). Variable-length delays were dropped at first for the same reason in normal mode, then added (2026-11-19) as a choice next to fixed delays (`p.comm.delays`), for delay sweeps: in Rapid Accelerator they cost about 0.6–1 s per 60 s run (2.2–2.6 s instead of 1.5–1.7 s).

Measured: 1 × 1 layout per plant, nodal network: first run including compile about 18 s, every further run about 3 s; an SCR change without recompiling gives the same result as a full run (V_poc 0.9653 at SCR 3); `grid_strength` (3 cases) 25 s in total. Default layout (2 / 2 / 1): first run about 50 s, further runs about 16 s. Simscape network: about 40 s per further run.

## 12. Figures  (2026-11-18)

`plotAllResults` makes the figures from the saved results, for two purposes with their own folders:
- **report** (`results/figures/report/`): one small figure per point a case set makes, defined in `scripts/reportFigures.m` (cases, signals per axis, time window, a one-line statement of what it shows). 8.8 cm wide (one column), 8 pt Times, at most two or three axes, legend inside the top axis with headroom left for it, references dashed, fault cases zoomed on the fault; vector PDF for LaTeX plus a PNG preview. The statements are checked against the results when the cases are rerun.
- **debug** (`results/figures/debug/`): every logged signal of every case of a set in one figure (P, Q, V, f, plant P with availability, BESS P with set-point, SOC, plant Q), full time plus a zoom on the event; PNG and `.fig`. Lines are thinned (minimum and maximum per bin, so peaks stay), since full-resolution figures of 16 cases ran MATLAB out of graphics memory.
Logged signals: P, Q, V at the POC, V_bus, P_ref, Q_ref, f, P / Q per plant, P_bess_set, available wind and PV power, SOC. Case sets were trimmed for the figures: open-loop cases removed, frequency steps at 50 s and voltage dips at 45 s, after the plant has ramped up following the wind step at 20 s.

## 13. Compiled simulation: Rapid Accelerator  (2026-11-19)

With MinGW (and MATLAB / Embedded Coder) installed, the model can be compiled. Measured on the 32 GB laptop, nominal 60 s case, nodal network, results compared with normal mode:

| Mode | One-time build | Each further run | Difference to normal mode |
|---|---|---|---|
| Normal, Fast Restart | 44–83 s first compile | 8–35 s (depends on machine load) | – |
| Model reference accelerator (controller and asset models as MEX) | 191 s (10 models, serial, cached in `work/`) | 3.4 s | 0 |
| Top and model references accelerator | +6 s | 2.0–2.8 s | 0 |
| **Rapid Accelerator** | 20–50 s | **1.4–2.2 s** | 0 (≤ 2e-12 in all case sets) |

**Rapid Accelerator in use** (`scripts/rapidTarget.m`, `runCase` / `runSweep` with `Mode="rapid"`, default in `main.m`): the target is built once and every run passes its own `p` with `RapidAcceleratorUpToDateCheck` off, so it skips Simulink's own check. `rapidTarget` rebuilds when the compile signature (layout, time step, delay mode with the delays or `comm.maxDelay`, network model) or a model file in `models/` changes. The signature of the last successful build is saved next to the target (`hpp_rapid_fingerprint.mat` in `work/slprj/raccel/HPP`), so a new MATLAB session reuses an unchanged target without rebuilding (nominal case 12.5 s in a fresh session, including loading). All seven case sets (46 cases) run in 68 s with fixed delays (about 15 min in normal mode); for example `hppc_qv` 16 cases in 23 s instead of 328 s. With variable delays: about 2.2 s per 60 s case, `hppc_qv` in 40 s, `comm_delay` (6 cases) in 15 s against 43 s in normal mode with Fast Restart, identical results. Switching between the two modes rebuilds the target (about 30 s); with fixed delays, every new set of delays also rebuilds it.

**Simscape network in Rapid Accelerator: not usable.** Measured (nominal, voltage dip 0.5, SCR 3): the target builds (167 s) and the nominal case matches normal mode (1e-12), but the other cases silently gave the nominal results — the dip did not happen and V_poc stayed at the SCR 10 value 1.0048 instead of 0.9880. The Simscape network's data and grid source are fixed when the program is built. Rebuilding per case (167 s) would be slower than normal mode (107–127 s per run in this test). So `rapidTarget` refuses the Simscape network and runs it in normal mode.

**Fallback to normal mode.** `rapidTarget` returns a status instead of erroring and warns with the reason when Rapid Accelerator is not possible: no C compiler selected, the Simscape network, `checkModels` findings in block expressions, or a failed build. `runCase` then runs the case in normal mode with Fast Restart; `runSweep` runs the remaining cases in normal mode (with Fast Restart when no case changes a compile-time value). A failure is remembered for the same compile-time values and model files, so a case set warns once. Test hook: `setenv("HPP_TEST_NO_COMPILER", "1")`.

**Parallel runs do not help here:** `hppc_qv` on 3 parsim workers took 101 s (plus 29 s to start the pool) against 24 s serial, with identical results. A Rapid Accelerator run takes ~1.5 s and parsim's overhead per run (transfer of the target and data, collecting the output) is larger. A pool would pay off only for long runs (Simscape network, long stop times); `runSweep(..., Mode="rapid", UseParallel=true)` keeps that option. The `Processes` profile is capped at 2 workers; a script can raise this per session (`c = parcluster("Processes"); c.NumWorkers = 3`) without changing the profile.

**Tunable block expressions.** Compiled code (accelerator, Rapid Accelerator, the later DLLs) keeps a block parameter that uses a model argument (`par`, `ap`) tunable only if it is simple arithmetic (+ − × ÷). The first builds failed on `par.PF^2`, `sqrt`, comparisons (`par.RefFlag == 1`, `double(par.MqG == 1)`) and a Transfer Fcn denominator `[ap.gen.Tg 1]`. Fixed by computing derived values in `controllerParameters` (`par.tanphi`, `par.isV`, `par.isPF`, `par.KpQ`, `par.KiQ`, `par.MqG1`) and building the converter current lags from gain 1/Tg and an integrator (`contLag` in `buildHPP.m`, the same dynamics). Expressions inside MATLAB Function code are not affected.

**Run Build (`buildfile.m`, 2026-11-19).** The project's Run Build button and `buildtool` run five tasks, each depending on the one before, so the chain stops at the first failure: `check` (`checkModels(Strict=true)`) → `model` (`buildHPP()`) → `rapid`, the default (`rapidTarget(HPP_parameters())`, errors with `st.reason` when Rapid Accelerator is not possible) → `sweeps` (`runSweep(..., Mode="rapid")` for nominal, freq_step, hppc_p, hppc_qv, wt_frt, bess_soc, the sets of the report figures) → `figures` (`plotAllResults(Purpose="report")`). `model` declares its inputs (`params/*.m`, `utilities/build/buildHPP.m`, `utilities/build/modelInterfaces.m`, `models/ctrl/*.slx`, `models/assets/*.slx`) and its output `models/HPP.slx`, so buildtool skips it when nothing changed; `sweeps` and `figures` declare theirs the same way. Measured: first run (check, model, rapid) 341 s, of which the Rapid build 142 s; second run 61 s with `model` skipped as up to date. `buildtool -tasks` lists the tasks; the cache `.buildtool/` is in `.gitignore`.

## 14. Frequency dynamics, PLLs, GFM and simulation tiers  (planned, 2026-11-19)

Goal: study how the HPP and the power system respond to each other in frequency, with grid-following (GFL) converters with real PLLs and later grid-forming (GFM) converters. The grid frequency today is an imposed signal (`p.grid.fev`, a step) and every asset is a current source aligned with the terminal voltage of the previous step (an ideal PLL in disguise). Both have to become dynamic.

### 14.1 Three simulation tiers

**Revised 2026-11-25 (section 14.9):** the dq tier is set aside. The tiers are now **RMS** (nodal network, the main tool) and **Simscape** with one network in two modes, **phasor** (frequency-and-time) and **EMT** (time mode), for checks. The table below is the original plan of 2026-11-19.

| Tier | Network | Converters | Step | Use | Run time per 60 s case (estimate) |
|---|---|---|---|---|---|
| **RMS** (today) | algebraic V = Z·I at 50 Hz (nodal solver) | IEC / WECC RMS models | 1 ms | sweeps, dispatch, grid code, frequency events | 1.5–3 s (measured 1.5 s without stages 6–7) |
| **dq** (planned) | linear state-space RLC network in a dq frame, discretized | averaged converters with inner dq current loop, PLL or GFM control | 50–100 µs (controllers keep their rates) | swing dynamics, PLLs, GFM, weak grid, balanced faults | 10–50 s; a 10 s event window 3–8 s |
| **EMT** | three-phase instantaneous (Simscape time mode, or PSCAD) | averaged (or switching) converter models; the controllers as DLLs in PSCAD | 10–20 µs | validation of critical cases, unbalanced faults, harmonics, protection | Simscape 15–60 min (normal mode only), PSCAD 2–20 min |

The controllers (HPPC, plant controllers, asset controllers) are the same models in every tier; only the network variant and the converter models underneath change. This keeps the DLL path intact. The tier is chosen per run like the network model today (Variant Subsystem, setting in `p`, option in `runCase` / `runSweep` / `main.m`). The run times are estimates scaled from the RMS tier; they are measured once stage 6 and a dq prototype run.

**Accuracy.** For a balanced network of lumped R, L, C with averaged converters, the dq model is an exact transform of the EMT equations; the only difference is the integration error. dq misses what positive-sequence averaged models cannot carry: unbalanced faults and negative-sequence control, the 100 Hz PLL ripple under unbalance, switching harmonics, saturation and inrush, frequency-dependent cable models; trapezoidal discretization lowers resonance frequencies (a 2 kHz resonance appears at 1.94 kHz with 50 µs, 1.79 kHz with 100 µs). RMS also drops the network's electromagnetic dynamics: it is accurate below about 5 Hz (frequency control, swing dynamics, inertia, FFR, Q / V loops) but optimistic for fast PLLs and GFM in weak grids, where the network's inductive dynamics add phase lag that RMS does not have. Validation plan: the same balanced cases (frequency event, PLL bandwidth sweep, balanced fault) in all three tiers with averaged converters; RMS against dq shows where RMS stops being valid, dq against EMT confirms the dq model.

| Dynamics (rough range) | RMS | dq | EMT |
|---|---|---|---|
| Dispatch, ramps, SOC, governors (< 0.1 Hz) | ✔ | ✔ | ✔ |
| Swing, inertia, nadir, FFR, Q / V loops (0.1–5 Hz) | ✔ | ✔ | ✔ |
| Slow PLLs (≤ ~10 Hz), FRT current injection | ≈ | ✔ | ✔ |
| Fast PLL, weak-grid instability, GFM–network interaction (10 Hz – 1 kHz) | ✘ | ✔ | ✔ |
| Resonances > 1 kHz, harmonics, switching | ✘ | ≈ / ✘ | ✔ |
| Unbalanced faults | ✘ | ✘ (without a negative-sequence network) | ✔ |

### 14.2 Common reference frame and asset–network interface

All tiers use one reference frame rotating at ω0 = 2π·50 rad/s. Every source has its own angle θ in that frame and its frequency f = 50 + (dθ/dt)/2π. The interface between an asset and the network becomes, per instance:
- **GFL:** current phasor (d, q) in the common frame, injected along the asset's PLL angle;
- **GFM:** internal voltage (d, q) behind an impedance (filter / virtual), with the angle from the GFM control;
- back from the network: the terminal voltage (d, q) in the common frame.
Frequencies that controllers use are measured (PLL) in every tier; no controller reads the system frequency directly.

In the nodal (RMS) solver, voltage sources enter as Norton equivalents (E/Z as a current injection, Z added to Y), so the grid machine, GFL and GFM sources all stay in the one V = Z·I step. The frame at ω0 drifts with the system frequency (0.2 Hz for 60 s is 12 turns); angles are wrapped.

### 14.3 Stage 6: grid frequency model (swing equation)

- The grid equivalent becomes a voltage source behind the grid impedance (SCR, X/R as now) whose angle comes from an aggregated system: 2H·dΔω/dt = ΔPm − ΔPe − ΔPdist − D·Δω, Δθ̇ = ω0·Δω. ΔPm from an aggregated governor (droop R, turbine lag; option reheat), ΔPdist a load step or loss of infeed (time, size), ΔPe includes the HPP's change in output, so the plant's response feeds back.
- Parameters (new `p.grid.sys`, typical): system rating Ssys, inertia H, load damping D, governor droop R and time constants, disturbance. Option: frequency imposed as now (for grid-code tests) or ramps / RoCoF / recorded profiles, so the current case sets keep working.
- RMS tier: reactances at the measured frequency (X = ω·L) as an option, and frequency-dependent load through D.
- Validation: nadir, RoCoF and settling frequency for a known ΔP, H, R, D against the analytic solution, with the HPP response off; then with LFSM / FSM on.
- Measured frequencies (POC for the HPPC, busbar for the plant controllers) come from the network through measurement PLLs (stage 7); until then from the grid machine.

**Status (2026-11-19): built, except the Energinet profiles.** New top-level block `Grid frequency` (`buildGridFrequency` in `buildHPP.m`, masked, parameter `g = p.grid`, tunable): one MATLAB Function with the swing equation (forward Euler at `p.sim.Ts`) and the imposed frequency (step `p.grid.fev` plus a 10-point profile `p.grid.fprof`, for the Energinet test profiles). Outputs f [pu] to the TSO bus (HPPC `f_meas`) and to the network bus (plant controllers), and the grid angle theta, which rotates the grid source in both network models (nodal: E·e^jθ in the Norton source; Simscape: phase input). P_poc enters one step old. Parameters `p.grid.sys` (`swing` 0 / 1, `Ssys` 2 GW, `H` 4 s, `D` 1, `R` 0.2, `Tg` 0.5 s, `Tt` 5 s, `tOn` 39 s, `dist.t0` 40 s, `dist.dP`). The system is taken as balanced at `tOn`: the HPP ramps up from start until about 38 s, so an earlier `tOn` shifts the frequency (+0.03 Hz with 30 s). Case set `sys_freq`.

Checks (default layout, Rapid Accelerator, 16 s for the 6 cases of 90 s):
- Imposed mode: nominal identical to the saved results; `freq_step` within 0.1 MW (the grid angle now rotates with the imposed frequency, a one-step lag of the asset current angle).
- Swing equation against the analytic solution of the same linear system (loss of 3.5 %, HPP frequency response off): within 0.2 mHz. Nadir 49.425 Hz after 4.9 s, settles at 49.709 Hz, initial RoCoF −0.219 Hz/s (−0.177 Hz/s averaged over the first 0.5 s).
- With the HPP: LFSM-U (default) +5.1 MW, nadir 49.536 Hz (+0.11 Hz), final 49.729 Hz; FSM +16.7 MW, nadir 49.635 Hz (+0.21 Hz), final 49.780 Hz; FSM with H 2 s: RoCoF −0.33 Hz/s, nadir 49.555 Hz. Load drop 3.5 %: LFSM-O −5.1 MW, peak 50.463 Hz.
- Not yet: the Energinet profiles in `p.grid.fprof` (values to be taken from Energinet's verification documents); frequency-dependent reactances in the RMS network (optional); measured frequency (stage 7: until then the controllers get the system frequency directly, the HPPC without delay).

### 14.4 Stage 7: PLLs

- One SRF-PLL per asset (error from the angle of the terminal voltage, PI, θ and f) with tunable bandwidth and damping; the asset injects its current along the PLL angle. Measurement PLLs (or filters with a window) for the plant controllers and the HPPC.
- RMS tier: PLL dynamics on the phasor angle (≈ valid for slow PLLs); dq tier: the main tier for PLL studies.
- Studies: PLL bandwidth against SCR (stability map), RoCoF and phase jumps, FFR and synthetic inertia (P ∝ df/dt) with measured frequency.

**Status (2026-11-19): built in the RMS tier.** Parameters `p.pll` (`params/pllParameters.m`; gains and window lengths derived in `deriveParameters`).
- **Asset PLL** (section `PLL` in every asset controller `WT_ctrl`, `PV_ctrl`, `BESS_ctrl`, code `pllVariant` / `ctrlPll` in `buildHPP.m`; a Variant Subsystem with the MAF-PLL and a simple SRF-PLL, chosen per run by `p.pll.type` (global variant control variable `HPP_pll`), parameters `par.pll` = `p.pll.asset` at the controller rate): SRF-PLL on the terminal voltage phasor, $e = v_q/|v|$ in the PLL frame, PI with $k_p = 2\zeta\omega_n$, $k_i = \omega_n^2$, $\omega_n = 2\pi f_{BW}$; frequency limited to ±5 Hz, held below 0.05 pu. The converter injects $I = (i_p - j i_q)e^{j\theta}$ along the PLL angle and $P + jQ = V I^*$. Default `fBW` 5 Hz, ζ 0.707, prefilter `Tpre` 20 ms. The PLL sits in the controller, as in a converter DLL, so the same asset controllers serve the RMS and (stage 8) the dq network; the controllers have two extra inputs (`V_re`, `V_im`, terminal voltage phasor) and two extra outputs (`th_pll`: the angle the converter part injects along; `f_pll`: the asset frequency, for asset-level frequency functions later). In the asset model the blocks Generator system (WT) or Converter (PV, BESS), Current injection (along `th_pll`) and PQ measurement follow the controller.
- **Prefilter** = moving average over one cycle of the voltage **in the PLL frame** (MAF-PLL), so its 10 ms delay is inside the loop. A first version averaged the phasor in the fixed frame, before the angle comparison: the delay was then outside the loop and every bandwidth up to 40 Hz looked stable. In the RMS tier there are no harmonics to filter, so the MAF only contributes its delay; that delay is what limits a real MAF-PLL.
- **Frequency meters:** every controller has its own meter (masked subsystem `meter`, `freqMeterBlock`: the same PLL with prefilter, f averaged over 20 ms, RoCoF over 500 ms) with its own parameters (`p.pll.hppc`, `.wppc`, `.sppc`, `.bessc`), placed where the controller is: in the HPP controller subsystem on the POC voltage phasor (gives the HPPC `f_meas`), and in each plant subsystem on the 33 kV busbar phasor (gives the plant controller `f_meas`). The system frequency reaches the meters only (TSO bus `f_meas` = system frequency for the HPPC meter, network bus `f` for the plant meters), so `on = 0` gives the system frequency as before. Both network models output the busbar and POC phasors (`V_bus_re/im`, `V_poc_re/im`). Logged: `f_sys`, `f_wpp`, `f_spp`, `f_bess` (in each plant's measurement bus), `f` and `RoCoF` (HPPC meter, output 4 of the HPP controller subsystem). Check: with the wind plant meter at 1 Hz and the others at 5 Hz, a +0.5 Hz step gives 50.533 / 50.490 Hz 0.2 s after the step.
- **Network block:** the grid frequency model (`Grid frequency`, stage 6) now sits inside the top-level `Network` subsystem, next to the network Variant Subsystem `Network model` (nodal / Simscape), since the grid machine is behind the grid impedance. `Network` outputs the network bus (its `f` = system frequency), the POC bus and the system frequency; it has no frequency input.
- **Phase jump** of the grid source `p.grid.pjump` (t0, dphi) to excite the PLLs; case set `pll_scr`.
- `p.pll.<asset, hppc, wppc, sppc, bessc>.on = 0` give the behaviour before stage 7 exactly (ideal angle, system frequency); unchanged after the meters were split, the grid frequency moved and the meters / asset PLL moved into the controllers' subsystems (≤ 2e-12 MW; defaults identical too).

Checks (Rapid Accelerator):
- PLLs off: nominal, `wt_frt`, `hppc_qv`, `sys_freq` identical to the saved results (≤ 2e-12 MW); `freq_step` within 0.5 MW (its saved results predate stage 6).
- Default PLLs (5 Hz, MAF): steady-state cases within 0.7–0.9 MW of the ideal PLL (start-up and the wind step); final values equal. LFSM-O +0.5 Hz: same final P (−21.6 MW), 90 % after 0.33 s instead of 0.29 s; measured and system frequency agree to 2e-9 pu. Voltage dip 0.5: P back within 2 % 0.06 s after clearing (0.13 s with the ideal PLL).
- **Bandwidth against SCR, 2° phase jump (small signal), MAF on:** 5 Hz well damped (ratio of successive peaks 0.2–0.3); 10 Hz poorly damped (0.45 at SCR 10, 0.75 at SCR 1.5); 20 and 40 Hz unstable (growing / sustained oscillation) at every SCR. Without the MAF (30 Hz) stable. This agrees with the loop: crossover near $2\omega_n$, phase margin of the PI about 65°, minus the MAF delay $\omega \cdot 10$ ms (≈ 30° at 5 Hz, ≈ 60° at 10 Hz).
- `pll_scr` (20° jump): 20 Hz oscillates without decaying (74 MW at SCR 1.5); 2–10 Hz settle in 0.5–2 s, slowest at SCR 1.5.
- The RMS tier has no network electromagnetic dynamics, so these limits are optimistic for weak grids; the dq tier (stage 8) repeats the map.

### 14.5 Stage 8: dq network  (tried, set aside 2026-11-25)

**Status:** step 1 built on branch `dq-network` (pushed, not merged): `params/networkDqData.m` (the nodal network as a discrete state-space model in the ω0 frame, 32 real states, asset terminals v_term = v_tap + Zt·I, exact zero-order hold, steady-state start) and the network choice Dq (`buildNetworkDq`, `HPP_network = 3`). It matched the nodal network in steady state (1e-14 pu) and at SCR 10 / 5 (transients ≤ 2 MW), and oscillated at SCR ≤ 4 with the RMS asset models (current sources without inner loop or filter: a dynamic network needs converters of the same fidelity). Set aside because industry practice is RMS for grid studies and EMT for weak-grid / IBR checks, with dq / dynamic-phasor models mostly used for academic small-signal analysis, which the PhD question does not need (14.9). The plan as it was:

- Network in the ω0 frame: inductors L di/dt = v − R·i − jω0·L·i, capacitors C dv/dt = i − jω0·C·v; the whole network as dx/dt = A·x + B·u, inputs the asset currents / voltages, outputs the bus voltages. Built from the same network data as the nodal solver (`networkParameters`).
- With the frame fixed at ω0, A and B are constant: discretize once (trapezoidal / Tustin) in `deriveParameters`, x[k+1] = Ad·x[k] + Bd·u[k], as tunable constants like today's Z. One matrix–vector product per step; works in Rapid Accelerator.
- Option: frame rotating with the system frequency ω(t) (constant steady-state values in long deviations): the cross terms jω(t)L, jω(t)C make it nonlinear; kept cheap by treating the small deviation as an input, x[k+1] = Ad·x[k] + Bd·u[k] + Bd′·(Δω·A1·x[k]), with Ad at ω0. Both frames give the same physics; comparing them is a check.
- Converters: averaged models with an inner dq current loop and filter at the small step; the asset controllers' outer loops and the plant / HPP controllers keep their rates (multirate).
- Positive sequence only; a negative-sequence network for unbalanced faults is a later option.

### 14.6 Stage 9: GFM

- A GFL / GFM choice per asset type (Variant Subsystem, first for the BESS): droop or VSM angle control, virtual impedance, current limiting (with fallback), in the dq tier; a simplified RMS version for sweeps.
- Studies: GFM against GFL for inertia and nadir, weak-grid stability, interaction with the plant and HPP controllers, central against decentral.

### 14.7 EMT tier

- Simscape Electrical in time mode, or PSCAD with the controllers as DLLs (the original plan). Converters as averaged voltage sources with an inner current loop and filter (14.9, work package C); the RMS asset models (IEC, WECC) are not valid in EMT.
- Revised 2026-11-25: the same Simscape network as the phasor (frequency-and-time) tier, only the solver configuration differs (14.9). Rapid Accelerator becomes possible once the network's block parameters are run-time configurable and the grid events are signals (work package A); before that Simscape parameters did not change in Rapid Accelerator (section 13).
- Work package A (2026-11-25), Rapid Accelerator: the cause was not the events but `p` itself. When `p` feeds any Simscape compile-time value (the solver configuration's `LocalSolverSampleTime`, the rated frequency `FRated` of sources and transformers, the Simscape clocks' sample times), Simulink warns "Unable to tune the variable 'p'" and ignores the whole `p` in Rapid Accelerator, so a dip, phase jump or SCR change had no effect. These values now come from model-workspace variables set per run with the variant choices (`variantVars`): `HPP_Ts` (`p.sim.Ts`), `HPP_Tnet` (solver step, `netSolverStep`) and `HPP_f0` (`p.grid.f0`). Everything else stays in `p` and is tunable; `rapidTarget` no longer refuses the Simscape network. Check: nominal, dip 0.5, SCR 3 and a 20° phase jump in Rapid against normal mode differ by at most 1.4e-5 (W / var on 150 MW, the same level as before the change between compiled and interpreted Simscape); no tuning warning. Normal-mode results are identical to before (difference 0).
- Mode: `p.net.simscapeMode = "phasor" | "emt"` (`networkParameters`, default phasor), variant control `HPP_simscape` (`simscapeMode`), part of `compileSignature`. The Solver Configuration is a Variant Subsystem (frequency-and-time / time, both with a fixed-step backward-Euler local solver at `HPP_Tnet`). In EMT the local step is `p.net.emtTs` (50 µs; `p.sim.Ts` must be a multiple), and the model's fundamental step follows, because the local solver rate must be a multiple of it. The measured voltages and P, Q pass Zero-Order Holds at `p.sim.Ts`, so the controllers keep their rates in both modes. The voltage phasor gets a one-cycle moving average (Discrete FIR, `round(1/(f0·Ts))` taps) in EMT and no filter in phasor mode (variant). Both modes share one Rapid Accelerator target folder; a mode change rebuilds the target (the fingerprint includes the mode). Check (nominal, 2026-11-25): EMT Rapid against EMT normal over 0.2 s differs by 8.7e-7; the EMT Rapid build took 2 min, 60 s of EMT in Rapid Accelerator 222 s (phasor Rapid 150–180 s, with the machine under the same load). The log keeps the phasor rate (6001 samples over 60 s). EMT against phasor at 60 s: P_poc 149.9 / 150.0 MW, V_poc 1.0053 / 1.0048 pu; largest differences over the run P 0.6 MW, Q 13 Mvar, V 0.007 pu (where in time not yet checked).

### 14.8 Decided (2026-11-19)

| Topic | Decision |
|---|---|
| System frequency model | **Nordic-like**, scaled so the plant's effect is visible: Nordic per-unit values (inertia H about 3–5 s on the system rating, governor droop and FCR-like time constants, small load damping) on a much smaller system rating Ssys (start about 2 GW, swept), and a disturbance of the same relative size as the Nordic dimensioning incident (about 1.4 GW of about 40 GW, so about 3.5 % of Ssys). Exact values in `p.grid.sys`, to be set in stage 6. |
| Grid structure | The machine sits **directly behind today's grid impedance** (SCR, X/R as now). One area. |
| Imposed frequency profiles | **Energinet test profiles** (from Energinet's test and verification documents for LFSM / FSM / FFR; to be taken from the current documents). Steps and the swing-equation grid remain. |
| PLL | SRF-PLL **with a prefilter** on the measured voltage (for example a second-order generalised integrator (SOGI) or a moving-average filter over one cycle), tunable bandwidth. |
| Frequency measurement for the controllers | Proposed (to be confirmed): the same prefiltered PLL, its frequency averaged over one cycle (20 ms) for control, and RoCoF over a sliding window (ENTSO-E uses 500 ms for RoCoF withstand). One measurement block, used by the HPPC and the plant controllers. |
| Converter (dq tier) | Averaged model with an **LCL filter**, a **simple DC link** (capacitor with a DC voltage controller and a power source / storage behind it) and **angle-preserving current limiting** (the current reference is scaled down in magnitude, keeping its angle). |
| Step size (dq tier) | Not fixed; **100 µs** to start, chosen after measuring speed and accuracy. |
| GFM | **Virtual synchronous machine (VSM) in swing-equation form** (virtual inertia and damping on the converter's internal angle), **BESS only** first; wind and PV later. |
| EMT tier | **In Simulink** (Simscape Electrical, time mode) for now; PSCAD later with the DLLs. |
| Unbalanced faults | **Positive sequence only** in the dq tier first, to measure the run time; a negative-sequence network later. |

Order of work: stage 6 (swing equation, Energinet profiles, frequency measurement) and stage 7 (PLLs) in the RMS tier first, then the dq network (stage 8) with the LCL / DC link converter for the BESS, then the BESS VSM (stage 9), then the EMT tier for validation. (Superseded by 14.9.)

### 14.9 Decided (2026-11-25): RMS plus Simscape phasor / EMT

| Topic | Decision |
|---|---|
| Tiers | **RMS** (nodal network) is the main tool, as in industry grid studies with the same generic models (IEC 61400-27-1, WECC). **Simscape** for checks: one network, **phasor** (frequency-and-time) or **EMT** (time mode). The dq tier is set aside (14.5). |
| One network, two modes | Only the Solver Configuration changes: `EquationFormulation` frequency-and-time or time, local step `p.sim.Ts` or about 20–50 µs. The interface already fits both: three-phase controlled current sources fed with magnitude and phase, voltages back as the space phasor at ω0. EMT adds a one-cycle filter on the measured voltage phasor. The mode is a compile-time parameter (`p.net.simscapeMode`, in `compileSignature`); each mode can keep its own Rapid Accelerator target (cache folder). |
| Rapid Accelerator for Simscape | Network block parameters run-time configurable, grid events (dip, phase jump) as signals, fixed-step local solver; check: dip and SCR cases equal in Rapid and normal mode. |
| Initialisation | Saved **operating points** (complete model state, per build and pre-event condition) instead of long settling; cases simulate an **event window** (event times relative to the start). Also for RMS sweeps. |
| Converter for Simscape EMT | Averaged converter with **inner dq current loop**, **LCL filter**, **simple DC link**, **angle-preserving current limit**, as a variant of the asset's converter part (the RMS current source stays the default). |
| EMT studies | Aggregated plants (one converter per plant or feeder); weak grid, PLL bandwidth, FRT; compared with RMS to show where RMS holds. |

Work packages (each on its own branch from main, regression against main before merging; models are rebuilt after each merge):
- **A** Simscape in Rapid Accelerator, then the phasor / EMT mode; *status 2026-11-25: Rapid Accelerator works for the Simscape network (agrees with normal mode to 1.4e-5; normal mode unchanged), phasor / EMT mode works (EMT nominal 60 s in Rapid Accelerator in 222 s; 14.7)*;
- **B** operating points and event windows;
  **Status B (2026-11-25): built, checked in Rapid Accelerator and normal mode (Fast Restart).** `runCase` / `runSweep` option `Window=[pre post]` (e.g. `[1 Inf]`). Event times stay absolute: `eventWindow` takes the earliest active event of the case set (`eventTimes`: `grid.Pstep`, `Qstep`, `dip`, `fev`, `pjump`, `sys.dist`), starts the window `pre` s before it, rounded down to the slowest sample time (0.1 s), and stops at `min(event + post, Tstop)`. One window per set, so the cases still plot on one time axis. `operatingPoints` saves the complete model state at the window start (SaveOperatingPoint, a run of the case from 0) and starts the run from it (`setInitialState`). The cache is `<cache folder>/hpp_operating_points`. The key is (1) the build: mode, `compileSignature`, model files, release, and in Rapid the target's fingerprint file; (2) the pre-event condition: all of `p` except `sim.Tstop`, `meta` and the event groups after the window start. Cases that differ only in the event share an operating point: freq_step needs 3 for 9 cases, wt_frt 2 for 4, sys_freq 4 for 6, pll_scr 12 for 12. Results start at the window start (`caseInfo.window`, `window` in `sweep.mat`), so min / max in `summarizeResults` cover the window only. Check: windowed against full runs, every logged signal identical (difference 0) in freq_step, wt_frt, pll_scr and sys_freq (Rapid), and in wt_frt (normal mode, Fast Restart). Speed: in Rapid the RMS tier gains nothing. A run has about 1.3 s fixed cost, the 60 s simulation takes about 1 s, and restoring the operating point adds about 1 s (e.g. wt_frt: 2.3–2.7 s full against 2.4–3.2 s windowed). In normal mode with Fast Restart wt_frt took 34 s windowed against 79 s full (4 cases, compile included). The window pays off where simulation time dominates (normal mode, Simscape phasor / EMT).
- **C** converter variant (status: built, 14.10);
- **D** (after A–C) the EMT case set and the RMS comparison.

### 14.10 Converter variant: averaged converter  (work package C, 2026-11-25)

**What.** The converter part of every asset has two models, chosen per run by `p.conv.type`. It is a tunable value (`ap.conv.type`), not a variant control variable: switching needs no rebuild, in Rapid Accelerator and in Fast Restart, and `variantVars`, `compileSignature` and `runSweep` are unchanged (see *Why not a variant* below):
- **1 Current source** (default, RMS): I = (i_p − j·i_q)·e^{jθ_pll}, as before.
- **2 Averaged converter**: inner dq current loop in the PLL frame, LCL filter, simple DC link, angle-preserving current limit. Meant for dynamic networks (Simscape EMT, work package D); with the nodal network it runs for comparison.

**Where it sits.** The block `Current injection` in each asset model holds both models side by side (`Current source`, and `Averaged`, an atomic subsystem at the model step) and a Switch per output on `ap.conv.type`. The averaged converter's MATLAB Function returns at once when type 1 is selected, so the RMS default costs almost nothing extra. Its ports are i_p, i_q, θ_pll, V_re, V_im in and I_re, I_im out; the current source uses the first three. The asset's top level, its ports and its layout template are unchanged. The Generator system (WT, IEC Type 4) / Converter (PV, BESS, WECC REGC_A) block stays in front of both choices with its ramps, LVPL and LVACM. Its **Tg lag is bypassed with the averaged converter** (inside each lag block a Switch on `ap.conv.type` between the lag output and its input): Tg is the RMS model's stand-in for the inner current loop. Counting it twice would add an artificial 20 ms lag exactly in the dynamics the EMT checks are meant to test. The network interface stays a current phasor (I_re, I_im) out and the terminal voltage (V_re, V_im) in per asset, so both network models take either choice.

**Model** (`params/converterParameters.m`; per unit on the asset rating, reactances at 50 Hz; in the common frame at ω0):
- LCL filter: (L1/ω0)·di1/dt = v_c − v_n − R1·i1 − jL1·i1;  (Cf/ω0)·dv_C/dt = i1 − i2 − jCf·v_C, v_n = v_C + Rd·(i1 − i2);  (L2/ω0)·di2/dt = v_n − v_g − R2·i2 − jL2·i2. The network sees the grid-side current i2. L1 0.10, R1 0.003, Cf 0.05 (5 % reactive power), L2 0.03 (the unit transformer is already in the network), R2 0.002; passive damping Rd = 1/(3·(f_res/f0)·Cf) = 0.23 pu in series with Cf; resonance 1.47 kHz.
- Current loop (PLL frame, d along θ_pll): reference i1* = (i_p − j·i_q) + ΔP/|v_n| + jCf·v_n (grid current, DC voltage correction, capacitor current: steady-state Q equals that of the current source), limited to |i1*| ≤ 1.2 pu with its angle kept. PI tuned by internal model control (k_p = α_c·L1/ω0, k_i = α_c·R1, α_c = 2π·500 Hz, so the loop is α_c/s), decoupling jL1·i1 and feed-forward of v_n. Modulation limit |v_c| ≤ 1.2·V_dc, with no integration while limited. No PWM or computation delay (averaged model).
- DC link: stored energy H_dc·V_dc² with H_dc = 5 ms; H_dc·d(V_dc²)/dt = P_src − Re(v_c·conj(i1)). The grid-side converter holds V_dc (PI on V_dc², 20 Hz, ζ 0.7, output ΔP). The source behind the link (WT machine side, PV DC-DC / MPPT, battery) delivers the commanded power Re(v_g·conj(I*)) through a 5 ms lag. A chopper clamps V_dc at 1.15 pu (surplus power while the current limit acts). In steady state the grid power is the commanded power minus the filter losses (about 0.5 % at rated current); the plant and HPP P loops correct that.
- The DC voltage (`Vdc`, the third output of the MATLAB Function) ends in a terminator; it is not an asset output or logged.

**Step and multirate.** The inner loop and the filter need a step of tens of µs, while the model step and all controllers stay at 1 ms. The averaged converter is therefore one MATLAB Function per asset at the model step, with n = round(Ts/Tfast) sub-steps (Tfast = 50 µs, so 20 per 1 ms step). Within the step the terminal voltage and θ_pll are held, as with the current source; the filter is advanced by its exact zero-order-hold discretization (Ad, Bd from `expm`, computed in `deriveParameters` as tunable real and imaginary parts in `ap.conv`). The current at the end of the step goes to the network. The model stays fixed-step with ode1, single-tasking, and works in Rapid Accelerator. With an EMT network step of 20–50 µs (work package A), Ts ≤ Tfast gives n = 1, so the converter runs at the network step and the controllers keep their own rates. Run time (Rapid Accelerator, nodal network, default layout): the averaged converter makes a run 1.5–1.7 times slower (wt_frt 4 cases 12.7 s against 8.7 s, pll_scr 12 cases 37.6 s against 22.7 s, hppc_qv 16 cases 63 s against 38 s). With the current source selected the run time is unchanged (the MATLAB Function returns at once). Both use the same compiled target.

**Why not a variant.** First built as a Variant Subsystem (and a Variant Source for the Tg bypass) with a global control variable `HPP_conv`, like the PLL. The asset models are referenced models, and the Rapid Accelerator / model reference build then worked only for the choice that was active when the asset model was last saved. The other failed with "computed interface information must match saved interface information for referenced model BESS_asset", from a clean cache too, with activation at update diagram and at startup alike (checked by building 1 → 2 → 1 with the models saved under each value). So per-run switching would have needed the asset models resaved. A Switch on a tunable value has neither problem, and switching needs no rebuild at all. The same may apply to `HPP_pll` (a Variant Source in the referenced PLL model); `pll.type = 2` was not tried in Rapid Accelerator here.

**Checks (2026-11-25, Rapid Accelerator, nodal network, default layout).** Reference: the six case sets nominal, wt_frt, freq_step, pll_scr, bess_soc, hppc_qv (49 cases) on main before the change.
- **Current source (default):** after rebuilding the three asset models all 49 cases are bit-identical to main (largest difference 0 in every logged signal), and the run times are the same (hppc_qv 38.3 s against 37.4 s).
- **Averaged converter**, the same 49 cases against the current source; the differences are findings with their causes, not errors:
  - Steady state: P_poc, Q_poc and V_poc are equal (the HPPC closes its loops). The wind plant gives 0.63 MW and the PV plant 0.07 MW less: the filter and damping losses, about 0.5 % at rated current. The BESS makes up the difference (+0.70 MW). Where nothing makes it up, P_poc ends 0.7–0.8 MW lower: bess_soc "SOC0 0.15, +20 MW" (BESS at its limit) and freq_step "-0.2 Hz, decentral" (open-loop HPPC). Plant Q is the same (the capacitor current is compensated in the reference).
  - Start-up: P_poc up to 5.5 MW higher at 0.23 s, because the current rises without the Tg lag.
  - Voltage dips (wt_frt): during the dip P and Q are within a few MW / Mvar (dip 0.5: P_poc 99.8 against 101.3 MW 50 ms into the dip; rms difference over the run 0.7–2 MW / Mvar). The largest differences are single samples at clearing (45.16 s, up to 54 MW (dip 0.5 at SCR 3) and 130 Mvar (dip 0.8)): with the current source the reactive current runs down through Tg (20 ms), so Q_poc is still 52–172 Mvar 10 ms after clearing; the averaged converter has already dropped it. Recovery and final values are the same.
  - Q / V steps (hppc_qv) and frequency events (freq_step): faster first response (up to 10 Mvar more at the step), the same final values.
  - PLL against SCR (pll_scr, 20° phase jump): the PLL / weak-grid interaction changes, as expected once the 20 ms lag is gone and the current responds within the step. 2–5 Hz: as with the current source. 20 Hz, unstable with the current source at every SCR: still a sustained oscillation, but half the amplitude at SCR 10 and 3 (7 / 5.5 MW peak to peak against 13 / 16 MW), and 25 MW against 133 MW at SCR 1.5. 10 Hz at SCR 1.5: **settles with the current source but oscillates with the averaged converter** (sustained, 67 MW peak to peak). Here the converter responds within one step to a weak algebraic network that sees its current one step late (1 ms). This mixed fidelity (dynamic converter on an algebraic network) is the reason the averaged converter is meant for the Simscape EMT network; work package D should repeat these cases there before reading anything into them.
- The converter code alone (a MATLAB copy against a Thevenin source, strong and weak, ramp, 0.2 rad phase jump, 0.5 pu dip with reactive current) settles to P = |V|·i_p minus losses and Q = 0, with V_dc back at 1.0 pu; the chopper takes the surplus power at fault clearing.
- Not checked: the DC voltage in the full model (not logged), the Simscape network with the averaged converter, other layouts.

