# HPP_model: architecture of the hierarchical plant model

Status: **stages 1–4 built** (2026-11-16): structure, N instances, communication delays, parameter structure, the phasor network, the IEC 61400-27-1 Type 4A wind turbine, the WECC REEC_B / REEC_C + REGC_A models for PV and BESS, closed-loop Q control in every asset controller, and REPC_A plant controllers (WPPC, SPPC, BESSC) with their own P, frequency and Q/V loops. Since 2026-11-17 also: two network models chosen per run (nodal and Simscape Electrical, section 5) and faster runs with Fast Restart, where almost all parameters change without recompiling (section 11). The HPPC still has placeholder (open-loop) logic. Stage 5 not started.
Values marked *typical* are starting points to be replaced by project data.

## 1. Decisions

| Topic | Decision |
|---|---|
| Simulation type in Simulink | **RMS (positive-sequence phasor)**. EMT validation is done in PSCAD. |
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

**N assets (decided in stage 1).** Each asset type is one parameterised file, instantiated N times by the build script `utilities/buildHPP.m`, which generates `HPP.slx` with N Model blocks per type and wires them. Instance k gets the model argument `ap = p.<type>.ap(k)` (asset parameters including its controller's `par`). `HPP.slx` stores the N it was built for, and `runCase` / `runSweep` stop if the parameters differ. A For Each subsystem was not used: the build script is simpler to debug, and every instance stays a normal Model block, which code generation and PSCAD handle the same way.

The plant controllers use fixed-width vector ports (width N, from their model workspace), so a DLL is built for one N. Changing N means rerunning `buildHPP(All=true)`, which is acceptable.

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

Functions, stage 5: P control at POC with ramp limits, Q / V / PF control at POC, frequency response (FSM / LFSM-O/U), dispatch between plants, POC limits.

### 4.2 Plant controllers  (base: plant rating; vectors of width N)

| In | Unit | Out | Unit |
|---|---|---|---|
| `P_set`, `Q_set` from HPPC | pu | `P_set_k`, `Q_set_k` (N) | pu |
| `P_meas`, `Q_meas`, `V_meas` at plant connection | pu | `P_avail`, `P_plant`, `Q_plant` to HPPC | pu |
| `P_avail_k` (N) from assets | pu | BESSC only: `P_chg_max`, `P_dis_max`, `SOC` | pu, – |
| BESSC only: `SOC_k` (N) | – | | |
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

Delays are parameters (`p.comm.*`) so case sets can sweep them. Implemented as a discrete delay of whole simulation steps (`round(delay / p.sim.Ts)`), fixed when the model is compiled; the receiving controller samples it at its own rate. Every measurement path into a controller also has a one-step memory (1 ms): without it, the loops through the controllers' direct feedthrough are algebraic loops when a delay is 0. The TSO delays start at the initial set-point, so the HPPC does not see a zero reference at start-up.

## 7. Parameters

```
p.sim            solver step 1 ms, stop time, logging decimation (every 10 ms)
p.grid           POC, base, grid SCR, X/R, frequency
p.net            transformers (incl. magnetizing branch), cables, lengths per feeder; solverType = default network model
p.comm           communication delays
p.wt  / p.pv / p.bess   nFeeders, nPerFeeder, nAgg, Pn (N, Prated derived), asset-model parameters
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
   - *Grid frequency:* `p.grid.fev` (step at `t0` by `df`, pu) drives a new `f` element of the network bus (plant controllers) and `f_meas` on the TSO bus (HPPC). Case set `freq_step` (+0.2 Hz, −0.2 Hz, +0.2 Hz with `Frqflag` = 0 in all plants).
   Checks (nominal case): each plant now meets its own set-point at its busbar feeder (Q_wpp, Q_spp, Q_bess → 0 within about 5 s; P_bess = −5.00 MW exactly); wind and PV stay at their available power, which they cannot exceed to cover losses. Q_poc improves from −21.3 to −10.4 Mvar (the rest is the 132 kV transformer and grid side, for the HPPC); P_poc 146.47 MW. Frequency +0.2 Hz: each plant reduces P by 5 % droop × (0.2 Hz − 20 mHz) = 7.2 % of its rating relative to its set-point (wind 135 → 124.2 MW, PV 20 → 16.4 MW, BESS −5 → −7.16 MW; P_poc 146.5 → 133.2 MW); −0.2 Hz: only the BESS responds (−5 → −2.84 MW); `Frqflag` = 0: no response. Dip 0.5: V_poc ≈ 0.56 pu, Q wind ≈ 56, PV ≈ 17, BESS ≈ 10 Mvar; P_poc back at 146.1 MW 0.35 s after clearing. Dip 0.8: Q wind ≈ 67 Mvar, P_poc back to 146.5 MW within 3 s. Since the HPPC is still open loop, nothing at the POC counteracts the plants' frequency response yet; with the stage-5 HPPC frequency response this becomes the first coordination case (double response, competing integrators).
5. **HPPC functions.** P and Q/V control at the POC, frequency response, dispatch, ramp limits, SOC management.

Afterwards: code generation of every controller (needs the Coder products and a compiler), the IEEE/CIGRE wrapper, and PSCAD validation.

## 9. Decided after review (2026-11-16)

- **Plant sizes:** 150 MW wind (30 × 5 MW turbines since stage 4b, 10 kV collector), 50 MW PV (10 × 5 MW stations), 30 MW / 60 MWh BESS, 180 MW POC export limit.
- **POC voltage:** 132 kV.
- **Grid code:** Energinet TF 3.3.1 as the reference for HPPC functions (FSM, LFSM-O/U, Q / U control), but strict compliance is not a goal for now.
- **Wind model:** IEC 61400-27-1 **Type 4A** (no drive-train model) for now.
- **Per-instance parameters:** not planned, but kept possible. Every asset instance k already gets its own `p.<type>.ap(k)`, including its controller's `par`, so different turbine types in one plant only need different values in `ap(k)`; the model does not change.

## 10. Open points
- **Stage 5, HPPC functions** (section 8): P and Q/V control at the POC, frequency response, dispatch, ramp limits, SOC management. The HPPC is still open loop, so nothing at the POC yet counteracts the plants' own loops; that coordination is the research question.
- **WPPC:** uses the WECC REPC_A structure; the IEC 61400-27-1 WPP control model is a later refinement.
- **Checks against the standard texts:** IEC Type 4A block details (filter forms, dip logic with hysteresis / post-fault hold, `Kqv` reference, current-limit options, `uqdip` / `Mqfrt` modes) and the WECC simplifications listed in stage 4 (VDL1/VDL2, state freeze and `Thld` during a dip, Pflag, high-voltage reactive current management, `Tfltr` in REGC_A, rate limits on the state). All parameter values are typical, not from a specific turbine or inverter.
- **Spike at fault clearing:** the network with one-step-delayed current sources gives a single-step spike when a dip clears (Q_poc ≈ 125 Mvar for one step). A smaller step or a filter on the network voltage would remove it.
- **Steady-state initialisation** at start-up (controllers currently start from zero measurements).
- **Tap changer and reactive compensation** (STATCOM, shunt reactors): not modelled yet (section 5).
- **Simulation speed:** largely solved with Fast Restart (section 11). Still open: the first compile per session (about 20–60 s) and the Simscape network's own simulation time; model reference accelerator mode (needs the C compiler) would help both.
- **Code generation and PSCAD** (after stage 5): needs the Coder products and a C compiler, which are not installed yet.

## 11. Simulation and run time  (2026-11-17)

Measured on the laptop (16 GB RAM), nominal 60 s case.

**Where the time goes.** Without Fast Restart most of a run is compiling, not simulating. `sldiagnostics` on the compile: 56 s in total, of which 23.5 s "finding and refreshing Model blocks" (loading the referenced controller and asset models; 0.6 s when they are already loaded) and 25.5 s "compiling the referenced models in normal mode". The simulation itself takes about 3–16 s with the nodal network (fewer units and more free memory are faster) and about 40 s with the Simscape network.

**Settings of the top model.**
- Solver ode1 (one evaluation per step) instead of ode4, single-tasking. The asset models keep their continuous current lags (`Tg`) and SOC integrator; making them discrete gave no speed gain and moved P during a dip by 2–3 MW, so it was not kept.
- Communication delays: discrete delays of whole steps instead of continuous transport delays.
- Logging every 10 ms (`p.sim.logDecimation`).
Effect: about 15 % faster per run; steady state unchanged, dip values within 0.7 MW.

**Fast Restart.** `runCase(..., FastRestart=true)` (the `fastRestart` setting in `main.m`, on by default) keeps `HPP.slx` compiled in the MATLAB session; `runSweep(..., FastRestart=true)` compiles once per case set. Almost all parameters change between runs without recompiling:
- controller parameters (`par` model arguments), set-points, events (dip, frequency step);
- nodal network: the numbers (impedance matrix, grid impedance, branch impedances and charging, E) enter the solver through tunable Constant blocks; only the node structure is fixed;
- Simscape network: R, L, C and transformer data are run-time block parameters.
Fixed at compile time: the layout (units per plant), the time step, the communication delays and the network model. `runCase` remembers these for the compiled model and recompiles by itself when one changes; `runSweep` with Fast Restart refuses case sets that change them. Tried and dropped: the whole network data as one tunable parameter structure and variable-length delays; each made every run 0.5–0.75 s per simulated second slower.

Measured: 1 × 1 layout per plant, nodal network: first run including compile about 18 s, every further run about 3 s; an SCR change without recompiling gives the same result as a full run (V_poc 0.9653 at SCR 3); `grid_strength` (3 cases) 25 s in total. Default layout (2 / 2 / 1): first run about 50 s, further runs about 16 s. Simscape network: about 40 s per further run.
