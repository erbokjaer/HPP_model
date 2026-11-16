# HPP_model: architecture of the hierarchical plant model

Status: **stages 1–2 built** (2026-11-16): structure, N instances, communication delays, parameter structure, and the phasor network; controller logic and generator models are placeholders. Stages 3–5 not started.
Values marked *typical* are starting points to be replaced by project data.

## 1. Decisions

| Topic | Decision |
|---|---|
| Simulation type in Simulink | **RMS (positive-sequence phasor)**. EMT validation is done in PSCAD. |
| Which controllers become DLLs | **All controllers**: HPP controller, the three plant controllers and the three asset controllers. They are our own models, not vendor DLLs. |
| Number of assets | **N per asset type**, set in the parameters. Default **N = 2** for wind, PV and BESS. |
| Asset model standards | Wind: **IEC 61400-27-1 Ed. 2, Type 4** (full converter). PV and BESS: **WECC second-generation generic models** (REGC / REEC / REPC). |
| Collector system and grid | Typical values (section 5). Own phasor network model in plain Simulink: no Simscape / Specialized Power Systems dependency. |
| Communication delays | Included from the start, typical values (section 6). |

## 2. Hierarchy

```
 TSO / market:  P_ref, Q_ref / V_ref, frequency response mode
        │
 HPPC  (HPP controller)                                   Ts = 100 ms
        │  P/Q set-points per plant         ▲ plant P, Q, availability, SOC
   ┌────┴──────────────┬───────────────────────┐
 WPPC               SPPC                     BESSC (plant controller / EMS)   Ts = 20 ms
 (IEC 61400-27-1     (WECC REPC_A)            (WECC REPC_A + SOC management)
  WPP control)
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
| `models/network/Network.slx` | Phasor network: transformers, cables, busbars, grid | – |
| `models/lib/Delay.slx` (or library block) | Communication delay: transport delay + sample-and-hold | – |

**N assets (decided in stage 1).** Each asset type is one parameterised file, instantiated N times by the build script `utilities/buildHPP.m`, which generates `HPP.slx` with N Model blocks per type and wires them. Instance k gets the model argument `ap = p.<type>.ap(k)` (asset parameters including its controller's `par`). `HPP.slx` stores the N it was built for, and `runCase` / `runSweep` stop if the parameters differ. A For Each subsystem was not used: the build script is simpler to debug, and every instance stays a normal Model block, which code generation and PSCAD handle the same way.

The plant controllers use fixed-width vector ports (width N, from their model workspace), so a DLL is built for one N. Changing N means rerunning `buildHPP(All=true)`, which is acceptable.

The network model (`models/network/`) and a library delay block are not built yet: in stage 1 the plant outputs are summed losslessly, and delays are plain Transport Delay blocks in `HPP.slx`.

**Aggregation.** One asset instance can stand for several identical physical units (`nAgg`). With N = 2 and the current plant sizes, each instance is half a plant: one feeder, so plant totals stay unchanged. With `nAgg = 1` an instance is one physical turbine or inverter.

## 4. Interfaces

Every controller keeps the pattern already used for `HPP_controller`:
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

### 4.3 Asset controllers  (base: asset rating)

| In | Unit | Out | Unit |
|---|---|---|---|
| `P_set`, `Q_set` from plant controller | pu | `ip_cmd`, `iq_cmd` to generator model | pu |
| `V_term` (magnitude), `P_meas`, `Q_meas` | pu | `P_avail` | pu |
| WT only: `P_wind_avail` (from wind input), `omega_gen` (4B) | pu | | |
| BESS only: `SOC` from battery model | – | | |

## 5. Network and grid  (typical values)

Single-line, per asset instance k:
`asset LV terminal ── unit transformer ── feeder cable ── 33 kV busbar ── main transformer ── POC 132 kV ── grid`

| Item | Typical value |
|---|---|
| Asset terminal voltage | WT 0.69 kV, PV and BESS inverters 0.69 kV (aggregated per feeder) |
| Unit / feeder transformer | 0.69 / 33 kV, rating = asset rating, u_k = 8 %, X/R = 8 |
| Collector voltage | **33 kV** |
| Feeder cable | 33 kV XLPE: R = 0.10 Ω/km, X = 0.12 Ω/km, C = 0.25 µF/km; length 5 km (wind), 2 km (PV), 0.5 km (BESS); parallel cables per instance: 2 (wind, 75 MW per instance), 1 (PV, BESS), since a 33 kV feeder carries about 30–40 MW |
| Main transformer | 33 / 132 kV, 250 MVA, u_k = 12 %, X/R = 40 |
| POC | 132 kV, export limit `p.grid.Pmax_poc` = 180 MW |
| Grid | Thevenin source, **SCR = 10** (strong), X/R = 10. Weak-grid case set: SCR = 3 |
| Frequency | 50 Hz; grid frequency as an input signal (events and ramps from case sets) |

Network solution (RMS, built in stage 2): per unit on the system base 180 MVA. Nodes: asset terminals, feeder starts (MV side of the unit transformers), 33 kV busbar, POC; 2N_total + 2 nodes (14 for N = 2). Transformers are series impedances, cables π-equivalents, the grid a Norton equivalent at the POC. `networkSolverData` builds Y and stores Z = Y⁻¹ once; each step the masked `Network` subsystem computes `V = Z * (I_inj + I_grid)` in a MATLAB Function block. It also returns each plant's P and Q into the busbar (sum over its feeders, including the busbar-end cable charging), the busbar voltage, and P, Q, V at the POC. Transformer taps and reactive compensation (STATCOM, shunt reactors) can be added later as extra nodes.

Generator interface (placeholder until stages 3–4): current source aligned with the terminal voltage, I = (i_p − j·i_q)·V/|V|, so P = |V|·i_p and Q = |V|·i_q. The voltage used is that of the previous step (as a PLL would), which also avoids an algebraic loop through the network.

Results at 150 MW injected, SCR 10, Q = 0 at all assets: P_poc 146.4 MW (losses 2.4 %), Q_poc −21.3 Mvar (cables and transformers), V_poc 0.993 pu, V_bus 0.987 pu, asset terminals 0.985–1.007 pu. With SCR 3, V_poc drops to about 0.94 pu.

## 6. Communication delays and rates  (typical values)

| Path | Delay | Controller rate |
|---|---|---|
| TSO → HPPC set-points | 200 ms | |
| POC measurement → HPPC | 20 ms | HPPC: 100 ms |
| HPPC → plant controllers | 50 ms | plant controllers: 20 ms |
| Plant measurement → plant controller | 20 ms | |
| Plant controller → asset controllers | 20 ms | asset controllers: 1 ms (= sim step) |
| Asset measurements → asset controller | 0 (local) | |

Delays are parameters (`p.comm.*`) so case sets can sweep them. Implemented as a transport delay; the receiving controller samples it at its own rate. Every measurement path into a controller also has a one-step memory (1 ms): without it, the loops through the controllers' direct feedthrough are algebraic loops when a delay is 0. The TSO delays start at the initial set-point, so the HPPC does not see a zero reference at start-up.

## 7. Parameters

```
p.sim            solver step 1 ms, stop time
p.grid           POC, base, grid SCR, X/R, frequency
p.net            transformers, cables, lengths per feeder
p.comm           communication delays
p.wt  / p.pv / p.bess   N, nAgg, rating, asset-model parameters (IEC / WECC tables)
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
3. **Wind asset.** IEC 61400-27-1 Type 4A generator system and control; WPPC from the IEC WPP control model. Check against the standard's test cases where possible.
4. **PV and BESS assets.** WECC REGC_A, REEC_B, REEC_C with SOC; SPPC and BESSC from REPC_A.
5. **HPPC functions.** P and Q/V control at the POC, frequency response, dispatch, ramp limits, SOC management.

Afterwards: code generation of every controller (needs the Coder products and a compiler), the IEEE/CIGRE wrapper, and PSCAD validation.

## 9. Decided after review (2026-11-16)

- **Plant sizes:** 150 MW wind (10 × 15 MW; as 2 instances × 5 turbines), 50 MW PV, 30 MW / 60 MWh BESS, 180 MW POC export limit.
- **POC voltage:** 132 kV.
- **Grid code:** Energinet TF 3.3.1 as the reference for HPPC functions (FSM, LFSM-O/U, Q / U control), but strict compliance is not a goal for now.
- **Wind model:** IEC 61400-27-1 **Type 4A** (no drive-train model) for now.
- **Per-instance parameters:** not planned, but kept possible. Every asset instance k already gets its own `p.<type>.ap(k)`, including its controller's `par`, so different turbine types in one plant only need different values in `ap(k)`; the model does not change.

## 10. Open points
- Steady-state initialisation at start-up (controllers currently start from zero measurements).
- Simulation speed: model reference accelerator mode (needs the C compiler), fewer delay blocks, or a larger asset step.
