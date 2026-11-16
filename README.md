# HPP_model

Simulink model of a hybrid power plant (wind, PV and battery behind one point of connection) with a separate plant controller. The controller is built so it can later be code-generated and wrapped as an IEEE/CIGRE DLL for PSCAD.

MATLAB R2026a with Simulink. Parallel Computing Toolbox is optional.

## Quick start

```matlab
openProject("HPP_model.prj")   % sets the path and loads the default p
runCase()                      % run the nominal case
listCaseSets()                 % show the available case sets
runSweep("grid_Pref_Pbess")    % run every case of a set
```

Or open `main.m`, pick a case set and cases in its settings block, and press Run: it runs (or reloads) the selected cases, plots them and prints a summary table.

Always open the project first. It puts `params/`, `models/`, `scripts/` and `utilities/` on the path and sends Simulink cache files to `work/`. No script uses `cd` or `addpath`.

## Folder layout

| Folder | Contents |
|---|---|
| `params/` | What the plant is. `HPP_parameters.m` builds one struct `p` from one function per component (simulation, wind, PV, battery, grid, controller). |
| `models/` | `HPP.slx`, the top model (fixed-step ode4, 1 ms), and `HPP_controller.slx`, the referenced discrete controller (0.1 s). |
| `scripts/` | How to run: `runCase`, `runSweep`, `defineCases`, `applyCase`, `caseList`, `caseGrid`, `caseOAT`, `listCaseSets`. Results: `plotResults`, `summarizeResults`, `loadResults`, `caseLabels`, `dtuColors`. |
| `scripts/+caseSets/` | Which variations to run: one file per case set. |
| `utilities/` | `projectStartup.m` (runs when the project opens) and the one-off `setupSkeleton.m`. |
| `hpc/` | LSF job array: `submit.sh` and `job.lsf`. |
| `results/` | Simulation output, one subfolder per case set. Not version-controlled. |
| `work/` | Simulink cache and generated code. Not version-controlled. |

## Parameters

Everything the models use lives in the struct `p`. Blocks reference fields such as `p.wt.Pn`; there are no other workspace variables. To change a default, edit the component function in `params/`. For interactive work, reload afterwards with `p = HPP_parameters();`.

## Controller interface (per unit)

`HPP_controller.slx` is the future code-generation / IEEE/CIGRE DLL target. It works entirely in per unit on the base `p.ctrl.Pbase` (= `p.grid.Pbase`, 180 MW), all ports are scalar `double`, and it runs at the fixed sample time `p.ctrl.Ts` (0.1 s).

| Port / parameter | Unit | Notes |
|---|---|---|
| in `P_ref` | pu | plant power reference |
| in `P_wt_avail`, `P_pv_avail` | pu | available wind / PV power |
| in `SOC` | – | battery state of charge, 0–1 |
| out `P_wt_set`, `P_pv_set`, `P_bess_set` | pu | set-points; + = export / discharge |
| param `par.Pbess_max` | pu | battery set-point limit |
| param `par.Ppoc_max` | pu | plant export limit (not used by the placeholder logic yet) |

The tunable parameters are the struct `p.ctrl.par`, passed to the controller as the model argument `par` (Model block → Instance parameters). In generated code it becomes the per-instance parameter struct, and later the DLL's parameter list. Everything the controller needs goes into `p.ctrl.par`, in pu; the controller reads nothing else. `Ts` and `Pbase` are fixed settings.

`HPP.slx` converts around the controller: gains `1/p.ctrl.Pbase` on the power inputs and `p.ctrl.Pbase` on the outputs, so the plant, the logged signals and the plots stay in W (MW). In PSCAD the same conversion belongs on the PSCAD side of the DLL. The controller's model workspace takes its standalone defaults (`Ts`, `par`) from `HPP_parameters()`, so they cannot drift.

## Case sets

A case set is a function in `scripts/+caseSets/` that returns a table. Each column is a field path in `p` (for example `"grid.Pref"`), each row is one case, and `NaN` means "keep the default". Rows may have names.

| Set | Cases | Built with |
|---|---|---|
| `nominal` | 1: the defaults. **Project default.** | row `nominal` of `handpicked` |
| `handpicked` | `nominal`, `lowRef`, `highRef`, `noBess` | `caseList` (one line per case) |
| `grid_Pref_Pbess` | 9: every combination of 3 Pref × 3 BESS ratings | `caseGrid` |
| `oat_Pref` | 8: Pref and BESS rating varied one at a time | `caseOAT` |

To add a set, copy one of these files, rename it and edit it. Nothing else needs to change. Three helpers build the table:

```matlab
% caseList: named cases, one line each, listing only what changes
cases = caseList({
    "nominal",  {}
    "lowRef",   {"grid.Pref", 100e6}
    "small",    {"grid.Pref", 100e6, "bess.Pmax", 15e6}
});

% caseGrid: every combination
cases = caseGrid("grid.Pref", [100 150 200]*1e6, "pv.Prated", [25 50]*1e6);

% caseOAT: one parameter at a time
cases = caseOAT("grid.Pref", [50 100 200]*1e6, "bess.Pmax", [0 60]*1e6);
```

In `handpicked`, adding a case is adding one line; the other cases never need editing.

## Running

| Goal | Command |
|---|---|
| Nominal case | `runCase()` |
| One case by name or number | `runCase("handpicked", "noBess")`, `runCase("grid_Pref_Pbess", 4)` |
| Quick check, no file written | `runCase("oat_Pref", 2, Save=false)` |
| All cases of a set, serial | `runSweep("oat_Pref")` |
| All cases with a local pool | `runSweep("oat_Pref", UseParallel=true)`, then `delete(gcp('nocreate'))` |
| Debug a case in the Simulink editor | `c = defineCases("handpicked"); p = applyCase(HPP_parameters(), c("noBess", :));` then open `HPP.slx` and press Run |

`runCase` saves `results/<set>/case_NNN.mat` with `out`, `p` and `caseInfo`. `runSweep` saves `results/<set>/sweep.mat` with `out`, `cases` and `setName`. A rerun overwrites the file, so copy results you want to keep.

Read signals with `out.yout.get("SOC").Values`. The logged signals are `P_poc`, `SOC` and `P_bess_set`.

## Looking at results

```matlab
plotResults("handpicked")                    % all saved cases of a set, one line per case
plotResults("handpicked", "noBess")          % one saved case
plotResults(out, Cases=defineCases("oat_Pref"))   % straight from runSweep, nothing saved
plotResults(..., Signals=["P_poc","SOC"])    % choose signals
plotResults(..., Theme="light")              % light figure for reports
plotResults(..., NewFigure=true)             % keep the old figure, open another

T = summarizeResults("grid_Pref_Pbess")      % final, min and max per signal and case
[out, cases] = loadResults("handpicked")     % the saved runs, for your own analysis
```

Plots have one panel per signal on a shared time axis, powers in MW, and lines labelled by case name or by the values the case changes (`Pref=100 MW, Pmax=0 MW`). Line colours come from the DTU design guide (`dtuColors`: DTU blue, corporate red, green, orange, purple, navy, red; lighter DTU tints on a dark figure); the background follows the MATLAB theme. Figures are always docked, and plotting the same set again redraws its existing figure (one figure per title, by default the set name) instead of opening a new one. `loadResults` uses `results/<set>/sweep.mat` if it exists, otherwise all `case_NNN.mat` files of the set. For interactive browsing of a single run, Simulink's Simulation Data Inspector also works: `Simulink.sdi.view`.

Serial is the default for `runSweep`. Each pool worker needs about 1 GB of RAM, and on a 16 GB laptop starting a pool next to the desktop session has crashed MATLAB.

## HPC (LSF)

From the project root on the cluster:

```bash
hpc/submit.sh grid_Pref_Pbess
```

This submits one job per case, each running `runCase(set, i)`. The script counts the cases itself; pass the count as a second argument to skip that MATLAB call. Logs go to `hpc/logs/`. Adjust the queue, memory, walltime and MATLAB module in `hpc/job.lsf` to the cluster. Not yet tested on the cluster.

## Conventions

- Battery power is positive when discharging (exporting to the grid).
- Powers are in W, energy in J, state of charge and availability in per unit.
- The controller sample time `p.ctrl.Ts` must be an integer multiple of `p.sim.Ts`; `controllerParameters` checks this.

## Status and next steps

- The plant subsystem and the controller logic are placeholders.
- The controller's SOC input is not used yet; `SOCmin`, `SOCmax` and `p.ctrl.par.Ppoc_max` are defined but unused.
- Known issue: the `"noBattery"` variant of `HPP_parameters` sets `p.bess.Pmax = 0` after `p.ctrl` is built, so `p.ctrl.par.Pbess_max` stays at 30 MW (0.167 pu). Use the case `handpicked/noBess` instead.
- Next phase: code generation for `HPP_controller` and an IEEE/CIGRE DLL for PSCAD. This needs Simulink Coder, Embedded Coder and a C compiler, which are not installed yet. Rapid Accelerator also needs the compiler.

## Source control

The repository is on GitHub: <https://github.com/erbokjaer/HPP_model>. The MATLAB project detects git, shows each file's status and can compare `.slx` versions block by block (Project → Source Control).

- `.gitignore` keeps `work/`, `results/`, `hpc/logs/` and `.mcp.json` (local MATLAB MCP server settings) out of git.
- Close a model before switching branches or pulling.
- Commit model changes together with the parameter changes they depend on.
- On the cluster: `git clone https://github.com/erbokjaer/HPP_model.git`, later `git pull`.
