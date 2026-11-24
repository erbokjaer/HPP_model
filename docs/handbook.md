# HPP_model handbook

An extended guide to the HPP_model project: how the code is set up and why, and how the power system is modelled and why. It is written so that you can understand every part of the project as if you had written it yourself, and recreate it from scratch.

- **Part A, Code setup**: architecture, MATLAB project, parameters, model generation, case system.
- **Part B, Power system modelling**: plant and modelling approach, network and grid, asset models, plant controllers, HPPC, communication delays, grid code and verification.

Every decision is described in the same pattern: **What** (the mechanism), **Why** (the reasoning), **Alternatives / tried and dropped**, and **Consequences** (what it means when you change or extend it). Each chapter ends with a *Recreating it* checklist. Statements marked *(inferred)* are reasoned from the code but not recorded anywhere; everything else is checked against the code, `docs/design.md` or the commit history. Where the docs and the code disagree, the code is described and the disagreement is noted; all of them are collected in Appendix X.

Status: 19 Nov 2026, commit `6c925ce` plus the uncommitted changes of that day (case sets with one stop time per set, `defineCases` warning, `plotAllResults` docking, `main.m` "all" option). Chapters A5 (running simulations), A6 (results and figures) and A7 (performance history and extending the project) are not written yet; until then, see `README.md` (Running, Looking at results) and `docs/design.md` sections 11–13.

`README.md` is the short user guide and `docs/design.md` the decision log; this handbook explains both in depth.

## Contents

- [A1. Overall architecture and philosophy](#a1-overall-architecture-and-philosophy)
- [A2. Parameters](#a2-parameters)
- [A3. Model generation and model files](#a3-model-generation-and-model-files)
- [A4. Case system](#a4-case-system)
- [B1. The plant and the modelling approach](#b1-the-plant-and-the-modelling-approach)
- [B2. Network and grid](#b2-network-and-grid)
- [B3. Asset models](#b3-asset-models)
- [B4. Plant controllers (WPPC, SPPC, BESSC: WECC REPC_A)](#b4-plant-controllers-wppc-sppc-bessc-wecc-repc_a)
- [B5. HPPC: central control at the POC](#b5-hppc-central-control-at-the-poc)
- [B6. Communication delays, controller rates, measurement memories and algebraic loops](#b6-communication-delays-controller-rates-measurement-memories-and-algebraic-loops)
- [B7. Grid code (Energinet DK1): requirements, verification and open points](#b7-grid-code-energinet-dk1-requirements-verification-and-open-points)
- [Appendix X. Discrepancies found while writing this handbook](#appendix-x-discrepancies-found-while-writing-this-handbook)

# Part A. Code setup

## A1. Overall architecture and philosophy

This chapter covers how the project is put together as software: which concerns are kept apart, how one struct carries every number from the parameter files into the models, how a run flows from a parameter file to a saved figure, and the infrastructure around it (MATLAB project, search path, startup, cache folder, git). Later chapters go into each part in depth. This one gives the map and the reasons behind it.

### A1.1 The four separated concerns

**What.** The repository splits the work into four questions, each answered in its own place:

| Concern | Question it answers | Where |
|---|---|---|
| Parameters | *What is the plant?* Ratings, layout, network data, controller settings, events. | `params/` |
| Models | *How does the plant behave?* Block diagrams of controllers, assets, network. | `models/` (`HPP.slx`, `ctrl/`, `assets/`), generated or checked by `utilities/` |
| Case sets | *Which variations do we want to look at?* | `scripts/+caseSets/` |
| Running and results | *How do we run, store and look at them?* | `scripts/` (run, results, plots), `buildfile.m`, `main.m`, `hpc/` |

`utilities/` holds the tools that create and check models (`buildHPP.m`, `modelInterfaces.m`) and the project startup file. `docs/` holds `design.md`. `experiments/` holds standalone test models that the main model does not use. `results/` and `work/` are output folders and are not under version control.

**Why.** Each concern changes for a different reason and at a different rate:
- Parameter values change most often (a gain, an SCR, a set-point), and you should never have to open a Simulink model to change one.
- Models change when the behaviour changes (a new loop, a new block). They should not contain numbers that belong to the plant.
- Case sets change whenever you want a new study. Adding one must not touch the models or the parameter defaults. README: "To add a set, copy one of these files, rename it and edit it. Nothing else needs to change."
- Running and plotting change when the workflow changes (Rapid Accelerator, Fast Restart, HPC, figures). They must work for any case set.

Keeping them apart means a change in one place cannot silently break another, and each part can be reasoned about alone. It also makes the same parameter set usable by Simulink now and by the PSCAD DLL wrapper later (design.md section 4: the W/pu conversion lives outside the controllers so that the same controller code works in both).

**Alternatives.** The project started (initial commit `22df7cb`, 2026-11-13) with the same four-way split, but with one controller model and a handful of parameter files. It was never organised as one big script or as a model with values typed into block dialogs. No alternative layout is recorded as tried and dropped. The split itself is the starting decision (inferred: the standard MATLAB-project layout for parameter studies, chosen up front).

**Consequences.**
- To change the plant, edit `params/`. To change behaviour, edit a controller or asset model (or `buildHPP.m` for `HPP.slx`). To study something new, add a file to `scripts/+caseSets/`. Never mix these steps.
- A case set can only change things that exist as fields of `p` (section A4.6). If a study needs a new knob, the knob is first added as a parameter (A2), then used in a model, then varied in a case set.

### A1.2 One struct `p`: the only variable the models read

**What.** `params/HPP_parameters.m` returns one struct `p`. Its help text states the rule: "Returns one struct, p, which is the only workspace variable the Simulink models use. Blocks reference fields, e.g. p.wt.Pn." The README repeats it: "Everything the models use lives in the struct `p`. Blocks reference fields such as `p.wt.Pn`; there are no other workspace variables."

`p` has one sub-struct per component (`p.sim`, `p.wt`, `p.pv`, `p.bess`, `p.grid`, `p.net`, `p.comm`, `p.pc`, `p.hppc`) plus derived parts (`p.ctrl`, `p.<type>.ap`, `p.net.solver`, `p.comm.steps`) and `p.meta` (variant name and creation time). Chapter A2 lists every field.

There are three ways `p` reaches the models:
1. **Top model `HPP.slx`.** Blocks in `HPP.slx` use expressions such as `p.grid.Pref`, `p.sim.Ts` or `p.ctrl.hppc.Pbase` directly. For a run, `runCase` and `runSweep` put `p` into the `Simulink.SimulationInput` object with `in.setVariable("p", p)` (`scripts/runCase.m` ~line 91, `scripts/runSweep.m` ~line 91). For interactive work, `projectStartup.m` loads a default `p` into the base workspace.
2. **Controller models (`models/ctrl/*.slx`).** They never see `p`. They see one struct `par`, a *model argument* (explained in A2.15). The Model block in `HPP.slx` sets it to `p.ctrl.<name>.par`.
3. **Asset models (`models/assets/*.slx`).** They see one struct `ap`, also a model argument. Instance k of a type gets `p.<type>.ap(k)`. Inside the asset model, the referenced asset controller gets `par = ap.ctrl`.

There are two exceptions, and both are deliberate. The *variant control variables* `HPP_network` (1 nodal, 2 Simscape) and `HPP_delays` (1 fixed, 2 variable) are set per run in the model workspace of `HPP.slx` (`in.setVariable(..., "Workspace", "HPP")`). They choose which variant of a Variant Subsystem is compiled, so they have to be plain scalar variables, not fields of a struct (inferred). They are computed from `p` (`networkCode(p.net.solverType)`, `delayMode(p, ...)`), so `p` is still the single source of truth. The model workspace of `HPP.slx` also stores `buildInfo` (the N it was built for, read by `checkModelBuild`).

**Why one struct.**
- *Reproducibility.* Each saved result (`results/<set>/case_NNN.mat`) stores `out`, `p` and `caseInfo`. Because `p` is complete, a result file says exactly what was simulated. A scattered set of workspace variables could not be saved this reliably.
- *Parallel and batch runs.* `SimulationInput.setVariable` hands over one variable per run. Workers (parsim, HPC jobs, Rapid Accelerator runs) need no shared base workspace. `projectStartup.m` says so explicitly: "Batch/HPC runs do NOT rely on this - they pass p via SimulationInput."
- *Fast Restart and Rapid Accelerator.* Both reuse a compiled model and change only the parameter values between runs. One struct passed per run is the natural carrier for this (A5 has the details).
- *Typo protection.* Case columns are field paths in `p`, and `applyCase` refuses a path that does not exist (A4.6). This only works because everything lives in one tree.
- *No hidden state.* A model never depends on whatever happens to be in the base workspace from an earlier session.

**Alternatives.** Simulink Data Dictionaries (`.sldd`) and many separate base-workspace variables are the usual alternatives. Neither is used, and no reason is recorded. Plausible reasons (inferred): a struct built by plain MATLAB functions can be diffed in git as text, built conditionally (variants, derived values), and passed per run without global state. A dictionary is a binary file and is harder to vary per case.

**Consequences.**
- A new parameter is a new field in one component function (A2). Blocks then refer to `p.<...>` (in `HPP.slx`) or `par.<...>` / `ap.<...>` (in the referenced models).
- Never create other workspace variables for a model to read. They would not be passed to parallel or compiled runs, and they would not be saved with the result.
- Any change to `p` after `HPP_parameters()` that touches a component value must be followed by `deriveParameters(p)` (`applyCase` does this). Otherwise the derived parts (`p.ctrl`, `ap`, network matrices) keep the old values (A2.13).

### A1.3 End-to-end data flow

The full path of one case, from parameter files to figure:

```
params/*Parameters.m  --HPP_parameters()-->  p (components)
      |  variant switch ("default" | "noBattery")
      v
deriveParameters(p): N, Prated, comm.steps, p.ctrl (controllerParameters),
                     p.<type>.ap(k), p.net.solver (networkSolverData)
      |
scripts/+caseSets/<set>.m  --defineCases(set)-->  table (columns = field paths, NaN = default)
      |
applyCase(p, row): set fields, check paths exist, deriveParameters again
      |
runCase / runSweep: Network option -> p.net.solverType; delayMode -> p.comm.delays + HPP_delays;
                    checkModelBuild (HPP.slx built for this N?); rapidTarget (Rapid build if needed)
      |
Simulink.SimulationInput("HPP"): setVariable("p", p), HPP_network, HPP_delays, StopTime = p.sim.Tstop
      |
HPP.slx:  p.* in blocks; Model blocks get par = p.ctrl.<c>.par, ap = p.<type>.ap(k)
      |
out (logged signals in SI units, every p.sim.logDecimation steps)
      |
results/<set>/case_NNN.mat (out, p, caseInfo)  or  results/<set>/sweep.mat (out, cases, setName)
      |
loadResults -> plotResults / summarizeResults / plotAllResults (reportFigures) -> results/figures/
```

`main.m` is a thin front end over this path. Its settings block chooses the set, the cases, `rerun`, `mode` (`"rapid"` or `"normal"`), `fastRestart`, `network`, `delays`, the signals and the plotting. It then calls `defineCases`, `runCase` (or `loadResults`), `summarizeResults` and `plotResults`. In the working copy, `setName = "all"` runs `runSweep` for every set and then `plotAllResults` (uncommitted change, see the note at the end of A1.11). `buildfile.m` runs the same steps as build tasks (`check` → `model` → `rapid` → `sweeps` → `figures`). `hpc/` runs the same `runCase(set, i)` as one LSF job per case. Chapters A5 and A6 cover these.

**Why this flow.** Every run goes through `applyCase(HPP_parameters(), row)`. Normal runs, parallel runs, Rapid Accelerator runs and HPC jobs therefore build `p` the same way, and a case is defined only by its row. There is no path where a run picks up a value that was not in its row or in the defaults.

### A1.4 The MATLAB project (`HPP_model.prj`)

**What.** The repository is a MATLAB project. `HPP_model.prj` is a one-line XML stub. The project definition lives in `resources/project/` as many small XML files (one pair `...d.xml` / `...p.xml` per entry, `Project.xml` with `MetadataType="fixedPathV2"`). This "fixed path" metadata format is the git-friendly format: every file's project membership and labels are separate small files, so merges rarely conflict. `.gitattributes` forces LF line endings for them (`/resources/project/**/*.xml text eol=lf`).

What the project defines (read from `resources/project/`):

| Setting | Value | Where it is stored |
|---|---|---|
| Project name | `HPP_model` | `root/GiiBklLg...d.xml` |
| Project path | `params`, `models`, `models/ctrl`, `models/assets`, `scripts`, `utilities` | `EEtUlUb-.../*.xml` (`Ref="..." Type="Relative"`) |
| Startup file | `utilities/projectStartup.m` (type `StartUp`) | one entry with `Name="projectStartup"` |
| Simulink cache folder | `work` | `WZRuNzqc-.../AtMjXDr...` (`SimulinkCacheFolder`) |
| Simulink code generation folder | `work` | `WZRuNzqc-.../66WgGCc...` (`SimulinkCodeGenFolder`) |
| File labels | all members labelled `Classification = Design` | `...d.xml` files with `<Label UUID="design"/>` |
| File membership | the files listed below | `...p.xml` files with `type="File"` |

There is also a leftover "Toolbox1" deployment configuration (`MyToolbox.mltbx`, `Toolbox1\release`) in `resources/project/root/`. It belongs to MATLAB's toolbox-packaging feature and nothing uses it. It was probably created by opening the packaging dialog once (inferred). It is harmless.

**Project membership.** Members are: the root files `README.md`, `main.m`, `buildfile.m`, `.gitignore`, and the folders `docs/` (`design.md`), `params/` (all 13 files), `models/` (`HPP.slx`, `ctrl/` with 7 models, `assets/` with 3 models), `scripts/` (all scripts and `+caseSets/` with all 11 sets), `utilities/` (`buildHPP.m`, `modelInterfaces.m`, `projectStartup.m`), `hpc/` (`submit.sh`, `job.lsf`) and `results/` (only `.gitkeep`). `experiments/` is **not** a project member and not on the path, although its files are in git (`experiments/simscapeGrid/ftGridTest.m` and two test models, added in `8a6d58d`). The git status at the time of writing shows a staged membership entry for `scripts/plotAllResults.m`: the file existed before but had not been added to the project.

**Why a project.**
- *Path without `addpath`.* Opening the project puts the right folders on the path and removes them when it closes. README: "Always open the project first... No script uses `cd` or `addpath`." A grep over the code confirms there is no `addpath` and no `cd(` in any `.m` file outside `experiments/`.
- *Startup actions.* `projectStartup.m` runs once when the project opens (A1.6).
- *Cache isolation.* Simulink's build artefacts go to `work/` instead of the current folder (A1.7).
- *Source control in the IDE.* The project shows each file's git status and can compare `.slx` versions block by block (README, "Source control").
- *Build tool integration.* The project's Run Build toolstrip button runs `buildfile.m` (A5).

**Alternatives.** A `startup.m` with `addpath` calls was not used. It would change the global MATLAB path for every session, not only for this project, and it would not route the Simulink cache (inferred).

### A1.5 The project path, and why `hpc/` is not on it

**What.** Exactly six folders are on the path: `params`, `models`, `models/ctrl`, `models/assets`, `scripts`, `utilities`. The project root itself is not a path entry. When a project opens, MATLAB makes the project root the current folder by default, so `main.m` and `buildfile.m` are found there (inferred from MATLAB's default project start-up folder; `buildtool` looks for `buildfile.m` in the current folder).

Points to note:
- `models/ctrl` and `models/assets` are listed separately because MATLAB does not add subfolders by itself. `HPP.slx` refers to the controller and asset models by name (Model blocks with `ModelName`), so all model folders must be on the path.
- `scripts/+caseSets/` is a *package folder* (its name starts with `+`). Package folders must **not** be on the path themselves. Their *parent* (`scripts`) is on the path, and the functions inside are called with the package prefix: `caseSets.handpicked()`, `feval("caseSets." + setName)`. `listCaseSets` finds them with `what("caseSets")`.
- `hpc/` is not on the path. It holds only a bash script and an LSF job file, nothing MATLAB calls. The jobs run `matlab -batch "openProject('HPP_model.prj'); runCase(...)"`, so they use the project path like any other session. No reason is recorded. The plausible reason (inferred) is that only folders with MATLAB code the models or scripts call belong on the path. The same reasoning keeps `docs/`, `results/` and `work/` off the path.
- `experiments/` is off the path and outside the project. Its test models (`ftGrid_N5_RL.slx`, `ftGrid_N5_T.slx`) have names that cannot clash with the main models, and the experiments are meant to be opened and run by hand. Keeping them off the path keeps `which`, `listCaseSets` and the model checks clean (inferred).

**Consequences.**
- A new MATLAB folder (for example a new `analysis/`) must be added to the project path in the project UI. Otherwise its functions are not found in batch or HPC runs.
- Never put a `+package` folder itself on the path.
- Function names must be unique across the six path folders.

### A1.6 Startup: `utilities/projectStartup.m`

**What.** The startup file is a script (not a function) and runs in the base workspace when the project opens:

```matlab
p = HPP_parameters();
fprintf("HPP_model: loaded default parameters into 'p' (variant: %s)\n", p.meta.variant);
if usejava("desktop")
    close(matlab.desktop.editor.getAll);
    root = currentProject().RootFolder;
    edit(fullfile(root, "README.md"));
    edit(fullfile(root, "main.m"));
end
```

1. It loads the default `p` into the base workspace. Then `HPP.slx` and the referenced models can be opened, updated and simulated interactively straight away, because `HPP.slx` blocks use `p.*`.
2. In an interactive (desktop) session, it closes all open Editor tabs and opens `README.md` and `main.m`. `close()` asks before discarding unsaved edits, so nothing is lost. In batch or HPC sessions (`usejava("desktop")` is false) it does nothing beyond loading `p`.

**Why.** The default `p` must exist before anyone opens `HPP.slx` by hand. The README and `main.m` are the entry points for a person (git log `0e70343` era note: "projectStartup opens README.md and main.m in the Editor in desktop sessions"). Batch runs must not depend on startup side effects, which is why runs pass `p` explicitly.

**Consequences.** The base-workspace `p` is only a convenience. After editing a parameter file, reload with `p = HPP_parameters();` (README). Scripts never read the base `p`. One exception, for an internal reason: `rapidTarget` temporarily assigns `p` in the base workspace for the Rapid Accelerator build and restores the old value afterwards (`scripts/rapidTarget.m` ~line 120). That is A5's topic.

### A1.7 Cache and code generation folder `work/`

**What.** Both the Simulink cache folder and the code generation folder of the project are `work/`. Everything Simulink generates goes there:
- `.slxc` cache files of every model (`HPP.slxc`, `HPPC.slxc`, ...),
- the model reference simulation targets (`*_msf.mexw64`),
- the `slprj/` folder with accelerator and Rapid Accelerator builds, including `work/slprj/raccel/HPP/` with the Rapid Accelerator program and its saved fingerprint `hpp_rapid_fingerprint.mat` (design.md section 13).

`work/` is in `.gitignore`. The folder is created by Simulink as needed. A fresh clone has none.

**Why.** Generated files must not pollute the source folders or git, and they are machine-specific (compiled MEX files, compiler output). Putting them in one folder also means one place to delete if the cache ever gets stale. A side effect that matters: the Rapid Accelerator target and its fingerprint survive between MATLAB sessions in `work/`, so a new session reuses an unchanged target without rebuilding (A5).

**Consequences.**
- Deleting `work/` is always safe. It only costs one recompile (model reference targets about 191 s serially, Rapid build 20–50 s, design.md section 13).
- `.gitignore` also lists `slprj/`, `*.slxc`, `*_grt_rtw/`, `*_ert_rtw/`, `*.mexw64`. These should never appear outside `work/` when the project is open. The entries guard against models opened without the project (inferred).

### A1.8 Root-relative paths: `currentProject().RootFolder`

**What.** Every script that reads or writes a file builds its path from the project root:

| File | Use |
|---|---|
| `scripts/runCase.m` ~125 | `results/<set>/case_NNN.mat` |
| `scripts/runSweep.m` ~163 | `results/<set>/sweep.mat` |
| `scripts/loadResults.m` ~28 | reads `results/<set>/` |
| `scripts/plotAllResults.m` ~42 | `results/figures/...` |
| `scripts/rapidTarget.m` ~147 | fingerprint file / model files |
| `scripts/checkModels.m` ~37 | default model root `models/` |
| `utilities/build/buildHPP.m` ~72 | model folders |
| `utilities/projectStartup.m`, `main.m` | README, `results/` |

**Why.** The result does not depend on the current folder. A user may `cd` anywhere, a parsim worker starts in a temporary folder, and an HPC job starts in its submit folder. All still write to the same `results/<set>/`. Together with "no `cd`, no `addpath`" this makes every script location-independent.

**Consequences.** Scripts need an open project (`currentProject()` errors otherwise). This is why `hpc/job.lsf` and `hpc/submit.sh` start with `openProject('HPP_model.prj')`. New scripts that touch files must use `currentProject().RootFolder` too.

### A1.9 Git and `.gitignore`

**What.** The repository is on GitHub (`https://github.com/erbokjaer/HPP_model`, README). `.gitignore`:

```
# Simulink build / cache artefacts (project routes them into work/)
work/  slprj/  *.slxc  *_grt_rtw/  *_ert_rtw/  *.mexw64  *.autosave  *.asv
# Local tool configuration (machine-specific paths)
.mcp.json
# Simulation output
results/*   !results/.gitkeep   hpc/logs/
# MATLAB build tool cache (buildtool)
.buildtool/
```

- `results/*` with `!results/.gitkeep`: the folder exists in a fresh clone (so the project membership entry is valid), but no result is committed. Results are reproducible from code plus parameters, and `.mat` files are large binaries.
- `.mcp.json` holds local MATLAB MCP server settings with machine-specific paths (commit `08e87e6`: "holds machine-specific paths (local executable and MATLAB root) and is not needed to run the model").
- `.buildtool/` is the build tool's incremental cache (A5).
- `*.autosave`, `*.asv`: editor backups.

`.gitattributes` (from MATLAB's project template) marks `.slx`, `.mat`, `.fig`, `.mex*` and similar as binary. `.slx` and some other Simulink formats get `merge=mlAutoMerge`, MATLAB's model merge driver. Project XML gets LF endings.

**Git habits (README "Source control").**
- Close a model before switching branches or pulling. A loaded model keeps the old version in memory and can overwrite the new file when saved (inferred reason).
- Commit model changes together with the parameter changes they depend on. A controller that uses a new `par.X` breaks unless `HPP_parameters()` defines `X` (`checkModels` checks this, A2.16).
- `HPP.slx` is generated (A3), but it is committed so that a fresh clone runs without calling `buildHPP` first (inferred: it is a project member and tracked in git).

**Commit history as documentation.** The 33 commits have detailed messages recording decisions and measurements, for example `fb180ee` (what is tunable, what was tried and dropped) and `ac18a90` (fixed and variable delays). Commits made with the assistant end with a `Co-Authored-By` line.

### A1.10 Conventions

**From README "Conventions":**

| Convention | Meaning |
|---|---|
| Battery power positive when discharging | Generator convention for every source: + P = export / discharge. Also + Q = export (capacitive), design.md section 4. |
| Powers in W, energy in J, SOC and availability in pu | Parameter files and logged signals use SI units: `p.bess.E = 60e6*3600` J, `p.grid.Pref = 150e6` W. Only the controller interfaces are per unit. |
| Controller sample times are integer multiples of `p.sim.Ts` | Checked by `controllerParameters` (assert). This mirrors PSCAD, where a controller DLL runs at a multiple of the EMT step (`simulationParameters.m` comment). |

**From design.md section 1 "Decisions":**

| Topic | Decision | Reason (where recorded) |
|---|---|---|
| Simulation type | RMS (positive-sequence phasor) in Simulink; EMT validation in PSCAD | Speed for parameter studies; PSCAD gives EMT detail (design.md section 1, section 2) |
| Which controllers become DLLs | All seven: HPPC, three plant controllers, three asset controllers | They are own models, not vendor DLLs; all must run in PSCAD later |
| Number of assets | Per plant `nFeeders` × `nPerFeeder` units, each its own asset instance at its own node | Shows the plant controller dispatching N assets; aggregation (`nAgg`, `cable.nPar`) keeps the count small |
| Asset model standards | Wind IEC 61400-27-1 Ed. 2 Type 4; PV and BESS WECC second-generation (REGC/REEC/REPC) | Parameters can be checked against public tables (design.md section 7: "Each standard model's parameter list is kept with the standard's own names") |
| Collector system and grid | Typical values; two network models (nodal, Simscape F&T) with the same results, chosen per run; Specialized Power Systems not used | Nodal is fast; Simscape is component-wise and checks the nodal model |
| Communication delays | Included from the start, typical values | The coordination problem depends on them |

**Further conventions that the code follows** (collected from design.md sections 3, 4, 7 and the code):
- *Each controller is per unit on its own base*: HPPC on the POC base (`p.grid.Pbase` = 180 MW), plant controllers on the plant rating, asset controllers on the unit rating (A2.14).
- *Controller ports are scalar or fixed-width vectors of `double`*. Tunable values sit in one struct `par` (model argument). `Ts` and the base are fixed.
- *W ↔ pu conversion outside the controllers*, in `HPP.slx` (in PSCAD later on the PSCAD side).
- *Parameter names follow the standards* (IEC: `kpq`, `udip`, `MqG`; WECC: `Kqp`, `Trv`, `Ddn`).
- *Defaults are typical values*, not project data. design.md: "Values marked *typical* are starting points to be replaced by project data." The parameter files say so in their comments.
- *Generated vs. source models*: `HPP.slx` is generated by `buildHPP` and never edited by hand. Controller and asset models are the source and are edited in Simulink (A3).
- *Block expressions using `par`/`ap` use only + − × ÷, parentheses, numbers and field access* (A2.16).
- *Closed loops at every level* (design.md stage 4, decided 2026-11-16): assets, plant controllers and HPPC each close their own loops, as vendor plant controllers do. The HPPC coordinating them is the research problem.
- *Each stage ends with a regression check* against the previous stage (design.md section 8).

### A1.11 Notes: docs against code

- `params/HPP_parameters.m` help text still shows the example `p = HPP_parameters(); p.ctrl.Kp = 0.8;`. There is no `p.ctrl.Kp` any more. Also, `p.ctrl` is rebuilt by `deriveParameters`, so such an override is lost at the next `applyCase` (A2.13, A4.6). The example dates from the initial commit.
- `params/simulationParameters.m` says "The controller sample time (p.ctrl.Ts)". Today each controller has its own `p.ctrl.<name>.Ts`.
- README "Parameters" says "there are no other workspace variables". The code also uses the variant control variables `HPP_network` and `HPP_delays` in the model workspace of `HPP.slx` (A1.2). They are derived from `p` per run.
- `main.m` in the working copy (uncommitted) has `if setName == "all"don` on line 26. This looks like a stray typo and would be a parse error when `main.m` runs. The committed version has no such line.

### Recreating it

1. Create the folders `params/`, `models/ctrl/`, `models/assets/`, `scripts/+caseSets/`, `utilities/`, `docs/`, `hpc/`, `results/` (with `.gitkeep`) and `experiments/`.
2. Create a MATLAB project in the root (`HPP_model.prj`) and pick the "fixed path" (many small XML files) metadata format, which is the default for new projects.
3. Add all folders except `experiments/` and `work/` to the project. Add `params`, `models`, `models/ctrl`, `models/assets`, `scripts`, `utilities` to the project path (not `hpc`, not `+caseSets`).
4. In project settings, set the Simulink cache folder and the code generation folder to `work`.
5. Write `utilities/projectStartup.m` (load `p`; in desktop sessions open README and `main.m`) and register it as a startup file.
6. Write `.gitignore` (`work/`, `slprj/`, `*.slxc`, `*_grt_rtw/`, `*_ert_rtw/`, `*.mexw64`, `*.autosave`, `*.asv`, `.mcp.json`, `results/*` with `!results/.gitkeep`, `hpc/logs/`, `.buildtool/`) and keep MATLAB's default `.gitattributes`.
7. Write `params/` so that `HPP_parameters()` returns one struct `p` (A2), and let models read only `p` (top model) or `par`/`ap` (referenced models).
8. In every script that touches files, build paths from `currentProject().RootFolder`. Never use `cd` or `addpath`.
9. Pass `p` to every run with `Simulink.SimulationInput.setVariable("p", p)`. Never rely on the base workspace in scripts.
10. Write README (quick start, folder table, conventions) and `docs/design.md` (decisions table, interfaces, stages). Commit with detailed messages.

---

## A2. Parameters

This chapter describes `params/`: the thirteen functions that build `p`, every field with value, unit and meaning, what is derived and why, the per-unit bases, the model-argument mechanism (`par`, `ap`), the rule for block expressions, and the checks `controllerParameters` makes. The physics behind the values (network, asset models, controllers) is in Part B. This chapter is about the structure and the reasons for it.

### A2.1 Overview: how `p` is built

`params/HPP_parameters.m`:

```matlab
function p = HPP_parameters(variant)          % variant (1,1) string = "default"
    p.sim  = simulationParameters();
    p.wt   = windTurbineParameters();
    p.pv   = pvParameters();
    p.bess = batteryParameters();
    p.grid = gridParameters();
    p.net  = networkParameters();
    p.comm = commParameters();
    p.pc   = plantControlParameters();
    p.hppc = hppcParameters();
    switch variant
        case "default"    % nothing
        case "noBattery", p.bess.Pmax = 0;
        otherwise, error("HPP_parameters:unknownVariant", ...);
    end
    p = deriveParameters(p);                  % last, so derived values follow the variant
    p.meta.variant = variant;
    p.meta.created = string(datetime("now"));
end
```

**What.** One function per component, each returning a plain struct with only *set* values (numbers a person chooses). After the variant, `deriveParameters` computes everything that follows from them.

**Why one function per component.** The help text: "Each component lives in its own function so it can be reused, tested and swapped independently." A component function takes no inputs and depends on nothing else. That is why derived quantities that combine components (plant rating, controller per-unit values, network matrices) cannot live in them and are computed later.

**Why the derivation comes last (the `noBattery` bug).** In the initial commit, `p.ctrl = controllerParameters(p)` was called *before* the variant switch. `HPP_parameters("noBattery")` then set `p.bess.Pmax = 0`, but the controller limits had already been computed from 30 MW. Stage 1 (`41a0b4f`) moved all derived values into `deriveParameters`, called after the variant. README: "The `"noBattery"` variant bug is fixed: derived parameters are now computed after the variant." The same rule makes `applyCase` call `deriveParameters` again after it sets case values (A4.6).

**Variants.** Only two exist:

| Variant | Change |
|---|---|
| `"default"` | none |
| `"noBattery"` | `p.bess.Pmax = 0` (the BESS converter and its 60 MWh remain in the model; only its power limit is zero) |

Variants are not reachable from case sets. `runCase` and `runSweep` always start from `HPP_parameters()` (default). A case set that wants no battery sets `{"bess.Pmax", 0}` (case `noBess` in `handpicked`). Variants are for interactive use and for whole alternative plants (inferred: the mechanism is a placeholder from the initial commit for future plant variants).

**`p.meta`.** `variant` (string) and `created` (timestamp string). For record-keeping in saved results. `created` changes on every call, so two `p` structs are never `isequal` (inferred consequence: code that compares `p` for changes should compare specific fields, which the compile signatures in A5 do).

### A2.2 `simulationParameters` → `p.sim`

| Field | Value | Unit | Meaning |
|---|---|---|---|
| `Ts` | 1e-3 | s | Base fixed step of the top model and the asset models. Asset controllers run at this rate. All controller `Ts` must be integer multiples of it. |
| `Tstop` | 60 | s | Simulation stop time. Case sets override it (70, 90, 120 s). |
| `logDecimation` | 10 | – | Log every 10th step (10 ms) "to save memory and time". |

**Why 1 ms.** It must resolve the fastest dynamics (converter current lag `Tg` = 10–20 ms, asset Q PI ~0.1 s). It also matches the PSCAD idea of a DLL at a multiple of the EMT step. The top model uses `ode1` (one evaluation per step). design.md section 11: switching from `ode4` to `ode1`, discrete whole-step delays and decimated logging made runs about 15 % faster with steady state unchanged and dip values within 0.7 MW. **Consequence:** `sim.Ts` is compile-time (A5). Changing it requires a recompile, and `runSweep` with Fast Restart refuses a case set that changes it.

### A2.3 `windTurbineParameters` → `p.wt`

**Layout and profile** (set values):

| Field | Value | Unit | Meaning |
|---|---|---|---|
| `Pn` | 5e6 | W | Rated power per turbine |
| `nAgg` | 15 | – | Turbines represented by one modelled unit (one converter model, unit transformer scaled by 15) |
| `nFeeders` | 1 | – | Feeders in the model |
| `nPerFeeder` | 2 | – | Units along each feeder |
| `Pavail0` | 0.6 | pu | Initial available power (all wind instances) |
| `PavailStep` | 0.9 | pu | Available power after the step |
| `tStep` | 20 | s | Time of the availability step |

Derived (in `deriveParameters`): `N = nFeeders·nPerFeeder = 2`, `Prated = N·nAgg·Pn = 150 MW`, `ap` (N×1 struct array).

The default layout stands for the full 150 MW plant of 30 turbines: 1 feeder × 2 units × 15 turbines, where the modelled feeder stands for 15 identical parallel feeders (`p.net.wt.cable.nPar = 15`). A full layout would be `nFeeders = 15`, `nPerFeeder = 2`, `nAgg = 1`, `nPar = 1`. design.md section 3: a full layout (41 converter models, 85 nodes) gives practically the same steady state (P_poc 149.46 against 149.40 MW in a static check). The reason for aggregation is run time: fewer converter models and nodes.

**IEC 61400-27-1 Type 4A control** (`wt.iec.ctrl` → copied to `p.ctrl.wt.par`). All pu on the asset (unit) rating, times in s. Typical values, "NOT taken from the standard or a turbine".

| Field | Value | Meaning |
|---|---|---|
| `Tufilt` | 0.01 | Voltage measurement filter |
| `Tpfilt` | 0.01 | Active power measurement filter |
| `Tqfilt` | 0.01 | Reactive power measurement filter |
| `udip` | 0.90 | Voltage dip (FRT) threshold |
| `dpmaxp4A` | 1.0 | Max power order ramp rate, pu/s (P control 4A) |
| `Tpordp4A` | 0.01 | Power order lag |
| `uref0` | 1.0 | Reference voltage for dip current injection |
| `Kqv` | 2.0 | Additional reactive current gain during a dip, pu/pu |
| `iqH1` | 1.05 | Max reactive current during a dip |
| `imax` | 1.1 | Max total current |
| `MqG` | 1 | Q control mode: 1 closed loop, 2 open loop |
| `kpq` | 0.1 | Reactive power PI, proportional gain (pu i / pu q) |
| `kiq` | 10 | Reactive power PI, integral gain, 1/s ("about 0.1 s: fastest level of the cascade") |

**Generator system** (`wt.iec.gen` → `p.wt.ap(k).gen`):

| Field | Value | Meaning |
|---|---|---|
| `Tg` | 0.01 | Current generation time constant, s |
| `dipmax` | 1.0 | Max active current ramp rate, pu/s |
| `diqmax` | 100 | Max reactive current ramp rate, pu/s |

**Why split `ctrl` and `gen`.** `ctrl` belongs to the asset *controller* (a future DLL, model argument `par`). `gen` belongs to the generator or converter model, which is plant side and is replaced by PSCAD's own models later (design.md section 2). Keeping them apart in `p` mirrors the model split.

### A2.4 `pvParameters` → `p.pv`

| Field | Value | Unit | Meaning |
|---|---|---|---|
| `Pn` | 5e6 | W | Rated power per inverter station |
| `nAgg` | 5 | – | Stations per modelled unit |
| `nFeeders` | 2 | – | Feeders |
| `nPerFeeder` | 1 | – | Units per feeder |
| `Pavail` | 0.4 | pu | Available power (constant placeholder) |

Derived: `N = 2`, `Prated = 2·5·5 MW = 50 MW`, `ap`.

**WECC REEC_B electrical control** (`pv.wecc.ctrl` → `p.ctrl.pv.par`), pu on the unit rating:

| Field | Value | Meaning |
|---|---|---|
| `Trv` | 0.02 | Terminal voltage filter, s |
| `Vdip` | 0.90 | Dip threshold, low |
| `Vup` | 1.10 | Dip threshold, high |
| `Vref0` | 1.0 | Reference voltage for reactive current injection |
| `dbd1` | −0.05 | Voltage deadband, lower (on Vref0 − V) |
| `dbd2` | 0.05 | Voltage deadband, upper |
| `Kqv` | 2.0 | Reactive current injection gain, pu/pu |
| `Iqh1` | 1.05 | Max injected reactive current |
| `Iql1` | −1.05 | Min injected reactive current |
| `Qflag` | 1 | 1 closed-loop Q (PI on Q_set − Q_gen), 0 open loop Iq = Q/V |
| `Kqp` | 0.1 | Reactive power PI, proportional gain |
| `Kqi` | 10 | Reactive power PI, integral gain, 1/s (~0.1 s) |
| `Tiq` | 0.02 | Reactive current lag, s |
| `dPmax` | 99 | P reference ramp up, pu/s (effectively unlimited) |
| `dPmin` | −99 | P reference ramp down, pu/s |
| `Tpord` | 0.02 | Power order lag, s |
| `Imax` | 1.1 | Max converter current |
| `PQflag` | 0 | Current priority: 0 Q priority, 1 P priority |

**WECC REGC_A converter** (`pv.wecc.gen` → `p.pv.ap(k).gen`):

| Field | Value | Meaning |
|---|---|---|
| `Tg` | 0.02 | Converter current lag, s |
| `Rrpwr` | 10 | Active current recovery ramp, pu/s |
| `Iqrmax` | 99 | Reactive current ramp up, pu/s |
| `Iqrmin` | −99 | Reactive current ramp down, pu/s |
| `Lvplsw` | 1 | Low-voltage power logic on/off |
| `Zerox` | 0.4 | LVPL: voltage where the Ip limit is 0 |
| `Brkpt` | 0.9 | LVPL: voltage where the Ip limit reaches Lvpl1 |
| `Lvpl1` | 1.22 | LVPL: Ip limit at Brkpt |
| `Lvpnt0` | 0.4 | LVACM: Ip = 0 below this voltage |
| `Lvpnt1` | 0.8 | LVACM: full Ip above this voltage |

### A2.5 `batteryParameters` → `p.bess`

Sign: positive power = discharging.

| Field | Value | Unit | Meaning |
|---|---|---|---|
| `Pn` | 30e6 | W | Converter rating of one unit |
| `nAgg` | 1 | – | |
| `nFeeders` | 1 | – | |
| `nPerFeeder` | 1 | – | |
| `Pmax` | 30e6 | W | Max charge/discharge power, plant total |
| `E` | 60e6·3600 | J | Energy capacity, plant total (60 MWh) |
| `SOC0` | 0.5 | – | Initial SOC |
| `SOCmin` | 0.1 | – | No discharge at or below |
| `SOCmax` | 0.9 | – | No charge at or above |

Derived: `N = 1`, `Prated = 30 MW`, `ap` with extra fields `SOC0` and `kSOC` (A2.13).

Note the difference between `Pn` (converter rating, sets the per-unit base and the unit transformer) and `Pmax` (the power limit used by the controllers). Case sets vary `Pmax` without changing the converter (A4.8).

**REEC_C** (`bess.wecc.ctrl` → `p.ctrl.bess.par`): the same fields and values as PV's REEC_B (`Trv` 0.02, `Vdip` 0.90, `Vup` 1.10, `Vref0` 1.0, `dbd1` −0.05, `dbd2` 0.05, `Kqv` 2.0, `Iqh1` 1.05, `Iql1` −1.05, `Qflag` 1, `Kqp` 0.1, `Kqi` 10, `Tiq` 0.02, `dPmax` 99, `dPmin` −99, `Tpord` 0.02, `Imax` 1.1, `PQflag` 0). The SOC limits come from `SOCmin` / `SOCmax` above and are copied in by `controllerParameters`.

**REGC_A** (`bess.wecc.gen` → `p.bess.ap(k).gen`): the same as PV (`Tg` 0.02, `Rrpwr` 10, `Iqrmax` 99, `Iqrmin` −99, `Lvplsw` 1, `Zerox` 0.4, `Brkpt` 0.9, `Lvpl1` 1.22, `Lvpnt0` 0.4, `Lvpnt1` 0.8).

**Why duplicate the PV values instead of sharing.** The two models are different WECC blocks (REEC_B and REEC_C), and the values will diverge once project data replace the typical values. Separate fields keep them independent (inferred).

### A2.6 `gridParameters` → `p.grid`

| Field | Value | Unit | Meaning |
|---|---|---|---|
| `Pmax_poc` | 180e6 | W | Export limit at the POC; also the base of the SCR |
| `Pbase` | 180e6 | W | Power base: system MVA base of the network model *and* the HPPC per-unit base |
| `Pref` | 150e6 | W | TSO power reference |
| `f0` | 50 | Hz | Nominal frequency |
| `Pstep.t0` | 40 | s | Time of a step in P_ref |
| `Pstep.dP` | 0 | W | Size of the P_ref step (0 = none); case set `hppc_p` |
| `Qref` | 0 | var | TSO Q reference (+ = export) |
| `Qstep.t0` | 40 | s | Time of a Q_ref step |
| `Qstep.dQ` | 0 | var | Size of the Q_ref step; case sets `hppc_qv`, `comm_delay` |
| `Vpoc` | 132e3 | V | Nominal POC voltage |
| `SCR` | 10 | – | Short-circuit ratio Ssc / Pmax_poc |
| `XR` | 10 | – | X/R of the grid impedance |
| `E` | 1.0 | pu | Thevenin source voltage (angle 0) |
| `dip.t0` | 30 | s | Start of a grid source voltage dip |
| `dip.dur` | 0.15 | s | Duration |
| `dip.depth` | 0 | pu | Reduction of E (0 = none; negative = voltage rise) |
| `fev.t0` | 40 | s | Time of a frequency step |
| `fev.df` | 0 | pu | Frequency change in pu of f0 (0.004 = +0.2 Hz) |

**Why events are parameters.** Dips, frequency steps and reference steps are fields of `p`. A case set can then create any event by setting numbers, and with Fast Restart or Rapid Accelerator the event changes between runs without a recompile (design.md section 11: "controller parameters, set-points, events (dip, frequency step)" are tunable). The default event times (dip at 30 s, steps at 40 s) are placeholders. The case sets move the dip to 45 s and frequency steps to 50 s (A4.9).

**`dip` doubles as a sustained voltage change.** With a long duration (`dur = 100`), `depth = 0.02` is a sustained 2 % grid voltage drop, and `depth = −0.08` a sustained rise (`hppc_qv`). No separate parameter was added for this.

### A2.7 `networkParameters` → `p.net`

| Field | Value | Meaning |
|---|---|---|
| `Vhpp` | 33e3 V | 33 kV HPP collector busbar |
| `solverType` | `"nodal"` | Default network model (`"nodal"` or `"simscape"`); a run can override it |
| `wt.Vcol` | 10e3 V | Wind collector voltage |
| `wt.unit` | `S` 5.6e6 VA, `uk` 0.07, `XR` 7 | 0.69/10 kV unit transformer per turbine (scaled by `nAgg`) |
| `wt.cable` | `R` 0.05 Ω/km, `X` 0.095 Ω/km, `C` 0.50e-6 F/km, `len1` 2.0 km, `len` 1.0 km, `nPar` 15 | 10 kV, 630 mm² Al; first segment and between taps; 15 parallel feeders |
| `wt.plant` | `S` 170e6, `uk` 0.10, `XR` 30, `Rm` 1000, `Xm` 500 | 10/33 kV wind transformer with magnetizing branch (pu on S) |
| `pv.Vcol` | 33e3 V | |
| `pv.unit` | `S` 5.5e6, `uk` 0.06, `XR` 7 | 0.6/33 kV per station |
| `pv.cable` | `R` 0.10, `X` 0.12, `C` 0.25e-6, `len1` 2.0, `len` 0.5, `nPar` 1 | 33 kV, 240 mm² Al |
| `pv.plant` | `S` 0, `uk` 0, `XR` 1, `Rm` 1, `Xm` 1 | None: `S = 0` means "no plant transformer", feeders end on the HPP busbar |
| `bess.Vcol` | 33e3 V | |
| `bess.unit` | `S` 33e6, `uk` 0.08, `XR` 8 | 0.69/33 kV |
| `bess.cable` | `R` 0.05, `X` 0.10, `C` 0.35e-6, `len1` 0.3, `len` 0.3, `nPar` 1 | Short 33 kV cable, 630 mm² Al |
| `bess.plant` | `S` 0 ... | None |
| `main` | `S` 250e6, `uk` 0.12, `XR` 40, `Rm` 1000, `Xm` 500 | 33/132 kV HPP transformer (tap changer later) |

**Why `plant.S = 0` as "none".** Every plant type has the same struct shape, so `networkSolverData` and `buildHPP` loop over the three types with the same code. A zero rating is the flag (`hasPlantTr = n.(t).plant.S > 0`). `Rm`, `Xm`, `XR` = 1 avoid divisions by zero if a value is used by mistake (inferred).

**Why the magnetizing branch.** The Simscape frequency-and-time model needs a finite magnetizing branch. To make both network models describe the same transformers, the nodal model got the same branch as a shunt (design.md section 5). Rm = 1000 pu gives 0.1 % no-load loss and Xm = 500 pu 0.2 % magnetizing current, "typical for large power transformers". Before the branch was added to the nodal model, P_poc differed by 0.84 MW between the two models. Part B2 covers the physics.

### A2.8 `commParameters` → `p.comm`

| Field | Value | Meaning |
|---|---|---|
| `tso` | 0.200 s | TSO set-points → HPPC |
| `meas_hppc` | 0.020 s | POC and plant measurements → HPPC |
| `hppc_to_plant` | 0.050 s | HPPC set-points → plant controllers |
| `meas_plant` | 0.020 s | Asset measurements → plant controllers |
| `plant_to_asset` | 0.020 s | Plant controller set-points → asset controllers |
| `maxDelay` | 0.5 s | Largest delay any case may use: buffer size of the variable delays (compile time) |
| `delays` | `"auto"` | `"fixed"`, `"variable"` or `"auto"` |

Derived: `p.comm.steps.<path> = round(p.comm.<path> / p.sim.Ts)` for the five paths: `tso` 200, `meas_hppc` 20, `hppc_to_plant` 50, `meas_plant` 20, `plant_to_asset` 20 steps.

**What `delays` means** (comment in `commParameters.m`, `scripts/delayMode.m`):
- `"fixed"`: delay lengths are block parameters, fixed when the model is compiled. Fastest (Rapid Accelerator ~1.5 s per 60 s case). A case with other delays needs a rebuild or recompile.
- `"variable"`: variable-length delays. The length is a tunable input, so delays change between runs up to `maxDelay`. About 0.6–1 s slower per 60 s run in Rapid Accelerator (2.2–2.6 s instead of 1.5–1.7 s).
- `"auto"` (default): `delayMode` picks `"variable"` when the case set has a `comm.*` column (other than `comm.maxDelay`, `comm.delays`), otherwise `"fixed"`. `runCase`, `runSweep` and `main.m` can override this (option `Delays`).

`delayMode` writes the resolved mode back into `p.comm.delays` and returns the variant code `HPP_delays` (1 fixed, 2 variable). `delaySignature(p)` returns the compile-time part (the steps with fixed delays, `maxDelay` with variable delays) for the compile signatures (A5).

**Why whole steps.** Discrete delays of whole simulation steps replaced continuous transport delays in `ae5559d` for speed. The receiving controller samples the delayed signal at its own rate.

**Why `maxDelay` is a parameter.** A variable-length delay needs a buffer of fixed size at compile time. 0.5 s covers the `comm_delay` set (largest 0.4 s for `tso` in "all delays x2"). `deriveParameters` asserts that every delay lies in `0 ... maxDelay` (A2.13), so a case cannot ask for more than the buffer holds.

**History.** Variable delays were first tried and dropped in normal mode (`fb180ee`: "each made every run 0.5–0.75 s per simulated second slower"). They came back on 2026-11-19 (`ac18a90`) as a *choice* next to fixed delays, compiled only when needed through a Variant Subsystem. This is the reason for the three-way setting.

**Consequences.** To compare with a model without delays, set all five to 0 (comment in the file, case `no delays` in `comm_delay`). Chapter B6 covers how the delays and the one-step measurement memories interact.

### A2.9 `plantControlParameters` → `p.pc`

`p.pc.wppc`, `p.pc.sppc` and `p.pc.bessc` start from the same defaults (local function `repcDefaults`). Only `Dup = 0` for wind and PV is different ("wind and PV run at available power: no upward frequency response"). Each is copied into `p.ctrl.<plant>.par` by `controllerParameters`, which then adds or overrides some fields (A2.12). All pu on the plant rating, times in s.

| Field | Value | Meaning |
|---|---|---|
| `Tfltr` | 0.02 | Voltage and Q measurement filter |
| `RefFlag` | 0 | 0 Q control (track Q_set), 1 voltage control (Vref, droop Kc). Overridden to 1 by `controllerParameters` in HPPC voltage control with `Vlocal` = 1 |
| `Vref` | 1.0 | Voltage reference (RefFlag 1); overridden with `p.hppc.Vref` in local voltage control |
| `Kc` | 0.0 | Reactive droop, pu V / pu Q; overridden in local voltage control |
| `dbd1`, `dbd2` | 0, 0 | Deadband on the V/Q error |
| `emax`, `emin` | 0.1, −0.1 | Error limits |
| `Kp` | 0.1 | Q PI proportional gain (RefFlag 0) |
| `Ki` | 1.0 | Q PI integral gain, 1/s (~1 s) |
| `KpV` | 0.5 | V PI (RefFlag 1), normalised by the loop gain Kc + dVdQ |
| `KiV` | 2.5 | 1/s (~0.6 s: slower than asset Q loops at ~0.1 s) |
| `dVdQ` | 0.3 | Busbar dV/dQ; placeholder, always overwritten by `controllerParameters` from the network |
| `Vfrz` | 0.7 | Q/V PI frozen while V < Vfrz |
| `Qmax`, `Qmin` | 0.5, −0.5 | Plant Q order limits |
| `Tfv` | 0.05 | Q order lag (REPC_A lead-lag with Tft = 0) |
| `Tp` | 0.02 | P measurement filter |
| `Frqflag` | 0 | 1 closed-loop P and frequency response (decentral); 0 P_set passed on (central control) |
| `Freq_ref` | 1.0 | Frequency reference, pu |
| `fdbd1`, `fdbd2` | −0.0004, 0.0004 | Frequency deadband, pu (±20 mHz) |
| `Ddn` | 20 | Droop for over-frequency, pu P / pu f (5 %) |
| `Dup` | 20 (0 for wppc, sppc) | Droop for under-frequency |
| `femax`, `femin` | 0.5, −0.5 | P error limits |
| `Kpg` | 0.1 | P PI proportional gain |
| `Kig` | 1.0 | P PI integral gain, 1/s |
| `Tlag` | 0.1 | P order lag |

**Why one default for all three.** All three plant controllers use the WECC REPC_A structure (`buildPlantCtrlRepc`). The WPPC uses REPC_A "for now". The IEC 61400-27-1 WPP control model is a later refinement. Separate structs per plant still let a case override one plant (`{"pc.wppc.Frqflag", 0}`).

**Why `Frqflag = 0` by default.** Central control (decided 2026-11-17): the HPPC closes the P loop and gives the frequency response. The stage-4 decentral setup is kept as the `decentral` cases in `freq_step` (A4.9).

### A2.10 `hppcParameters` → `p.hppc`

Copied into `p.ctrl.hppc.par` by `controllerParameters` (all fields). Per unit on the POC base (180 MW), frequency in pu of 50 Hz, times in s. Grid code: Energinet DK1 (Danish RfG requirements, PPM type D; TR 3.3.1 for storage). The ranges in the comments are the grid-code ranges; the defaults are the DK1 default settings. Part B5 explains the functions.

**P control at the POC:**

| Field | Value | Meaning |
|---|---|---|
| `Pflag` | 1 | 1 closed-loop P (PI on P_target − P_poc), 0 open loop |
| `Tp` | 0.2 | P_poc measurement filter (≥ Ts = 0.1 s) |
| `Tref` | 0.4 | Reference filter on the target in the PI error |
| `Vfrz` | 0.9 | PIs frozen while V_poc < Vfrz ... |
| `Thld` | 1.0 | ... and for Thld after it |
| `Kpg` | 0.1 | P PI proportional gain |
| `Kig` | 0.5 | P PI integral gain, 1/s |
| `femax`, `femin` | 0.2, −0.2 | P error limits |

**Ramp rate constraint:**

| Field | Value | Meaning |
|---|---|---|
| `rampFlag` | 1 | 1 on, 0 off |
| `rampUp`, `rampDn` | 0.20 | pu of Pn per minute (20 %/min = 36 MW/min); range 1–20 %/min, at most 60 MW/min |

**Frequency response:**

| Field | Value | Meaning |
|---|---|---|
| `Frqflag` | 1 | 1 frequency response in the HPPC (central), 0 none |
| `Freq_ref` | 1.0 | Nominal frequency, pu |
| `fO` | 50.2/50 | LFSM-O threshold (range 50.2–50.5 Hz) |
| `sO` | 0.05 | LFSM-O droop (2–12 %) |
| `fU` | 49.8/50 | LFSM-U threshold (49.5–49.8 Hz) |
| `sU` | 0.05 | LFSM-U droop |
| `FSMflag` | 0 | 1 FSM active (FCR delivery); BESS keeps `fsmRange` as reserve |
| `sFSM` | 0.05 | FSM droop |
| `fsmDb` | 0.010/50 | FSM deadband (10 mHz) |
| `fsmRange` | 0.10 | FSM range, pu of Pn |

**SOC-aware BESS limits:** `SOCband` = 0.1 (discharge or charge derated to zero over this band above SOCmin or below SOCmax).

**Q / V / PF control:**

| Field | Value | Meaning |
|---|---|---|
| `Qflag` | 1 | 1 closed loop, 0 open loop split of Q_ref |
| `QawFlag` | 1 | Anti-windup over the plant limits (`Q_sat`) |
| `RefFlag` | 0 | 0 Q control, 1 voltage control, 2 PF control |
| `Vref` | 1.0 | Voltage set-point (0.95–1.05) |
| `Vslope` | 0.04 | V change for a Q change of Qmax (2–7 %) |
| `PF` | 1.0 | PF set-point (RefFlag 2) |
| `PFsign` | 1 | +1 Q export with P, −1 import |
| `Tq` | 0.2 | Q_poc and V_poc measurement filter |
| `TrefQ` | 0.4 | Reference filter on Q_ref in the PI error |
| `Kpq` | 0.1 | Q/PF PI proportional gain |
| `Kiq` | 0.2 | Q/PF PI integral gain, 1/s (~5 s, slower than plant Q loops ~1 s) |
| `Vlocal` | 1 | Voltage control: 1 cascade (plants on V-Q droop, HPPC trims), 0 direct POC voltage control |
| `KpVt` | 0 | HPPC voltage trim gains (Vlocal 1), normalised: ~3 s |
| `KiVt` | 0.3 | |
| `KcBusMin` | 0.02 | Minimum busbar droop, pu V / pu Q on Pbase |
| `cascadeSep` | 3 | Each level at least this much slower than the one below (warning) |
| `KpV` | 0.5 | Direct voltage-control PI (Vlocal 0), normalised |
| `KiV` | 2.5 | 1/s |
| `SCRset` | NaN | Grid strength assumed by the voltage control; NaN = the actual `p.grid.SCR` |
| `eqmax`, `eqmin` | 0.2, −0.2 | Q/V error limits |
| `Qmax`, `Qmin` | 0.33, −0.33 | Q limits at the POC, pu on 180 MVA (~60 Mvar, PF 0.95 at 180 MW) |

**Why `SCRset` defaults to NaN.** In reality the voltage-control gains are set at commissioning for the grid strength known then. NaN means "commissioned for the grid you simulate", so a weak-grid case is also correctly tuned. A case can set `SCRset` explicitly to simulate a wrongly tuned controller (`hppc_qv` "V ctrl, SCR3, set for SCR10").

### A2.11 `deriveParameters`: what is derived and why

`params/deriveParameters.m` is called at the end of `HPP_parameters` and by `applyCase`. In order:

1. **Instances and ratings** for `wt`, `pv`, `bess`:
   `p.(t).N = nFeeders·nPerFeeder`, `p.(t).Prated = N·nAgg·Pn`. Defaults: N = 2 / 2 / 1, Prated = 150 / 50 / 30 MW.
2. **Delay steps:** `nMax = round(maxDelay/Ts)` (500). For every field of `p.comm` except `maxDelay`, `steps`, `delays`: `n = round(delay/Ts)`, asserted to lie in `0 ... nMax`, stored in `p.comm.steps.<field>`. Note that the loop takes *all other* fields of `p.comm`. A new delay path is just a new field in `commParameters`.
3. **Controller parameters:** `p.ctrl = controllerParameters(p)` (A2.12).
4. **Asset model arguments**, one per instance, identical for now:
   - `p.wt.ap(k) = struct("ctrl", p.ctrl.wt.par, "gen", p.wt.iec.gen)`
   - `p.pv.ap(k) = struct("ctrl", p.ctrl.pv.par, "gen", p.pv.wecc.gen)`
   - `p.bess.ap(k) = struct("ctrl", p.ctrl.bess.par, "gen", p.bess.wecc.gen, "SOC0", p.bess.SOC0, "kSOC", p.ctrl.bess.Pbase / Eunit)` with `Eunit = p.bess.E / p.bess.N`.
   `kSOC` is dSOC/dt per pu of power, 1/s. Default 30e6 / 216e9 = 1.389e-4 1/s. At 1 pu (30 MW) the SOC changes by 0.5 per hour, consistent with 60 MWh / 30 MW = 2 h for 0 → 1.
5. **Network solver data:** `p.net.solver = networkSolverData(p)` (A2.13).

**Why derive instead of set.**
- *Consistency.* A value that follows from others (a rating, a per-unit limit, a matrix inverse) would go stale the moment someone changes one input in a case. Deriving it every time makes that impossible.
- *Case sets only list inputs.* A case that changes `bess.Pmax` automatically gets the new controller limit, the new `ap`, everything. The case author does not need to know which derived values depend on it.
- *No duplicated numbers.* Each number is set in exactly one place.

**Consequences.**
- Never set a derived field (`N`, `Prated`, `ap`, `comm.steps`, `p.ctrl.*`, `p.net.solver`) in a case or by hand before a run. `applyCase` will overwrite it (A4.6).
- A new derived value goes into `deriveParameters` (or `controllerParameters` if it concerns a controller), never into a component function.

### A2.12 `controllerParameters`: `p.ctrl`

**What every controller has.** Three fields:

| Field | Meaning |
|---|---|
| `Ts` | Fixed sample time, s. Not tunable. |
| `Pbase` | Power base of its per-unit interface, W. Not tunable. |
| `par` | Tunable parameters in pu on `Pbase`. Passed as the model argument `par`. "Becomes the parameter struct of the generated code / IEEE-CIGRE DLL." |

`Ts` and `Pbase` are fixed settings. The W ↔ pu conversions use `Pbase` in `HPP.slx`, outside the controllers.

**HPPC (`ctrl.hppc`).**

| Field | Default | How |
|---|---|---|
| `Ts` | 0.1 s | set |
| `Pbase` | 180e6 W | `p.grid.Pbase` |
| `par.Ppoc_max` | 1.0 pu | `Pmax_poc / Pbase`; used by the HPPC "export limit" block (see note) |
| `par.qshare` | [0.652; 0.217; 0.130] | `Prated / sum(Prated)`, Q split wind/PV/BESS by rating |
| `par.<all of p.hppc>` | as A2.10 | `copyFields(par, p.hppc)` |
| `par.dVdQ` | 0.1 | `1 / scr`, `scr = SCRset`, or `p.grid.SCR` if NaN |
| `par.SOCmin`, `par.SOCmax` | 0.1, 0.9 | from `p.bess` |
| `par.LV`, `par.KpVn`, `par.KiVn`, `par.QmaxOrd`, `par.QminOrd` | see below | voltage-control loop gain, gains, order range |
| `par.isV`, `par.isPF`, `par.tanphi`, `par.KpQ`, `par.KiQ` | see A2.16 | derived for tunable block expressions |
| `cascade` | struct | timing check result (not in `par`) |

> Note (code against code comment): `controllerParameters.m` line 25 marks `Ppoc_max` "(not used yet)". It is used: the HPPC template has a constant `par.Ppoc_max` feeding the "export limit" stage (`utilities/build/buildHPP.m` ~line 2053), and `models/ctrl/HPPC.slx` (`system_root.xml`) contains `par.Ppoc_max`. The comment is stale.

**Plant controllers (`ctrl.wppc`, `ctrl.sppc`, `ctrl.bessc`).** From the local function `plantCtrl(Prated, N)`:

| Field | Default | How |
|---|---|---|
| `Ts` | 0.02 s | set |
| `Pbase` | 150 / 50 / 30 MW | plant `Prated` |
| `par.share` | `ones(N,1)/N` | fallback split when nothing is available; its length also sets the controller's port width N in the model workspace (`N = numel(par.share)`) |
| `par.<all of p.pc.<plant>>` | A2.9 | copied |
| `par.SOCmin`, `par.SOCmax` (bessc only) | 0.1, 0.9 | from `p.bess` |
| `par.dVdQ` | `xBus · sumP` | busbar dV/dQ on the plant rating (below) |
| `par.RefFlag`, `par.Vref`, `par.Kc` | overridden in local voltage control | below |

**Asset controllers (`ctrl.wt`, `ctrl.pv`, `ctrl.bess`).** From `assetCtrl(Ts, Pbase)`:

| Field | Default | How |
|---|---|---|
| `Ts` | 1 ms | `p.sim.Ts` |
| `Pbase` | 75 / 25 / 30 MW | `nAgg · Pn` (unit rating) |
| `par.Vmin` | 0.01 pu | voltage floor for current commands (avoids division by zero in i = p/V) |
| WT: `par.<p.wt.iec.ctrl>` + `par.MqG1` | | copied; `MqG1 = double(MqG == 1)` |
| PV: `par.<p.pv.wecc.ctrl>` | | copied |
| BESS: `par.Pmax`, `par.<p.bess.wecc.ctrl>`, `par.SOCmin`, `par.SOCmax` | `Pmax = (p.bess.Pmax / N) / Pbase` = 1.0 pu | |

**Voltage-control derivation** (lines 52–96). With `Sb = 180 MVA`:
- `xTr = uk·XR/hypot(1,XR) · Sb/S` of the 33/132 kV transformer = 0.12 · 40/√1601 · 180/250 ≈ 0.0864 pu on 180 MVA.
- `xBus = 1/scr + xTr` ≈ 0.1864: dV/dQ seen at the 33 kV busbar.
- `KcPoc = Vslope / Qmax` = 0.04/0.33 ≈ 0.1212: the required slope at the POC in pu V per pu Q.
- `KcBus = max(KcPoc − xTr, KcBusMin)` ≈ 0.0348: the busbar droop of all plants together. The plants alone then give about the POC slope.
- `sumP = sum(Prated)/Sb` = 230/180 ≈ 1.278: converts from the 180 MVA base to each plant's own rating with *equal* droop, so the Q split follows the ratings.
- Each plant: `par.dVdQ = xBus · sumP` ≈ 0.238. If voltage control is local (`vlocal = RefFlag == 1 && Vlocal == 1`): `par.RefFlag = 1`, `par.Vref = p.hppc.Vref`, `par.Kc = KcBus · sumP` ≈ 0.0445.
- HPPC with `vlocal`: `auth = KcBus/(KcBus + xBus)` ≈ 0.158 (share of an offset change that becomes real Q), `LV = (KcPoc + 1/scr)·auth`, `KpVn = KpVt`, `KiVn = KiVt`, `QmaxOrd = Qmax/auth` ≈ 2.09, `QminOrd = Qmin/auth`.
- HPPC without `vlocal`: `LV = KcPoc + 1/scr` ≈ 0.221, `KpVn = KpV`, `KiVn = KiV`, `QmaxOrd = Qmax`, `QminOrd = Qmin`.

**Why the plant order limits are not tightened.** Comment in the code: the plant order limits act on the asset terminals, and "the wind plant loses up to ~40 % of it in its transformers and cables". A tighter order limit held the wind plant at 28 instead of 49 Mvar (design.md stage 5 step 4). The POC Q limit is held in the HPPC instead.

**Why a minimum droop `KcBusMin`.** A negative busbar droop "makes the parallel plants unstable against each other (circulating Q)". At a 2 % slope `KcPoc − xTr` is negative, so the minimum applies and the HPPC trim corrects the slope within ~3 s (design.md).

**Why these values are computed here, not in the models.** They are commissioning-type calculations (loop gains from network data). Doing them in MATLAB keeps the controller blocks simple and tunable (A2.16), and a case that changes SCR, the transformer or the slope automatically gets consistent gains.

### A2.13 `networkSolverData` → `p.net.solver`

Builds the nodal admittance matrix of the layout and returns its inverse as real arrays. Part B2 explains the model. Here are the structure and the fields.

- Per unit on `Sb = p.grid.Pbase` and the nominal voltage of each level (ideal ratios).
- Asset order (global index k): wind, PV, BESS. Within a plant by feeder, and along each feeder from the collector bus outwards.
- Nodes: asset terminals `1..nA`, feeder taps `nA+1..2nA`, then one collector bus per plant *with* a plant transformer, then the 33 kV HPP busbar (`bus = nNodes − 1`), then the POC (`poc = nNodes`). Default: nA = 5, nodes = 2·5 + 1 + 2 = 13.
- Elements: unit transformer `impedance(uk, XR)·Sb/(S·nAgg)`; feeder segments as π with `Zc = (R + jX)·L/nPar/Zb` and `Bh = ω·C·L·nPar/2·Zb`; plant transformer with magnetizing shunt at the collector side; HPP transformer with magnetizing shunt at the 33 kV side; grid `Zg = impedance(1, XR)·Sb/(SCR·Pmax_poc)` (0.1 pu at SCR 10) as a Norton admittance at the POC.

Returned fields:

| Field | Meaning |
|---|---|
| `nA`, `nNodes`, `bus`, `poc` | sizes and node indices |
| `plantOf` | plant (1 wind, 2 PV, 3 BESS) of each asset |
| `Zre`, `Zim` | real and imaginary parts of Z = Y⁻¹ |
| `brFrom`, `brZre`, `brZim`, `brB`, `brPlant` | the plant branches into the HPP busbar (from-node, impedance, busbar-end shunt susceptance, plant): P/Q of each plant are the sums of their flows at the busbar end (plant measurement points) |
| `Zg_re`, `Zg_im` | grid impedance |
| `E` | Thevenin source voltage |
| `Sb` | base |

**Why real arrays.** "Returned as real arrays so it can be a block parameter." Complex values and nested structs are awkward as tunable block parameters. In the nodal network the numbers enter the solver through tunable Constant blocks, so with Fast Restart and Rapid Accelerator an SCR or cable change needs no recompile. Only the node structure (size of Z, set by N) is fixed (`fb180ee`). Tried and dropped: "the whole network data as one tunable parameter structure (it made every run 0.5–0.75 s per simulated second slower)" (design.md section 11).

**Why Z = Y⁻¹ once.** The network is linear and constant during a run (events change the source, not the network), so the inverse is computed once in MATLAB, and each step is a matrix-vector product `V = Z·(I_inj + I_grid)` (design.md section 5).

### A2.14 Per-unit bases of each controller

| Controller | `Pbase` | Default | Frequency | Voltage |
|---|---|---|---|---|
| HPPC | `p.grid.Pbase` (POC rating) | 180 MW | pu of 50 Hz | pu of 132 kV (V_poc) |
| WPPC / SPPC / BESSC | plant `Prated` | 150 / 50 / 30 MW | pu of 50 Hz | pu of 33 kV (V_bus) |
| WT / PV / BESS asset controllers | unit rating `nAgg·Pn` | 75 / 25 / 30 MW | – | pu of the terminal nominal voltage |
| Network (not a controller) | `p.grid.Pbase` | 180 MVA | – | pu of each level's nominal |

**Why each controller has its own base.** A real controller is configured in per unit of what it controls. A plant controller from a vendor knows its plant rating, not the HPP's POC rating. Own bases also make each controller (and later its DLL) reusable in another plant without rescaling its gains. The conversions sit in the link blocks of `HPP.slx` (A3), and in PSCAD on the PSCAD side.

**Consequence.** A gain in `p.pc` is "per plant rating", a gain in `p.hppc` "per POC base". The same number means a different MW value in the two places. The voltage-control derivation (A2.12) converts explicitly between them with `sumP`.

### A2.15 Model arguments: `par` and `ap`

**Simulink background.** A *referenced model* is a separate `.slx` used inside another model through a *Model block*. A *model argument* is a variable in the referenced model's model workspace that is marked as an argument (`ParameterArgumentNames`). Each Model block that references the model can then give the argument its own value (an *instance argument*, set in the block's `InstanceParameters`). Model arguments are tunable: they can change between runs of a compiled model, and in generated code they become a per-instance parameter struct.

**Controllers: `par`.** Each controller model has the model argument `par` (`set_param(mdl, "ParameterArgumentNames", "par")`). Its model workspace (data source "MATLAB Code") holds standalone defaults:

```matlab
pDefault = HPP_parameters();
Ts  = pDefault.ctrl.<c>.Ts;     % fixed sample time
par = pDefault.ctrl.<c>.par;    % tunable parameters (model argument)
N   = numel(par.share);         % plant controllers only: port width
clear pDefault
```

In `HPP.slx`, the Model block for the HPPC sets `par` to `p.ctrl.hppc.par`, and the plant controller blocks to `p.ctrl.<wppc|sppc|bessc>.par` (`setArg`, `utilities/build/buildHPP.m` ~lines 901, 988).

**Assets: `ap`.** Each asset model has the model argument `ap` and a model workspace with `ap = pDefault.<type>.ap(1)` and `Tsim = pDefault.sim.Ts`. In `HPP.slx` instance k gets `p.<type>.ap(k)` (`buildHPP.m` ~line 1048: `setArg(..., "ap", sprintf("p.%s.ap(%d)", pl.t, k))`). Inside the asset model, the asset controller Model block gets `par = ap.ctrl` (~line 803). So an asset controller's `par` is passed through two levels: `p.<type>.ap(k).ctrl` → asset model `ap.ctrl` → controller `par`.

**Why this mechanism.**
- *DLL interface.* The generated code of each controller will have exactly one parameter struct, `par`, which becomes the IEEE/CIGRE DLL's parameter list (design.md section 4).
- *Standalone use.* Thanks to the model workspace defaults, each controller opens, compiles and generates code on its own, without `HPP.slx` or a base-workspace `p` (commit `467b5f2`).
- *Per instance.* Each asset instance already has its own `ap(k)`. Different turbine types in one plant only need different values in `ap(k)`, with no model change (design.md section 9). For now all instances are identical (`repmat`).
- *Tunable.* Model arguments change between Fast Restart and Rapid Accelerator runs without a recompile.

**Why `ap` bundles `ctrl` and `gen`.** The asset model contains both the asset controller (a future DLL) and the converter model (plant side). One argument per instance keeps the Model block in `HPP.slx` simple. Inside, `ap.ctrl` goes to the controller and `ap.gen` (plus `SOC0`, `kSOC` for BESS) to the plant-side blocks.

**Consequences.**
- A new controller parameter: add it to the component function (e.g. `p.hppc.X`). It reaches `par` automatically if the component is copied wholesale (`p.hppc`, `p.pc.*`, `p.wt.iec.ctrl`, `p.pv.wecc.ctrl`, `p.bess.wecc.ctrl`). Then use `par.X` in the model.
- The interface (argument name and source) is defined in `utilities/build/modelInterfaces.m` (`arg`, `src`), and `checkModels` checks that every `par`/`ap` field a model uses exists in `HPP_parameters()` (A3).
- `Ts` and `N` are not arguments. They are compile-time. N fixes the port widths, so a DLL is built for one N (design.md section 3).

### A2.16 The block-expression rule and the derived fields it requires

**The rule.** In block dialog parameters that use `par` or `ap`, only `+ - * /`, parentheses, numbers and field access are allowed. No `^`, `sqrt`, comparisons (`==`, `>`), function calls, `[a b]` vectors or indexing. Code *inside* MATLAB Function blocks is not restricted. (README "Rules for hand edits", `scripts/checkModels.m` help.)

**Why.** Compiled simulation (model reference accelerator, Rapid Accelerator) and code generation (the future DLLs) keep a block parameter that depends on a model argument tunable only if the expression is simple arithmetic. Otherwise the build fails, or the value is frozen at build time. design.md section 13: "The first builds failed on `par.PF^2`, `sqrt`, comparisons (`par.RefFlag == 1`, `double(par.MqG == 1)`) and a Transfer Fcn denominator `[ap.gen.Tg 1]`." MATLAB Function code is exempt because it is compiled as code: `par` is an input struct there, and any MATLAB expression on it becomes generated C.

**How `checkModels` enforces it** (`scripts/checkModels.m` ~lines 122–165). For each block dialog parameter that references `par`/`ap` (MATLAB Function blocks skipped), it strips whitespace, flags any name followed by `(` (function call or indexing), removes names with field access, numbers and the allowed operators, and reports whatever is left. The finding table gives block path and fix: "compute it in `params/controllerParameters.m` as a new `par` field ... and use only that field here; or move the calculation into a MATLAB Function block".

**Derived fields that exist because of the rule:**

| Field | Definition (`controllerParameters`) | Replaces in the block |
|---|---|---|
| `p.ctrl.hppc.par.isV` | `double(p.hppc.RefFlag == 1)` | `par.RefFlag == 1` (voltage control switch) |
| `p.ctrl.hppc.par.isPF` | `double(p.hppc.RefFlag == 2)` | `par.RefFlag == 2` (PF control switch) |
| `p.ctrl.hppc.par.tanphi` | `PFsign · sqrt(1/PF^2 − 1)` | `tan(acos(par.PF))`, `par.PF^2`, `sqrt` |
| `p.ctrl.hppc.par.KpQ` | `KpVn/LV` if RefFlag = 1, else `Kpq` | a mode-dependent gain selection |
| `p.ctrl.hppc.par.KiQ` | `KiVn/LV` if RefFlag = 1, else `Kiq` | idem |
| `p.ctrl.wt.par.MqG1` | `double(p.wt.iec.ctrl.MqG == 1)` | `double(par.MqG == 1)` (closed-loop Q flag) |

A flag such as `isV` is then used as a multiplier (`isV·x + (1 − isV)·y`), which is plain arithmetic. The converter current lags `[ap.gen.Tg 1]` were rebuilt as gain `1/Tg` and an integrator in a loop (`contLag` in `buildHPP.m`), with the same dynamics.

Other derived `par` fields also serve the rule as a side effect, because they move non-trivial expressions out of blocks: `qshare`, `dVdQ`, `LV`, `QmaxOrd`, `QminOrd`, `Ppoc_max`, `ctrl.bess.par.Pmax`, `ap.kSOC` (inferred: each would otherwise need a division or a function of several parameters in a block).

**Consequences.**
- When a block needs a nonlinear function of parameters, add a derived `par` field in `controllerParameters` and reference that. Or put the logic into the MATLAB Function block.
- A derived field is fixed per run (computed before the run). It is still tunable between runs, because the new `p` carries the new value.
- `checkModels` runs at the start of `buildHPP` and of a Rapid Accelerator build (warning; error with `Strict=true`, which is what the `check` task in `buildfile.m` uses).

### A2.17 Checks in `controllerParameters`

| Check | Code | Effect |
|---|---|---|
| Ramp range | `rampMax = min(0.20, 60e6/Pbase)` (= 0.20 for 180 MW); assert `0.01 ≤ rampUp, rampDn ≤ rampMax` | error: "within 1-20 % of Pn per minute (max 60 MW/min)" |
| Droop range | assert `sO`, `sU`, `sFSM` in `[0.02, 0.12]` | error: "hppc droops must be within 2-12 %" |
| Cascade timing | `tau(kp, ki) = (1 + kp)/ki`; `tAsset` = max over WT/PV/BESS asset Q PIs; `tPlant` = plant voltage PI (vlocal) or plant Q PI; `tHppc` = HPPC trim (vlocal), HPPC voltage, or HPPC Q/PF; warn if `tPlant < sep·tAsset` or `tHppc < sep·tPlant` | warning `controllerParameters:cascade`; result stored in `p.ctrl.hppc.cascade` (`asset`, `plant`, `hppc`, `plantLoop`, `hppcLoop`) |
| Sample times | for every controller: `Ts/sim.Ts` integer (tolerance 1e-9) | error |

Default cascade (Q control): asset 1.1/10 = 0.11 s, plant Q 1.1/1 = 1.1 s, HPPC Q 1.1/0.2 = 5.5 s (ratios 10 and 5, both ≥ 3). With local voltage control: plant voltage 1.5/2.5 = 0.6 s, HPPC trim 1/0.3 ≈ 3.3 s.

**Why check grid-code ranges in the parameter code.** A case set that sets an illegal ramp or droop fails at once, before a simulation, instead of producing a non-compliant result that looks valid. The ranges are from Energinet DK1 (RfG 15(6)(e), TR 3.3.1 §32; droops 2–12 %).

**Why a warning, not an error, for the cascade.** Some studies deliberately break the cascade (for example `Vlocal = 0`, direct POC voltage control, which the comment says "breaks the cascade rule"). The warning makes it visible without blocking the study. The estimate `(1 + Kp)/Ki` is the closed-loop time constant of a PI on a unit-gain plant (gains normalised by the loop gain). It is a rule of thumb, not a full analysis.

**Why cascade separation matters** (design.md stage 5 step 4): the voltage loops are nested (asset Q → plant Q/V → HPPC). A slow outer loop over a fast inner loop behaves well; similar speeds make the loops fight. The values were retuned for this when the cascade was introduced: asset Q PIs 5 → 10 /s, HPPC Q/PF 0.5 → 0.2 /s (`309edbb`).

### A2.18 Notes: docs against code

- `controllerParameters.m` marks `par.Ppoc_max` "(not used yet)". The HPPC uses it in the export limit (A2.12). The code is right, the comment is stale.
- `params/HPP_parameters.m` help shows `p.ctrl.Kp = 0.8` (no such field; and `p.ctrl` is rebuilt by `deriveParameters`).
- `params/simulationParameters.m` refers to "p.ctrl.Ts". It is now one `Ts` per controller.
- design.md section 7 is consistent with the code.

### Recreating it

1. Write one function per component (`simulationParameters`, `windTurbineParameters`, `pvParameters`, `batteryParameters`, `gridParameters`, `networkParameters`, `commParameters`, `plantControlParameters`, `hppcParameters`), each returning only set values, in SI units (W, J, V, s) for plant data and in pu on the controller's base for controller settings. Use the standards' parameter names.
2. Write `HPP_parameters(variant)`: call the component functions, apply the variant, then `deriveParameters`, then `p.meta`.
3. Write `deriveParameters`: N and Prated per type; `comm.steps` with the `maxDelay` assert; `p.ctrl = controllerParameters(p)`; `ap` arrays (`ctrl`, `gen`, BESS `SOC0`, `kSOC`); `p.net.solver = networkSolverData(p)`.
4. Write `controllerParameters`: `Ts`, `Pbase`, `par` per controller (HPPC from `p.hppc` plus `qshare`, `dVdQ`, SOC limits, `Ppoc_max`; plant controllers from `p.pc` plus `share`, `dVdQ`, local voltage-control overrides; asset controllers from the IEC/WECC `ctrl` structs plus `Vmin`, BESS `Pmax`, SOC limits). Add the voltage-control derivation, the tunable-expression helpers (`isV`, `isPF`, `tanphi`, `KpQ`, `KiQ`, `MqG1`), the grid-code asserts, the cascade warning and the `Ts` multiple check.
5. Write `networkSolverData`: build Y in the documented node order, invert it, and return real arrays plus the plant branch data.
6. In each controller model, define `Ts` and `par` in the model workspace from `HPP_parameters()` and mark `par` as the model argument. In each asset model, do the same with `ap` and pass `ap.ctrl` to the asset controller.
7. In `HPP.slx`, set the Model block instance arguments to `p.ctrl.<c>.par` and `p.<type>.ap(k)`.
8. Keep block expressions that use `par`/`ap` to + − × ÷, parentheses, numbers and field access. Add a derived `par` field for anything else.

---

## A3. Model generation and model files

This chapter explains how the Simulink models are made, which of them are generated and which are maintained by hand, and how the generator (`utilities/build/buildHPP.m`, about 2,900 lines) works function by function. It also covers the interface definition (`utilities/build/modelInterfaces.m`), the checks on hand-maintained models (`scripts/checkModels.m`, `scripts/checkModelBuild.m`) and what still has to happen before the controllers become IEEE/CIGRE DLLs for PSCAD. The physics inside the network and the assets, and the control laws, are in Part B; here they appear only as far as the code structure needs them.

### A3.1 The model files and who owns them

#### What

There are eleven model files:

| File | Contents | Owner | Future DLL |
|---|---|---|---|
| `models/HPP.slx` | Top model: TSO sources, HPP controller, three plant subsystems with N asset instances each, network (two variants), logging | **generated** by `buildHPP()` on every call | no |
| `models/ctrl/HPPC.slx` | HPP controller | hand-maintained source | yes |
| `models/ctrl/WPPC.slx`, `SPPC.slx`, `BESSC.slx` | Plant controllers (WECC REPC_A structure), asset signals as vectors of width N | hand-maintained source | yes |
| `models/ctrl/WT_ctrl.slx`, `PV_ctrl.slx`, `BESS_ctrl.slx` | Asset controllers (IEC 61400-27-1 Type 4A; WECC REEC_B; WECC REEC_C) | hand-maintained source | yes |
| `models/assets/WT_asset.slx`, `PV_asset.slx`, `BESS_asset.slx` | Asset (plant-side) models: one asset-controller instance plus the generator/converter model (plus battery energy for BESS) | hand-maintained source | no |

The rule, stated in the header of `buildHPP.m`, in `README.md` ("Who owns which model") and in `docs/design.md` §3 ("Ownership (2026-11-19)"):

- `HPP.slx` is always generated and is never edited by hand. Its description string, set by the build, says so: "Hierarchical HPP model, generated by utilities/build/buildHPP.m. Do not edit by hand: rebuild instead."
- The controller and asset models are the source. They are developed in the Simulink editor. The templates in `buildHPP.m` only create models that are missing.
- The interface between the two worlds (port names and order, the model argument and its default) is written down once, in `utilities/build/modelInterfaces.m`. `checkModels` verifies that the hand-maintained models still match it.

#### Why HPP.slx is generated

The structure of `HPP.slx` depends on parameters: one Model block per asset instance (N = `nFeeders · nPerFeeder` per plant), Selector widths and global asset indices that depend on all three N, Mux widths, and the Simscape network variant, whose blocks are laid out feeder by feeder from `p.<type>.nFeeders` and `nPerFeeder`. Drawing this by hand for every layout change would be slow and error-prone. A script also makes the diagram reproducible and reviewable in git (the `.slx` is a binary zip, the script is text).

Generating it also lets the build enforce a readable layout (aligned ports, straight lines, colour-coded Goto/From) that would decay quickly under hand editing.

#### Why the controller and asset models are hand-maintained

Until 2026-11-19 every model was generated: `buildHPP(All=true)` rebuilt them from templates, `Force=true` overwrote them. Commit `4a15dea` ("Controller and asset models as source: checkModels, no overwrite, fallback to normal mode") reversed this for the ten controller and asset models. The commit message and the design doc give the reasons, which come down to:

- These models are where the research work happens (control laws, flags, limits). Developing them in the Simulink editor is the normal workflow for a control engineer, and the models will later be code-generated as DLLs. Their structure does not depend on N (except the plant controllers' vector width, which is read from the model workspace, see A3.7).
- A template string inside a 2,900-line MATLAB file is a poor place to develop a block diagram. Keeping both in sync by hand is a source of silent divergence.
- Accidental regeneration would destroy hand edits. The commit therefore made overwriting explicit and verified that `buildHPP()` and `buildHPP(All=true)` leave all ten files unchanged (MD5 checked, per the commit message).

#### History

| Commit | Date | Ownership state |
|---|---|---|
| `41a0b4f` Stage 1 | 2026-11-16 | `buildHPP.m` generates all models; each controller is "inports → one MATLAB Function block `logic` → outports". |
| `6c04aa2` Readable model layout | 2026-11-16 | `buildHPP(All=true)` only creates missing controller and asset models; `Force=true` regenerates (overwrites) them. `HPP.slx` stays generated. |
| `0e70343`, `b296539`, `3adff21`, … | 2026-11-16 to 18 | Controllers replaced by block-diagram templates (`buildWTCtrlType4A`, `buildReecCtrl`, `buildPlantCtrlRepc`, `buildHppcCtrl`), regenerated with `Force=true` as the stages progressed. |
| `4a15dea` | 2026-11-19 | Models are the source: no overwrite unless `Overwrite=true`; `Force=true` alone errors; `modelInterfaces.m` and `checkModels.m` added. |

#### Alternatives / tried and dropped

- *Everything generated* (the state up to `4a15dea`): dropped for the reasons above.
- *Everything hand-made*, including `HPP.slx`: not considered seriously, because the top model must change with N (see A3.2 on why For Each subsystems were not used).

#### Consequences

- Never open `HPP.slx` to "fix" something: the next `buildHPP()` (also run by the `model` task of Run Build) discards it. Change `buildHPP.m` instead.
- When you edit a controller or asset model, keep its ports exactly as in `modelInterfaces.m` and follow the expression rules (A3.10), then run `checkModels`.
- The templates in `buildHPP.m` are now a record of how the models were first created, and a way to recreate a lost model. They are not guaranteed to stay in sync with hand edits (at the time of writing they are: see A3.12).

### A3.2 buildHPP options and the top-level sequence

#### Options

`buildHPP(opts)` takes name-value options (`arguments` block, lines ~44–50):

| Option | Default | Meaning |
|---|---|---|
| `All` | `false` | Also create controller and asset models that do not exist yet, from the templates. Existing ones are kept (`"Kept existing X.slx"` is printed). |
| `Overwrite` | `false` | With `All=true`: regenerate the selected models from their templates, overwriting hand edits. Without `All` it errors (`buildHPP:overwriteAll`). |
| `Only` | empty | With `All`: restrict creation/regeneration to these names (`"HPPC"`, `"WPPC"`, …, or `"WT_asset"`, `"PV_asset"`, `"BESS_asset"`). |
| `Force` | `false` | Older option. Alone it errors (`buildHPP:overwrite`) with a message explaining that it would overwrite hand edits and that the explicit form is `buildHPP(All=true, Overwrite=true, Only=[...])`. With `Overwrite=true` it is accepted but adds nothing. |
| `Strict` | `false` | Stop with an error if `checkModels` finds a problem (otherwise warn). |

Typical calls:

```matlab
buildHPP()                                      % rebuild HPP.slx (the normal case)
buildHPP(All=true)                              % also create missing controller/asset models
buildHPP(All=true, Overwrite=true, Only="HPPC") % regenerate HPPC.slx from its template
```

#### The top-level sequence (function `buildHPP`, lines 1–121)

1. Validate the option combination (`Force` without `Overwrite`, `Overwrite` without `All`).
2. Run `checkModels(Quiet=true)`. If it finds anything, run it again non-quietly so the report is printed (and errors when `Strict=true`), then warn `buildHPP:checkModels` that Rapid Accelerator will fall back to normal mode until the findings are fixed. The build itself continues: a finding does not prevent a normal-mode simulation.
3. Resolve the folders from `currentProject().RootFolder` (`models/`, `models/ctrl`, `models/assets`; created if missing).
4. `p = HPP_parameters()`. The build always uses the default parameters, so `HPP.slx` is built for the default N.
5. Switch off the warning `Simulink:Engine:MdlFileShadowing` (restored by `onCleanup`), because rebuilding a model that already exists on the path triggers it.
6. If `All=true`: loop over `controllerSpecs()` (controllers first, because asset models reference the asset controllers) and build each missing (or, with `Overwrite`, each selected) controller with its template: `buildHppcCtrl`, `buildWTCtrlType4A`, `buildPlantCtrlRepc` (WPPC, SPPC, BESSC), `buildReecCtrl` (PV_ctrl, BESS_ctrl). The generic `buildController` is the fallback branch, which no current name reaches. Then the asset models with `buildAsset("WT" | "PV" | "BESS")`.
7. `buildTop(p, dirs.top)`: generate and save `HPP.slx`.

New model files are added to the MATLAB project by `registerInProject` (that is where the `resources/project/*.xml` entries in some commits come from).

#### Why N instances are made by the script and not by a For Each subsystem

`docs/design.md` §3 ("N assets"): a For Each subsystem was not used because the build script is simpler to debug and every instance stays a normal Model block, which code generation and PSCAD handle the same way. Each instance k receives its own argument `p.<type>.ap(k)`, so per-instance parameters (different turbine types in one plant) need only different values, not a different model (design §9).

Consequence: changing N (via `p.<type>.nFeeders` / `nPerFeeder`) requires `buildHPP()`. Case sets cannot change N; `checkModelBuild` stops a run whose `p` has a different N from the one `HPP.slx` was built for (A3.11).

### A3.3 buildTop: the top level of HPP.slx

#### Configuration set by the build

`buildTop` (lines ~233–332) creates a new system `HPP` and sets:

| Setting | Value | Why |
|---|---|---|
| `SolverType`, `Solver` | Fixed-step, `ode1` | One derivative evaluation per step. The plant has few continuous states (converter current lags, SOC integrator); `ode1` instead of `ode4` was part of a ~15 % speed gain with steady state unchanged and dip values within 0.7 MW (design §11). |
| `FixedStep` | `p.sim.Ts` (1 ms) | The base step of plant and asset controllers; all controller sample times must be integer multiples of it (asserted in `controllerParameters`). This mirrors PSCAD, where a controller DLL runs at a multiple of the EMT step (`simulationParameters.m`). |
| `SolverMode` | `SingleTasking` | All rates run in one task; rate transitions between 1 ms, 20 ms and 100 ms need no extra rate-transition blocks for determinism. (inferred: the reason is not written down; single-tasking is the simple, deterministic choice for a simulation-only top model.) |
| `Decimation` | `p.sim.logDecimation` (10) | Log every 10th step (10 ms) to save memory and time (design §11). |
| `StopTime` | `p.sim.Tstop` (60 s) | Overridden per run by `runCase` / `runSweep`. |
| `SaveOutput`, `SignalLogging` | on | Root outports are saved to `out.yout` (the 17 logged signals). |
| `AlgebraicLoopMsg` | `error` | Any algebraic loop stops compilation instead of being solved iteratively. This makes the one-step memories (A3.5) mandatory and keeps every step a fixed amount of work. |

The parameter names in these settings (`p.sim.Ts`, …) are resolved when the model compiles: from the base workspace (where `projectStartup` puts the default `p`) for editor runs, and from the `SimulationInput` variable `p` for `runCase` / `runSweep`.

#### Subsystems and layout

The top level has six subsystems, left to right: `TSO`, `HPP controller`, `Wind plant`, `PV plant`, `BESS plant`, `Network`, `Logging`, followed by 17 root outports. They are built by:

| Block | Builder |
|---|---|
| `TSO` | `buildTSO` |
| `HPP controller` | `buildHppcSub` |
| `Wind plant`, `PV plant`, `BESS plant` | `buildPlantSub(sys, p, plants(i), i, offset)` with a running `offset` of global asset indices (wind assets first, then PV, then BESS) |
| `Network` | `buildNetworkVariants` (Variant Subsystem with `Nodal` = `buildNetwork` and `Simscape` = `buildNetworkSimscape`) |
| `Logging` | `buildLogging` |

The plants are described by a small struct array: title, tag (`wpp`, `spp`, `bess`), controller (`WPPC`, `SPPC`, `BESSC`), parameter field (`wppc`, …), asset type (`WT`, `PV`, `BESS`) and type field (`wt`, `pv`, `bess`).

Positions are set before any line is drawn, aligned to ports so that the forward lines are straight: the HPP controller has its three set-point outputs `D = 150` pixels apart; each plant is placed so its input 1 is level with HPPC output i; the Network is level with the current output of the wind plant; TSO is level with HPPC input 1; Logging is sized to 36 px per logged signal.

Background colours by role (`areaColor`): controllers light blue `[0.894 0.906 0.988]`, assets/plants light green `[0.878 0.965 0.925]`, interfaces and network light grey `[0.95 0.95 0.95]`.

#### Wiring: lines forward, Goto/From backward

Forward chain as lines (`ln`): TSO → HPPC input 1 (named "TSO"), HPPC output i → plant i input 1 ("set-points wpp", …), plant i output 2 → Network input i ("I wpp", …).

Feedback and logging as colour-coded Goto/From pairs (`gotoFrom`), DTU palette:

| Tag | Source | Destinations | Colour |
|---|---|---|---|
| `meas_wpp`, `meas_spp`, `meas_bess` | plant output 1 (measurement bus) | HPPC inputs 3, 4, 5; Logging 6, 7, 4 | orange (plant measurements) |
| `network` | Network output 1 | inputs 2 of all plants; Logging 1 | green (network) |
| `POC_meas` | Network output 2 | HPPC input 2; Logging 2 | green |
| `tso` | TSO output 1 | Logging 5 | grey (logging only) |
| `setpoints_bess` | branch of the BESS set-point line | Logging 3 | grey |

The BESS set-point goto is placed by hand just above the set-point line, because below it sits the plant's network From.

**Why.** The diagram must be readable at a glance: forward lines show the control chain, and backward lines that would cross the diagram are replaced by tagged pairs. The reorganisation in `6c04aa2` changed no results (largest relative difference 6e-13 on all logged signals). All Goto blocks are `TagVisibility = local`, so tags never leak between subsystems.

**Buses.** The six subsystems are connected by virtual buses in SI units (W, var; voltages in pu). Each subsystem converts to its controller's per-unit base inside. The controller models themselves keep flat scalar (or fixed-width vector) ports, the future DLL interface; Bus Selectors and Bus Creators sit only in the wiring around them (design §3, "Diagram layout").

#### Model workspace of HPP.slx

At the end, `buildTop` writes three variables into the model workspace and saves the model:

| Variable | Value | Used by |
|---|---|---|
| `buildInfo` | struct `N_wt`, `N_pv`, `N_bess`, `built` (timestamp) | `checkModelBuild` (A3.11) |
| `HPP_network` | `networkCode(p.net.solverType)`: 1 nodal, 2 Simscape | variant control of `Network`, for runs from the editor; `runCase` / `runSweep` / `rapidTarget` override it per run |
| `HPP_delays` | code from `delayMode(p, table())`: 1 fixed, 2 variable | variant control of every delay in the link blocks; overridden per run likewise |

#### Root outports and the logged signals

`logSignalNames()` (line ~584) defines the 17 logged signals in outport order; `plotAll…` functions read `out.yout` by these names:

| # | Name | Unit | Source |
|---|---|---|---|
| 1 | `P_poc` | W | POC bus (power into the grid) |
| 2 | `SOC` | – (0–1) | BESS measurement bus, `SOC_plant` (share-weighted mean of the asset SOCs) |
| 3 | `P_bess_set` | W | BESS set-point bus from the HPPC, `P_set` |
| 4 | `Q_poc` | var | POC bus |
| 5–7 | `P_wpp`, `P_spp`, `P_bess` | W | network bus `P_plant` (flow of each plant into the 33 kV busbar), demuxed |
| 8–10 | `Q_wpp`, `Q_spp`, `Q_bess` | var | network bus `Q_plant`, demuxed |
| 11 | `V_poc` | pu | POC bus |
| 12 | `V_bus` | pu | network bus (33 kV HPP busbar) |
| 13 | `P_ref` | W | TSO bus |
| 14 | `Q_ref` | var | TSO bus |
| 15 | `f` | pu (of 50 Hz) | TSO bus `f_meas` |
| 16 | `P_avail_wpp` | W | wind measurement bus `P_avail` |
| 17 | `P_avail_spp` | W | PV measurement bus `P_avail` |

Each root outport is placed level with its Logging output port.

### A3.4 The subsystem builders

#### buildTSO

Creates `TSO` with three sources into a Bus Creator `TSO bus` and one outport `TSO`:

| Element | Block | Parameters |
|---|---|---|
| `P_ref` [W] | Step | before `p.grid.Pref`, after `p.grid.Pref + p.grid.Pstep.dP`, at `p.grid.Pstep.t0` |
| `Q_ref` [var] | Step | before `p.grid.Qref`, after `p.grid.Qref + p.grid.Qstep.dQ`, at `p.grid.Qstep.t0` |
| `f_meas` [pu] | Step | before 1, after `1 + p.grid.fev.df`, at `p.grid.fev.t0` |

All with `SampleTime = 0` (continuous source, sampled by the receiving blocks). These are the "TSO/set-point and event sources". The voltage dip and the wind availability step live elsewhere: dips in the network variants (grid source E), the wind step in the wind plant subsystem. Note: the comment in `buildTSO` still says "constants for now" and keeps a `vals` array with `"0"` for `Q_ref`; both are leftovers, since all three are now Step blocks.

#### buildHppcSub

Builds `HPP controller`: the Model block `HPPC` (ModelName `HPPC`, instance argument `par = p.ctrl.hppc.par`), five bus inports and three set-point bus outports.

Inputs are grouped per source, one group per bus inport (cell array `G`, lines ~901–907):

| Inport | Bus elements | HPPC ports | Link steps |
|---|---|---|---|
| 1 `TSO` | `P_ref`, `Q_ref`, `f_meas` | 1–3 | P, Q: `gain:1/Pbase`, `delay:p.comm.tso@<initial set-point in pu>`; `f_meas`: no link (measured locally, no delay) |
| 2 `POC meas` | `P_poc`, `Q_poc`, `V_poc` | 4–6 | P, Q: `gain:1/Pbase`, `memory`, `delay:p.comm.meas_hppc`; V: `memory`, `delay:p.comm.meas_hppc@1` |
| 3 `meas wpp` | `P_avail`, `P`, `Q`, `Q_sat` | 7–10 | meas (gain, memory, delay); `Q_sat`: memory, delay (no unit) |
| 4 `meas spp` | same | 11–14 | same |
| 5 `meas bess` | `P`, `Q`, `P_chg_max`, `P_dis_max`, `SOC`, `Q_sat` | 18, 19, 15, 16, 17, 20 | meas for P, Q, limits; flag (memory, delay) for SOC and `Q_sat` |

The BESS bus order (P, Q first) differs from the HPPC port order (limits first), so the code maps it explicitly (`ports = [18 19 15 16 17 20]`). Each Bus Selector is sized to span the HPPC ports it feeds, and its outputs are sorted by HPPC port so no lines cross. Each link block sits on the line of its port (`rowInto`). The link's label is the HPPC port name, taken from `modelInterfaces().HPPC.in`, so the wiring and the interface definition use the same list.

The TSO delays start at the initial set-point (`@p.grid.Pref/p.ctrl.hppc.Pbase`, `@p.grid.Qref/...`), so the HPPC does not see a zero reference at start-up (design §6). `V_poc` delays start at 1 pu for the same reason.

Outputs: per plant a two-input Bus Creator (`P_set`, `Q_set`) fed by links `gain:p.ctrl.hppc.Pbase` (pu → W / var), then an outport `set-points wpp|spp|bess`.

#### buildPlantSub

Builds one plant subsystem (lines ~963–1165). Inputs: 1 set-point bus (W/var) from the HPPC, 2 network bus. Outputs: 1 measurement bus (W/var) to the HPPC, 2 current injections (system pu) to the network. The diagram is divided into four background areas aligned in a grid (`addArea`, `alignAreaGrid`):

```
[ plant controller (future DLL)        ] [ outputs to HPPC ]
[ bus inputs ] [ N × <type> asset       ] [ and network     ]
```

Steps:

1. **Plant controller.** Model block `<C>` (WPPC/SPPC/BESSC), instance argument `par = p.ctrl.<c>.par`. Inputs 7 (8 for BESSC), outputs 6 (9 for BESSC).
2. **Bus inputs → Goto.** Selectors for the set-point bus (`P_set`, `Q_set`) and network bus (`V_re`, `V_im`, `P_plant`, `Q_plant`, `V_bus`, `f`), each element to a local Goto (blue set-points, green network).
3. **Into the plant controller**, each row level with its port (`rowInto`):

| Port | Row | Link steps |
|---|---|---|
| 1 `P_set`, 2 `Q_set` | From → link | `delay:p.comm.hppc_to_plant`, `gain:1/p.ctrl.<c>.Pbase` |
| 3 `P_meas`, 4 `Q_meas` | From `P_plant`/`Q_plant` → Selector (index = plant number of 3) → link | `gain:1/Pbase`, `memory`, `delay:p.comm.meas_plant` |
| 5 `V_meas` | From `V_bus` → link | `memory`, `delay:p.comm.meas_plant@1` |
| 6 `P_avail_k` (N) | From → link | `gain:<asset Pbase>/<plant Pbase>`, `memory`, `delay:p.comm.meas_plant` |
| 7 `SOC_k` (BESSC, N) | From → link | `memory`, `delay:p.comm.meas_plant` |
| last `f_meas` | From `f` → link | `memory` only (measured locally) |

4. **Out of the plant controller:** `P_set_k`, `Q_set_k` → link (`delay:p.comm.plant_to_asset`, `gain:<plant Pbase>/<asset Pbase>`) → Goto.
5. **N asset instances**, one row each, under the controller (the plant grows downward with N, `rowH = 220` px per asset). For instance k with global index g = offset + k:
   - Model block `"<TYPE> k"`, ModelName `<TYPE>_asset`, instance argument `ap = p.<t>.ap(k)`.
   - Inputs 1–4 from From blocks via Selectors: `P_set_k(k)` and `Q_set_k(k)` (width N), `V_re(g)` and `V_im(g)` (width nA = total asset count).
   - Input 5 (WT, PV): the primary source. Wind: Step `p.wt.Pavail0` → `p.wt.PavailStep` at `p.wt.tStep` (the wind step at 20 s). PV: Constant `p.pv.Pavail`. The BESS has no input 5.
   - Outputs: `P`, `Q` to Terminators (the plant measurement comes from the network, not from the assets); `P_avail`, `I_re`, `I_im` (and `SOC`) to Gotos tagged `<name>_<k>`.
6. **Right column:** per output a Mux of N Froms (`P_avail_k`, `I_re_k`, `I_im_k`, `SOC_k`); `P_avail_k` and `SOC_k` go back to Gotos for the controller inputs; `I_re`, `I_im` pass a link `gain:<asset Pbase>/p.grid.Pbase` (asset pu → system pu) into a Bus Creator `injection` → outport `I`.
7. **Measurements to the HPPC:** controller outputs (WPPC/SPPC: ports 3–6 `P_avail`, `P_plant`, `Q_plant`, `Q_sat`; BESSC: ports 4–9 `P_plant`, `Q_plant`, `P_chg_max`, `P_dis_max`, `SOC`, `Q_sat`, with `P_avail` terminated) pass links `gain:<plant Pbase>` (to W), except `SOC` and `Q_sat` (no unit), into the Bus Creator `meas bus`. Bus element names come from line names: wind/PV `P_avail`, `P`, `Q`, `Q_sat`; BESS `P`, `Q`, `P_chg_max`, `P_dis_max`, `SOC`, `Q_sat`, `SOC_plant`.
8. **BESS only:** the extra element `SOC_plant` = Gain `p.ctrl.bessc.par.share'` (matrix multiply) on `SOC_k`, continuous and undelayed, for logging. It makes all three plants have the same two outputs.

Local Goto/From colours in the plants: blue set-points, green network, orange asset signals back to the plant controller.

#### buildLogging

Picks the logged signals out of the buses (lines ~502–582), one straight row per signal: Bus Selectors on `POC meas` (P, Q, V), `set-points bess` (`P_set` → `P_bess_set`), `meas bess` (`SOC_plant` → `SOC`), `network` (`P_plant`, `Q_plant` demuxed into three each, `V_bus`), `TSO` (`P_ref`, `Q_ref`, `f_meas` → `f`), `meas wpp` and `meas spp` (`P_avail` → `P_avail_wpp`, `P_avail_spp`). Ports are created in row order and then renumbered explicitly with `renumberPorts`: inports in the order `network, POC meas, set-points bess, meas bess, TSO, meas wpp, meas spp`, outports in `logSignalNames()` order. (Simulink shifts existing port numbers when a port number is requested out of order, which is why renumbering is a separate step.)

### A3.5 Link blocks

#### What

Every signal path between control levels in `HPP.slx` is one masked subsystem, a "link", made by `makeLink(sys, label, steps)` (line ~1313). `steps` is a string array of steps applied in series:

| Step | Block | Mask label |
|---|---|---|
| `gain:<expr>` | Gain with that expression | `to pu` if the expression starts with `1/` (SI → pu); `pu base` if it contains another `/` (one pu base → another); otherwise `to SI` (pu → W/var) |
| `memory` | Memory (one-step memory) | `mem` |
| `delay:p.comm.<name>[@<initial output>]` | Variant Subsystem from `buildDelayVariant`, length `p.comm.steps.<name>` | `comm.<name>` |

The mask icon displays the labels joined by `|`, e.g. `to pu | mem | comm.meas_plant`; the mask description spells out every step ("Link: gain 1/p.ctrl.hppc.Pbase, then one-step memory, then delay …"). The block name `<label> link` is hidden: the icon says what the link does, and the port it feeds says which signal it carries. The current `HPP.slx` has 69 link blocks (counted in the saved file: 16 `to SI`, 12 `to pu | mem | comm.meas_hppc`, 6 `comm.hppc_to_plant | to pu`, 6 `comm.plant_to_asset | pu base`, 6 `to pu | mem | comm.meas_plant`, 6 `pu base`, 5 `mem | comm.meas_hppc`, 4 `mem | comm.meas_plant`, 3 `pu base | mem | comm.meas_plant`, 3 `mem`, 2 `to pu | comm.tso`) containing 44 delay variants.

`chain(sys, src, steps, dst, label, outName)` is a helper that makes a link and connects it without alignment; it is not used by the current builders.

#### Why links

- One block per path makes the diagram compact and makes the three effects of a real communication link (unit conversion, sampling memory, delay) visible in one place.
- Unit conversions sit outside the controllers on purpose: controllers work in per unit on their own base, and in PSCAD the same conversions belong on the PSCAD side of the DLLs (README, design §4).

#### Why every measurement path has a one-step memory

The controllers have direct feedthrough (a measurement can affect the output in the same step, e.g. through the proportional part of a PI or a feed-forward). Without a memory, a loop such as HPPC → plant controller → asset → network → plant measurement → plant controller is an algebraic loop whenever a communication delay is 0 (design §6: "without it, the loops through the controllers' direct feedthrough are algebraic loops when a delay is 0"). With `AlgebraicLoopMsg = error`, such a loop would stop compilation; with iterative solving it would cost time and break the fixed cost per step.

The memory costs one simulation step (1 ms) of extra measurement latency. Stage 1 measured its effect: with delays set to 0, results matched the earlier single-controller model except that the HPPC reacted one sample (0.1 s) later at start-up and at the wind step (design §8, stage 1).

Set-point paths (HPPC → plant → asset) have no memory: they are forward paths, and the loop is already broken on the measurement side. The local frequency measurement `f_meas` into the plant controllers has a memory but no delay. The TSO set-points have no memory (they come from sources, not from a loop).

The network and asset models have their own one-step memories as well: the network returns the asset terminal voltages one step old (`V_re prev`, `V_im prev`, a generator "PLL"), and each asset model passes its own P and Q to its controller through Memory blocks (`P meas`, `Q meas`).

#### The delay as a Variant Subsystem

`buildDelayVariant(vs, stepsField, init)` (line ~1370) creates a Variant Subsystem with ports `u`, `y` and two choices:

| Choice | Condition | Implementation |
|---|---|---|
| `Fixed` | `HPP_delays == 1` | Delay block, `DelayLength = p.comm.steps.<name>`, initial condition `init`, sample time `p.sim.Ts`. The length is fixed at compile time. |
| `Variable` | `HPP_delays == 2` | Delay block with `DelayLengthSource = Input port`, `DelayLengthUpperLimit = round(p.comm.maxDelay/p.sim.Ts)`, fed by a Constant `p.comm.steps.<name>` (double, sample time `p.sim.Ts`). The length is a tunable input. |

`VariantActivationTime = "update diagram"`: only the active choice is compiled, so the fixed choice costs nothing for having a variable sibling. The choice ports are renamed `u` and `y` because a Variant Subsystem matches choice ports to its own ports by name. To empty the default Variant Subsystem, the code deletes its blocks using `find_system(..., "MatchFilter", @Simulink.match.allVariants)` (without the filter, inactive choices are not found).

`p.comm.steps.<name> = round(p.comm.<name> / p.sim.Ts)` is computed in `deriveParameters`, which asserts that every delay lies between 0 and `p.comm.maxDelay` (0.5 s). Which choice is active is decided per run by `delayMode(p, cases, override)` from `p.comm.delays`: `"fixed"`, `"variable"` or `"auto"` (variable when the case set changes a `comm.*` column, otherwise fixed). The receiving controller samples the delayed signal at its own rate.

**Why.** Delay sweeps (case set `comm_delay`) need delays that change between runs without recompiling, but variable-length delays are slower. Both choices exist, and each run picks the cheaper one it can use.

**Tried and dropped / history.**
- Stage 1 used continuous Transport Delay blocks. They were replaced by discrete delays of whole steps (design §11).
- Variable-length delays were first tried and dropped in `fb180ee` because they made every run 0.5–0.75 s per simulated second slower in normal mode. They came back in `ac18a90` (2026-11-19) as a *choice* next to fixed delays. Measured in Rapid Accelerator: about 1.5–1.7 s per 60 s case with fixed delays, 2.2–2.6 s with variable delays, same results.
- With fixed delays, every new set of delays needs a recompile or Rapid rebuild; switching modes rebuilds the Rapid target (about 30 s, design §13).

**Consequences.** A new path with a delay must use `makeLink` with a `delay:p.comm.<name>` step, and `p.comm.<name>` must exist in `commParameters` so `deriveParameters` produces `p.comm.steps.<name>`. The delay mode and (when fixed) the delays are part of the compile signature used by `runCase`, `runSweep` and `rapidTarget` (chapter A5).

### A3.6 The Network variant subsystem

The physics of both network models is in B2. This section covers only the code structure.

#### buildNetworkVariants (line ~2410)

Creates `Network` as a Variant Subsystem (emptied with the `allVariants` filter as above), with three inports `I wpp`, `I spp`, `I bess` and two outports `network`, `POC meas`. Choices:

| Choice | Condition | Builder |
|---|---|---|
| `Nodal` | `HPP_network == 1` | `buildNetwork` |
| `Simscape` | `HPP_network == 2` | `buildNetworkSimscape` |

`VariantActivationTime = "update diagram"`: the inactive choice is not compiled. `networkCode(name)` (`scripts/networkCode.m`) maps `"nodal"` → 1 and `"simscape"` → 2. `runCase(..., Network=...)`, `runSweep(..., Network=...)` and `main.m` set `HPP_network` per run through `SimulationInput.setVariable(..., "Workspace", "HPP")`, so no rebuild is needed to switch. Both choices have the same ports and output buses:

- network bus: `V_re`, `V_im` (all asset terminals, one step old), `P_plant`, `Q_plant` (3 × 1, W / var), `V_bus` (pu), `f` (pu, Step from `p.grid.fev`);
- POC bus: `P_poc` (W), `Q_poc` (var), `V_poc` (pu).

**Why a variant and not two models.** Both models are built from the same layout parameters; the Simscape one is the component-wise reference, the nodal one is fast (design §5). A variant keeps one `HPP.slx` and lets the choice be a run option. The network sits inside `HPP.slx` rather than in its own file, since its size depends on N and it is not a DLL (design §3).

#### buildNetwork: the nodal solver at the code level (line ~367)

`Nodal` is a masked subsystem with mask parameter `net` (prompt "Network solver data", value `p.net.solver` from `networkSolverData`, evaluated, tunable). Three areas, laid out with `ctrlSection`/`rowChain` like the controllers:

1. *Current injections and grid source (dip):* each input bus is split into `I_re`, `I_im`; Muxes collect `I_re all`, `I_im all` (3 elements, each the vector of one plant's assets); the grid voltage is `E grid = E0 − dip start + dip end`, with `E0 = net.E` and two Steps of size `p.grid.dip.depth` at `p.grid.dip.t0` and `t0 + dur`.
2. *Network solution:* a MATLAB Function block `solve` (`addMatlabFunction`) with inputs `I_re`, `I_im`, `E`, `Zre`, `Zim`, `Zg_ri`, `brZre`, `brZim`, `brB` and parameter `net`. It computes `V = Z * I` with the grid as a Norton source at the POC, the branch flows of each plant into the busbar, and the POC power into the grid.
3. *Output buses:* Memory blocks `V_re prev` (initial `ones(net.nA,1)`) and `V_im prev` (initial zeros); gains `net.Sb` from system pu to W/var; Step `f grid`; Bus Creators named via the line names (`nameInputs`).

The key mechanism for Fast Restart (`fb180ee`): the *structure* (node indices `net.poc`, `net.bus`, `net.brFrom`, `net.brPlant`, `net.nA`) is a non-tunable parameter of the MATLAB Function (the code sets `Tunable = false` on the Stateflow data `net`), while the *numbers* (`net.Zre`, `net.Zim`, `[net.Zg_re, net.Zg_im]`, `net.brZre`, `net.brZim`, `net.brB`) enter as tunable Constant blocks (`kc`). So grid strength, cable and transformer data change without recompiling; only the node structure is fixed. Tried and dropped: the whole network data as one tunable parameter struct, which made every run 0.5–0.75 s per simulated second slower (design §11).

#### buildNetworkSimscape at the code level (line ~2439)

Builds the same network from Simscape Electrical three-phase blocks, laid out row by row from `p.<type>.nFeeders × nPerFeeder`:

- **Inputs:** bus selectors per plant, Demuxes into one `Ire_k`, `Iim_k` per asset (when N > 1), and a Digital Clock `t` at `p.sim.Ts` (time for the phasor transforms).
- **One row per converter:** `convBlock` (Controlled Current Source (Three-Phase) driven by the current phasor's rms magnitude and angle, a Phase Voltage Sensor and the space-phasor transform back to `V_re`, `V_im`) → `unit Z k` (RLC (Three-Phase), series R-L from the unit transformer data referred to the collector voltage, scaled by `nAgg`) → `feeder k` (`piBlock`: series R-L and half the charging capacitance at each end, R, L, C divided or multiplied by `cable.nPar`). Electrical nodes between rows are joined by Connection Labels (`nodeLabel`, `tapLabel`: `tap_k`, `<tag>_collector`, `HPP_bus`, `POC`) rather than drawn lines.
- **Plant column:** optional plant transformer (`trBlock`, wind only, since `p.net.pv.plant.S` and `p.net.bess.plant.S` are 0), `meterBlock` (Power Sensor (Three-Phase), P and Q in W/var to Gotos `P_<tag>`, `Q_<tag>`).
- **Grid column:** grid voltage magnitude `E0 = p.grid.E` with the same dip Steps, scaled to volts by `p.grid.Vpoc`; HPP transformer 33/132 kV (`trBlock`), POC meter, `grid Zg` (series R-L from SCR and X/R), `grid E` (Controlled Voltage Source (Three-Phase), single-frequency AC); voltage meters for `V_bus`, `V_poc` (`vMeterBlock`); a Solver Configuration with `EquationFormulation = NE_FREQUENCY_TIME_EF` (frequency-and-time mode), local Backward Euler solver at `p.sim.Ts`, fixed cost, one nonlinear iteration.
- **Outputs:** Muxes of all terminal voltages, Memory blocks for the one-step-old voltages, the same two output buses.

Implementation details worth knowing:

- `eeLibrary()` builds a lookup from `ee_lib` block paths with line breaks replaced by spaces, because many library paths contain newlines. `load_system("ee_lib")` is called first.
- R, L, C and transformer data are set as **run-time** parameters (`R_conf = "runtime"` etc.; `trBlock` sets `<param>_conf = "runtime"` for all its ratings), so they change without recompiling in normal mode with Fast Restart.
- The transformer blocks include the magnetizing branch (`Rm`, `Xm`), needed in frequency-and-time mode; the nodal model has the same branch as a shunt (B2).
- The library Transmission Line (Three-Phase) block is avoided (a non-physical 100 Hz power ripple in this mode, comment in `piBlock` and design §5); unit transformers are series R-L because real transformer blocks made the network 3.5 times slower (design §5).
- The Simscape network does not work in Rapid Accelerator: its data and grid source are fixed when the program is built, so `rapidTarget` refuses it and runs it in normal mode (design §13).

**Consequence for layout changes.** The Simscape choice is drawn for one specific feeder layout. Changing `nFeeders` / `nPerFeeder` therefore needs `buildHPP()` even if N stays the same (e.g. PV 2 × 1 → 1 × 2). `checkModelBuild` compares only N, so it would not catch this (inferred from the code: `buildInfo` stores only the three N). The nodal choice reads its node structure from `p.net.solver` at compile time and is not affected.

### A3.7 Model references, sample times and model arguments

#### What a referenced model is

A Model block in `HPP.slx` refers to another `.slx` file (a *referenced model*). The referenced model is a complete model on its own: own configuration, own model workspace, own sample time. It is compiled separately (in normal mode it is interpreted, in accelerator and Rapid Accelerator modes compiled to code) and can be instantiated many times. `HPP.slx` contains 9 Model blocks in the default layout (HPPC, WPPC, SPPC, BESSC, 2 × `WT_asset`, 2 × `PV_asset`, 1 × `BESS_asset`, all in `SimulationMode = Normal`), and each asset model in turn contains one Model block for its asset controller.

#### Why each controller is its own .slx

- **Future DLL.** Each controller will be code-generated separately and wrapped as an IEEE/CIGRE DLL for PSCAD (design §1: all seven controllers become DLLs; they are our own models, not vendor DLLs). A referenced model is the natural unit for code generation.
- **Own rate.** Each controller runs at its own fixed sample time, as a real controller would, and as the DLL will in PSCAD.
- **Own per-unit base and parameter struct**, so the generated code is self-contained.
- **Plant side not code-generated.** The asset models (generator, converter, battery) are only for Simulink; in PSCAD they are replaced by PSCAD's own EMT models (design §2). They are separate files so that one parameterised file can be instantiated N times.

#### Sample times per level

| Level | Model | `Ts` | Source of `Ts` |
|---|---|---|---|
| HPPC | `HPPC.slx` | 0.1 s | model workspace: `pDefault.ctrl.hppc.Ts` |
| Plant controllers | `WPPC`, `SPPC`, `BESSC` | 20 ms | `pDefault.ctrl.<c>.Ts` (`plantCtrl` in `controllerParameters`) |
| Asset controllers | `WT_ctrl`, `PV_ctrl`, `BESS_ctrl` | 1 ms (= `p.sim.Ts`) | `pDefault.ctrl.<t>.Ts` (`assetCtrl(p.sim.Ts, …)`) |
| Asset models | `<TYPE>_asset` | inherited (inports `SampleTime = -1`); configuration step `Tsim = pDefault.sim.Ts` | model workspace |
| Top model, links, network | `HPP.slx` | `p.sim.Ts` = 1 ms | `p` |

The controller models are configured `FixedStepDiscrete` with `FixedStep = Ts`; their inports have `SampleTime = Ts`, so the whole model runs at `Ts`. The asset models are configured `ode4` with `FixedStep = Tsim` for standalone use; inside `HPP.slx` the top model's solver (`ode1`) integrates their continuous states (inferred: in a model-reference hierarchy the top model's solver is used). `controllerParameters` asserts that every `ctrl.<c>.Ts` is an integer multiple of `sim.Ts`.

The rates and delays are typical values (design §6). Why 100 ms / 20 ms / 1 ms: a slow coordinating controller, faster vendor plant controllers, and asset controllers at the simulation step as a stand-in for the converter control (design §2, §6; the exact values are typical, not from a specific project).

#### Model arguments, instance arguments and model workspace defaults

*Model argument.* A variable in a referenced model's workspace that is declared as an argument (`ParameterArgumentNames`). Each Model block instance can then give it its own value (an *instance argument*, set in the block's `InstanceParameters`). In generated code it becomes a per-instance parameter; in this project it is meant to become the parameter list of the IEEE/CIGRE DLL (commit `467b5f2`).

| Model | Argument | Instance value set by `buildHPP` | Default in the model workspace |
|---|---|---|---|
| `HPPC` | `par` | `p.ctrl.hppc.par` | `pDefault.ctrl.hppc.par` |
| `WPPC`, `SPPC`, `BESSC` | `par` | `p.ctrl.wppc.par`, `.sppc.par`, `.bessc.par` | `pDefault.ctrl.<c>.par` |
| `WT_ctrl`, `PV_ctrl`, `BESS_ctrl` | `par` | `ap.ctrl` (inside the asset model) | `pDefault.ctrl.<t>.par` |
| `WT_asset`, `PV_asset`, `BESS_asset` | `ap` | `p.<t>.ap(k)` for instance k | `pDefault.<t>.ap(1)` |

`setArg(blk, argName, value)` sets the instance argument: it reads `InstanceParameters`, asserts the argument exists, sets its value and writes it back. Before that, `Simulink.ModelReference.refresh(blk)` loads the referenced model's interface so the block knows its ports and arguments.

*Model workspace defaults.* Every controller and asset model has a model workspace with `DataSource = "MATLAB Code"`. The code (written by `ctrlHeader`, `buildController`, `buildAsset`) runs `HPP_parameters()` and copies out the defaults, for example in `HPPC.slx`:

```matlab
pDefault = HPP_parameters();
Ts  = pDefault.ctrl.hppc.Ts;    % [s] fixed sample time
par = pDefault.ctrl.hppc.par;   % tunable parameters (model argument)
clear pDefault
```

Plant controllers add `N = numel(par.share);` (their vector port width). Asset models define `ap = pDefault.<t>.ap(1)` and `Tsim = pDefault.sim.Ts`.

**Why.** Each model must also run and generate code on its own, without `HPP.slx` or a base-workspace `p` (commit `467b5f2`: "Sample time Ts and the standalone default par come from the model workspace, computed from HPP_parameters(), so the controller also runs and generates code on its own"). The defaults also give the argument its size and type, which Simulink needs to compile the model.

**Asset arguments nest the controller's.** `p.<t>.ap(k)` (built in `deriveParameters`) is a struct with fields `ctrl` (the asset controller's `par`) and `gen` (generator/converter data); for the BESS also `SOC0` and `kSOC`. The asset model passes `ap.ctrl` on to its controller's `par`. So one struct per instance carries everything that instance needs, and different values per instance only need different `ap(k)`.

#### Consequences

- `par` and `ap` are tunable: they change between runs without recompiling, including in Rapid Accelerator, as long as block parameters that use them follow the expression rule (A3.10).
- `Ts`, the per-unit bases and the plant controllers' `N` are **not** arguments. They come from `HPP_parameters()` defaults through the model workspace (design §4: "base and `Ts` not tunable"). A case set that changes `ctrl.hppc.Ts` would not change the HPPC's rate (inferred from the code: `Ts` is read from `pDefault`, not from the run's `p`). To change a rate, change the default in `controllerParameters`.
- The plant controllers' port width `N` follows `numel(par.share)` of the defaults, so it updates by itself when the default N changes; the DLL is built for one N (design §3).
- `docs/design.md` §3 says that changing N "means rerunning `buildHPP(All=true)`". From the code, `buildHPP()` is enough, because the controllers' `N` is recomputed from the defaults whenever the model workspace is loaded; `All=true` would only create missing models (inferred). The `buildHPP.m` header and README say `buildHPP`.

### A3.8 The controller and asset templates in buildHPP.m

#### The common pattern

Every controller follows the interface pattern of design §4: per unit on its own base, all ports scalar or fixed-width vectors of `double`, tunable parameters in one struct `par` (model argument), fixed sample time `Ts`. W↔pu conversions are outside, in `HPP.slx`.

**The original "logic" pattern (stage 1).** `controllerSpecs()` and `buildController(s, folder)` (lines ~126–218, ~701–745) build a controller as: inports (double, given dimensions, sample time `Ts`) → **one MATLAB Function block `logic`** with the parameter `par` → outports. `addMatlabFunction` writes the function signature `function [outs] = logic(ins, par)`, adds `%#codegen`, and sets the Stateflow data `par` to scope `Parameter`, so it is resolved from the model workspace (the model argument). Ports are placed level with the block's ports by `rowInto`/`rowFrom`. This is the pattern the `buildHPP.m` header still describes ("Each controller: inports -> one MATLAB Function block "logic" -> outports, all double, per unit on its own base").

**The current pattern (stages 3–5).** All seven controllers have since been replaced by block-diagram templates. They keep the same interface rules but implement the standard model structures (IEC, WECC) as visible block diagrams, with MATLAB Function blocks only for logic that is awkward in blocks (current limitation, dispatch, ramp limiter with state, flags). `controllerSpecs()` is still used for the list and order of controller names; its placeholder bodies and `buildController` are no longer reached (all names have dedicated builders). The section heading "Controller definitions (stage 1: placeholder logic)" is a leftover.

#### Layout helpers for controller diagrams

Every controller and asset model is drawn in three columns of areas (comment block at line ~1426):

```
inputs (Inport -> Goto) | function areas, stacked | outputs (From -> Outport)
```

| Helper | Role |
|---|---|
| `ctrlHeader(mdl, c, desc, withN)` | New model: `FixedStepDiscrete`, `FixedStep = Ts`, `StopTime = inf`, description; model workspace code with `Ts`, `par` (and `N`); `ParameterArgumentNames = "par"`. |
| `ctrlInputs(sys, names, dims, cm, ts)` | One row per input: Inport (double, dims, sample time `ts`, default `Ts`) → Goto; returns the area and the x of the next column. |
| `ctrlOutputs(sys, names, srcs, x, cm)` | One row per output: From (or an existing block) → Outport (double). |
| `stg(blk, ip, sides, outs)` | One stage of a row: block, main input port, side inputs `{port, src}` (`"@Tag"` = a From, otherwise a constant block placed left of the port), extra outputs `{port, tag}` to Gotos. `"@Tag"` / `">Tag"` as `blk` create a row-start From / row-end Goto. |
| `rowChain(sys, x, yTop, stages, cm)` | Places a row: each block's main input level with the previous block's output; side blocks directly left of their ports; then shifts the row to `yTop` and draws straight lines. |
| `ctrlSection(sys, title, rows, x, y, cm, color)` | Stacks rows into a titled function area; returns the next y, the area handle and the right edge. |
| `areaAround`, `alignColumns` | Area annotation around blocks; aligns the input/output areas to full height and the function areas of each column to common edges. |
| `tagColor(cm, tag)` | Colour of a Goto/From by its tag: blue (`cm.ref`: references, orders, commands), green (`cm.meas`: measurements), orange (internal signals and flags). |
| `nb(sys, kind, name, ...)` | New block of a standard size by kind (`sum`, `gain`, `div`, `minmax`, `sumel`, `lag`, `pid`, `switch`, `sat`, `satdyn`, `rate`, `dz`, `rel`, `or`, `term`, `mem`, `tf`, `int`, `dint`). |
| `kc(sys, name, value)` | Parameter constant (name hidden, value shown). |
| `lagBlk(sys, name, T, y0)` | Discrete first-order lag: Transfer Fcn First Order with pole `1 - Ts/T` and initial output `y0`, i.e. $y_k = p\,y_{k-1} + (1-p)\,u_k$, $p = 1 - T_s/T$. |
| `contLag(sys, name, T)` | Continuous first-order lag $1/(Ts+1)$ built from blocks (see below). |
| `piBlk(sys, name, Kp, Ki, lim)` | Discrete PID block in PI mode with clamping anti-windup, sample time inherited; `lim = ""` selects external limit ports (up, lo), otherwise symmetric `±lim`. |
| `ctrlSave(mdl, folder)` | Save, close and register in the project. |

**Why this layout.** It was introduced in `b296539` ("Readable controller and asset diagrams") and `dbbf75e` (same for the Network) so that every signal path reads as one straight row, side inputs sit next to their ports, and no line crosses: the models are meant to be read and edited by hand. Logic was unchanged (bit-identical results in the regression cases).

#### contLag and why it exists

`contLag` builds $1/(T s + 1)$ as a masked subsystem: `u` → Sum (`+-`) → Gain `1/(T)` → Integrator → `y`, with the output fed back to the Sum. It is used for the converter current lags `Tg ip`, `Tg iq` in the asset models (`T = ap.gen.Tg`).

**Why.** Compiled code (accelerator, Rapid Accelerator, the later DLLs) keeps a block parameter that uses a model argument tunable only if the expression is simple arithmetic. A Transfer Fcn block with denominator `[ap.gen.Tg 1]` is a vector concatenation and failed the first compiled builds (design §13, commit `ee3b136`). The gain `1/(ap.gen.Tg)` is plain division and stays tunable. The dynamics are identical.

**Alternative tried and dropped.** Making the asset lags discrete gave no speed gain and moved P during a dip by 2–3 MW, so the continuous lags were kept (design §11).

#### The templates one by one (structure only; control laws in Part B)

**`buildHppcCtrl(folder)`** (line ~1956): HPPC, 20 inputs and 6 outputs taken from `modelInterfaces().HPPC`. Two columns of function areas:
- *Measurements, SOC-aware BESS limits, P limits:* lags `P filt` (`par.Tp`), `Q filt`, `V filt` (`par.Tq`); MATLAB Function `dip freeze` (persistent counter: freeze while `V_poc < Vfrz` and for `Thld` after); `SOC limits` (discharge/charge derated over `SOCband`); `available` sum → `P_hi`, `charge limit` → `P_lo`.
- *Ramp limit, frequency response, export limit:* MATLAB Function `ramp limiter` (persistent state, starts at the set-point, `rampUp`/`rampDn` per minute, `rampFlag`), `frequency response` (LFSM-O, LFSM-U, optional FSM, `Frqflag`), sum, `export limit` min with `Ppoc_max`.
- *P control at the POC:* reference lag `Tref`, error, switches for `Pflag` and the dip freeze, error limits `femax`/`femin`, PI `Kpg Kig` with external limits, feed-forward sum, dynamic saturation → `P_ord`.
- *Q, V or PF control:* slope gain `Kc = par.Vslope / par.Qmax`, `POC Q limit` (MATLAB Function), `tan phi` gain and PF switch, Q feed-forward switch, `TrefQ` lag, switches for V control / `Qflag` / dip freeze, error limits, `plant anti-windup` (MATLAB Function on `Q_sat_*`), PI `Q PI` with gains `par.KpQ`, `par.KiQ`, limits `QmaxOrd`/`QminOrd` → `Q_ord`.
- *Dispatch to the plants:* MATLAB Function `dispatch` → the 6 set-points.
- *Not used yet:* terminators for `P_wpp`, `Q_wpp`, `P_spp`, `Q_spp`, `P_bess`, `Q_bess`.
The code comment notes that derived values (tan φ, mode flags, mode-dependent gains) come from `controllerParameters` to keep block expressions tunable. `Ts` enters the MATLAB Functions as an input from a constant `Ts` (not through `par`).

**`buildPlantCtrlRepc(name, folder)`** (line ~2212): WPPC, SPPC, BESSC from one template (`isB` for BESSC). Areas: *Measurements and P limits* (lags `Tp`, `Tfltr`; sum of `P_avail_k`; BESSC: MATLAB Function `BESS limits`); *Frequency response* (deadband `fdbd1/2`, `Dup`, `Ddn`); *P control (Frqflag)* (PI `Kpg Kig`, reference fed forward, switch on `Frqflag`, dynamic limits, lag `Tlag`); *Q or V control (RefFlag)* (droop `Kc`, `Vref`, freeze below `Vfrz`, deadband, error limits, PI whose gains are the expressions `par.Kp + par.RefFlag*(par.KpV / (par.Kc + par.dVdQ) - par.Kp)` and the `Ki` equivalent, which use only + − × ÷; lag `Tfv`; MATLAB Function `Q saturation` → `Q_sat`); *Dispatch to the assets* (MATLAB Function `dispatch`). `ctrlHeader(..., withN = true)` adds `N` to the model workspace; inputs `P_avail_k` and `SOC_k` have dimension `"N"`.

**Asset controller templates.**
- `buildWTCtrlType4A(folder)` (line ~1730), IEC 61400-27-1 Type 4A: *Measurement and dip detection* (lags `Tufilt`, `Tpfilt`, `Tqfilt`; `U floor` with `Vmin`; `dip` = U < `udip`; the filtered P is terminated, since Type 4A has no P loop); *P control type 4A* (dynamic saturation by `P_wind_avail`, rate limit `dpmaxp4A`, lag `Tpordp4A`, divide by U); *Q control* (open loop q/U plus PI `kpq kiq` switched by `par.MqG1`, frozen in a dip; dip current `Kqv·(uref0 − U)` limited by `iqH1`); *Current limitation* (MATLAB Function, P priority normally, Q priority in a dip). Output `P_avail = P_wind_avail`.
- `buildReecCtrl(type, folder)` (line ~1826), PV_ctrl (REEC_B) and BESS_ctrl (REEC_C): *Voltage filter and dip detection* (`Trv`, `Vmin`, `Vdip`, `Vup`); *P path* (limits 0…`P_pv_avail` or ±`Pmax`, ramp `dPmax`/`dPmin`, lag `Tpord`, divide by V); *Q path* (Q/V plus PI `Kqp Kqi` switched by `Qflag`, frozen in a dip, lag `Tiq`, injection `Kqv` with deadband `dbd1/2` and limits `Iqh1/Iql1`); *Current limit logic* (MATLAB Function, `PQflag`, BESS SOC limits). `P_avail` = available PV power or the BESS `Pmax`.

**Asset model template `buildAsset(type, folder)`** (line ~755): configuration `ode4`, `FixedStep = Tsim`; model workspace `ap`, `Tsim`; `ParameterArgumentNames = "ap"`. Inputs `P_set`, `Q_set`, `V_re`, `V_im` (+ `P_wind_avail` / `P_pv_avail`), sample time inherited. Areas: *Terminal measurements* (MATLAB Function `terminal` → `V_term`; Memory `P meas`, `Q meas`); *Asset controller* (Model block `ctrl` → `<TYPE>_ctrl`, argument `par = ap.ctrl`); *Generator system / Converter* (WT: rate limits `dipmax` (rise only), `diqmax`, `contLag` `Tg`; PV/BESS: REGC_A rate limits `Rrpwr`, `Iqrmax/Iqrmin`, `contLag` `Tg`, MATLAB Function `LVPL LVACM`; then MATLAB Function `generator`: current source aligned with the terminal voltage, $I = (i_p - j i_q)\,V/|V|$, $P = |V| i_p$, $Q = |V| i_q$); *Battery energy* (BESS: Gain `-ap.kSOC` → Integrator with initial `ap.SOC0`). Outputs `P`, `Q`, `P_avail`, `I_re`, `I_im` (+ `SOC`).

#### The MATLAB Function block pattern

`addMatlabFunction(blk, fname, inNames, outNames, body, paramName, pos)` (line ~666) is the single way the build creates MATLAB Function blocks:

1. Add the library block.
2. Find its chart through the Stateflow API: `sfroot().find("-isa", "Stateflow.EMChart", "Path", blk)`.
3. Write `ch.Script` = `function [outs] = fname(ins[, par])`, `%#codegen`, the body, `end`.
4. If a parameter name is given, find the Stateflow data of that name and set its `Scope = 'Parameter'`, so it is resolved from the model (or mask) workspace: `par` / `ap` in the models, `net` in the nodal network.

Conventions followed by every template: all signals `double`, per unit on the model's base; `%#codegen` so the code can be generated; persistent variables for state where needed (`dip freeze`, `ramp limiter`); comments in the code that state the function's intent. Code inside a MATLAB Function block is not restricted by the tunability rule (expressions on `par` there are evaluated in the generated code at run time).

### A3.9 utilities/build/modelInterfaces.m

#### What

`I = modelInterfaces()` returns one field per controller and asset model, each with:

- `folder`: `"ctrl"` or `"assets"` (under `models/`);
- `in`, `out`: port names in port order;
- `arg`: `"par"` (controllers) or `"ap"` (assets);
- `src`: the field path of the argument's default in `HPP_parameters()`.

| Model | Inputs (in order) | Outputs (in order) | arg | src |
|---|---|---|---|---|
| HPPC | `P_ref`, `Q_ref`, `f_meas`, `P_poc`, `Q_poc`, `V_poc`, `P_avail_wpp`, `P_wpp`, `Q_wpp`, `Q_sat_wpp`, `P_avail_spp`, `P_spp`, `Q_spp`, `Q_sat_spp`, `P_chg_max_bess`, `P_dis_max_bess`, `SOC_bess`, `P_bess`, `Q_bess`, `Q_sat_bess` (20) | `P_set_wpp`, `Q_set_wpp`, `P_set_spp`, `Q_set_spp`, `P_set_bess`, `Q_set_bess` | par | `ctrl.hppc.par` |
| WPPC, SPPC | `P_set`, `Q_set`, `P_meas`, `Q_meas`, `V_meas`, `P_avail_k`, `f_meas` | `P_set_k`, `Q_set_k`, `P_avail`, `P_plant`, `Q_plant`, `Q_sat` | par | `ctrl.wppc.par`, `ctrl.sppc.par` |
| BESSC | as above with `SOC_k` before `f_meas` | as above with `P_chg_max`, `P_dis_max`, `SOC` before `Q_sat` | par | `ctrl.bessc.par` |
| WT_ctrl | `P_set`, `Q_set`, `V_term`, `P_meas`, `Q_meas`, `P_wind_avail` | `ip_cmd`, `iq_cmd`, `P_avail` | par | `ctrl.wt.par` |
| PV_ctrl | … `P_pv_avail` | same | par | `ctrl.pv.par` |
| BESS_ctrl | … `SOC` | same | par | `ctrl.bess.par` |
| WT_asset | `P_set`, `Q_set`, `V_re`, `V_im`, `P_wind_avail` | `P`, `Q`, `P_avail`, `I_re`, `I_im` | ap | `wt.ap(1)` |
| PV_asset | … `P_pv_avail` | same | ap | `pv.ap(1)` |
| BESS_asset | `P_set`, `Q_set`, `V_re`, `V_im` | … plus `SOC` | ap | `bess.ap(1)` |

#### Why

`HPP.slx` connects to the controller and asset models **by port number**. Once the models became hand-maintained, the port list had to live in one place that both the generator and the checker read. `buildHppcCtrl` (template ports) and `buildHppcSub` (wiring and link labels) read `I.HPPC`; `checkModels` compares every model against it. The plant and asset wiring in `buildPlantSub` still uses port numbers directly, consistent with the lists (comment in `modelInterfaces.m`: "When a model's ports change on purpose, change them here and in buildHPP's wiring together").

#### Consequences

To add a port: change the model, change `modelInterfaces.m`, change the wiring in `buildHppcSub` / `buildPlantSub` (and the template, if you want it to stay in sync), then `buildHPP()`. The anti-windup `Q_sat` ports (stage 5 step 4) were added this way: HPPC 20 inputs, plant controllers one more output (design §8).

Port dimensions and data types are not part of `modelInterfaces` and are not checked.

### A3.10 scripts/checkModels.m

#### What

`T = checkModels(Models=…, Strict=…, Root=…, Quiet=…)` checks the hand-maintained models and returns a table with columns `Model`, `Check`, `Block`, `Detail`, `Fix`, one row per finding. Options: `Models` restricts the list; `Strict` errors on any finding (`checkModels:findings`); `Root` checks a copy under `<Root>/ctrl` and `<Root>/assets` (used to test the checker on deliberately broken copies); `Quiet` suppresses the printed report. Models that do not exist yet are skipped.

It runs at the start of `buildHPP` (warning, error with `Strict`), at the start of a Rapid Accelerator build in `rapidTarget` (a finding makes Rapid Accelerator fall back to normal mode with the reason), and as the `check` task of Run Build (`checkModels(Strict=true)`).

#### How it reads models without opening windows

For each model: `load_system(file)` loads the model into memory without opening an editor window (if a model of the same name is loaded from another file, that one is closed first; the shadowing warning is switched off for the load). An `onCleanup` closes the model afterwards if it was not loaded before. Everything else uses `find_system`, `get_param` and the Stateflow API on the loaded model.

#### The three checks

1. **port** (`portFindings`): finds Inports and Outports at the top level (`SearchDepth = 1`, `MatchFilter = @Simulink.match.allVariants`), sorts them by their `Port` number and compares the names with `modelInterfaces`. Reports missing, extra or renamed ports with their number. *Why:* `HPP.slx` connects by port number, so a renamed or reordered port would connect the wrong signal silently, or break the build.
2. **expression** (`blockFindings`, `badTokens`): visits every block, including inactive variant choices (`allVariants`, "they may become active"), under masks (`LookUnderMasks = all`) and through library links (`FollowLinks = on`), but skips the inside of MATLAB Function blocks (detected by `SFBlockType == "MATLAB Function"`). For every dialog parameter (`DialogParameters`) whose value contains `par.` (or `ap.`), it collects the references with the regular expression `(?<![\w.])par(\.[A-Za-z_]\w*)+` and checks the whole expression: whitespace removed; any `name(` is a function call or indexing (not allowed); then names with field access, numbers (including exponents) and the characters `+ - * / ( )` are removed; anything left (for example `^`, `==`, `<`, `[`, `]`, `,`) is reported. *Why:* compiled code (accelerator, Rapid Accelerator, generated code) can keep a block parameter that uses a model argument tunable only if it is simple arithmetic. The first compiled builds failed on `par.PF^2`, `sqrt`, comparisons (`par.RefFlag == 1`, `double(par.MqG == 1)`) and `[ap.gen.Tg 1]` (design §13). The suggested fix is to compute the value in `params/controllerParameters.m` as a new `par` field (as was done for `tanphi`, `isV`, `isPF`, `KpQ`, `KiQ`, `MqG1`) or to move the calculation into a MATLAB Function block.
3. **field** (`codeFields`, `fieldFindings`): gathers every `par.x.y` / `ap.x.y` reference from block parameters (from check 2) and from the code of all MATLAB Function blocks, read via the Stateflow API (`sfroot` → `Simulink.BlockDiagram` of the model → all `Stateflow.EMChart` objects → `.Script`, with comments stripped). It walks each path through the default argument struct (`eval("p." + src)`, e.g. `p.ctrl.hppc.par`) and reports paths that do not exist. *Why:* a misspelled or removed parameter field otherwise appears only as a compile error deep inside a referenced model, or (in a MATLAB Function) only when the model is compiled.

The report gives block path and fix for each finding, e.g. "keep port names and order as in modelInterfaces.m (HPP.slx connects by number); if the change is intended, update modelInterfaces.m and buildHPP's wiring, then buildHPP()".

Verification recorded in `4a15dea`: no findings on the current models; three deliberate errors in a copy were found.

#### Limits (from the code)

- It checks only the ten controller and asset models, not `HPP.slx` (which is generated).
- It does not check port dimensions, data types, sample times or the model workspace code.
- The field check uses the default struct (`ap(1)` for assets), so a field present only in some instances would not be caught. (Today all instances are identical.)
- The expression check looks at the whole parameter string only when it contains `par.` / `ap.`; an expression that uses `Ts` with a function, for example, is not checked (not needed for tunability of the arguments).

### A3.11 scripts/checkModelBuild.m

`checkModelBuild(p)` loads `HPP`, reads `buildInfo` from its model workspace and errors if it is missing (`checkModelBuild:noInfo`: "Rebuild it with buildHPP") or if `[p.wt.N, p.pv.N, p.bess.N]` differs from the stored counts (`checkModelBuild:mismatch`, with both sets of numbers). `runCase` and `runSweep` call it before every run.

**Why.** `HPP.slx` contains one Model block per asset instance for the N it was built with. Case sets cannot change N, and a run with the wrong structure might not fail loudly (the Selectors would index into vectors of another width, or the network solver data would not match). This check stops it with a clear message.

**Limit.** Only N is compared. A layout change with the same N (different `nFeeders`/`nPerFeeder` split) is not detected; it matters for the Simscape network variant, whose blocks are laid out per feeder (A3.6). (inferred)

### A3.12 Templates against the saved models

Checked for this handbook by reading the `.slx` archives (Python `zipfile`, no MATLAB):

- **MATLAB Function code.** Every line of every MATLAB Function block in all eleven `.slx` files (HPPC 7 blocks, plant controllers 2–3, asset controllers 1, asset models 2–3, the network's `solve`, `current` and `phasor` blocks) appears verbatim in the corresponding template string in `buildHPP.m`. No hand edits of code exist yet.
- **Block parameter expressions.** All parameter values that reference `par`, `ap` or `p` in the saved models are expressions the templates produce (for example `1 - Ts/par.Tp` from `lagBlk`, `1/(ap.gen.Tg)` from `contLag`).
- **Configuration and workspaces.** Controllers: `FixedStepDiscrete`, `FixedStep = Ts`, `ParameterArgumentNames = par`, MATLAB-code model workspace as in `ctrlHeader`. Assets: `ode4`, `FixedStep = Tsim`, `ap`. `HPP.slx`: `ode1`, `FixedStep = p.sim.Ts`, `Decimation = p.sim.logDecimation`, `AlgebraicLoopMsg = error`.
- **HPP.slx structure.** 9 Model blocks in the default layout (2 WT, 2 PV, 1 BESS assets plus four controllers), 44 delay variants (`HPP_delays == 1/2`), one network variant pair (`HPP_network == 1/2`), consistent with the builders.

Stale text in `buildHPP.m` (comments, not behaviour): the header's "inports -> one MATLAB Function block logic -> outports"; "stage 1: placeholder logic" on `controllerSpecs`; "created only if missing unless Force=true" on `buildController` and `buildAsset` (now `Overwrite`); `makeLink`'s "(communication delay, variable length p.comm.steps.<name>)" (now fixed or variable by variant); `buildTSO`'s "constants for now". Docs: design §4.3 lists `omega_gen` (4B) as a WT input, which is not a port (future option); design §4.1 lists "`Q_ref` or `V_ref` from TSO", while `V_ref` is a parameter (`p.hppc.Vref`), not a port; design §3 says `buildHPP(All=true)` is needed after changing N, while `buildHPP()` suffices (A3.7).

### A3.13 The DLL path

#### What is already prepared

The models were shaped for code generation from the start (commit `467b5f2`, "Per-unit, tunable controller interface for code generation", and design §4):

- one referenced model per controller, with flat `double` ports (scalars, or fixed-width vectors of N in the plant controllers);
- per unit on the controller's own base, W↔pu conversion outside (in `HPP.slx`; in PSCAD on the PSCAD side);
- all tunable parameters in one struct `par` (model argument), meant to become the DLL's parameter list;
- fixed sample time `Ts` per controller, an integer multiple of the simulation step (in PSCAD a multiple of the EMT step);
- standalone defaults in the model workspace, so each model compiles on its own;
- block parameters that use `par` limited to + − × ÷ (checked by `checkModels`), so they stay tunable in generated code; already proven by the Rapid Accelerator builds, which compile all models (design §13);
- `%#codegen` in every MATLAB Function block.

#### What still needs to be done

From design §8 ("Afterwards: code generation of every controller (needs the Coder products and a compiler), the IEEE/CIGRE wrapper, and PSCAD validation"), §10 and the memory note on the Coder installation:

1. **Simulink Coder.** MATLAB Coder, Embedded Coder and MinGW64 are installed (2026-11-19). Simulink Coder, needed to generate code from Simulink models, does not show in `ver` although licensed; it is "being added". Check `ver` before starting.
2. **Code generation configuration per controller** (not started; the following is the expected work, inferred): a code-generation target for each of the seven controller models, with `par` kept as a tunable parameter struct in the generated interface, and the per-instance state held in instance data (the controllers use persistent variables and discrete states, e.g. `dip freeze`, `ramp limiter`, PI integrators).
3. **IEEE/CIGRE wrapper.** A C wrapper that exposes the generated step function, inputs, outputs and parameters through the IEEE/CIGRE DLL interface used by PSCAD (not started; no wrapper code exists in the repository).
4. **PSCAD side.** The W↔pu conversions, the communication delays and the measurement points that `HPP.slx` provides around the controllers must be rebuilt in PSCAD, and the asset (plant-side) models are replaced by PSCAD's EMT models.
5. **PSCAD validation** of the closed loop against the Simulink RMS results.

Consequences for development now: keep new controller logic within the interface rules (A3.8, A3.10) so the DLL step stays mechanical; a change of N requires a new plant-controller DLL (design §3); and the asset controllers run at 1 ms, the simulation step, which in PSCAD must be a multiple of the EMT step.

### Recreating it

1. Write `utilities/build/modelInterfaces.m`: for each of the ten controller and asset models, folder, input and output port names in order, argument name (`par` / `ap`) and default path in `HPP_parameters()`.
2. Write the parameter side first (chapter A2): `p.ctrl.<c>.{Ts, Pbase, par}`, `p.<type>.ap(k)` with `ctrl` and `gen`, `p.comm.*` and `p.comm.steps.*`, `p.net.solver`, and derived `par` fields so that block expressions need only + − × ÷.
3. In `buildHPP.m`, write the generic helpers: `newSubsystem`, `inport`, `ln`, `gotoFrom`, `addGoto`/`addFrom`, `portY`, `place`, `rowInto`/`rowFrom`, `setArg`, `addMatlabFunction` (Stateflow API, `Scope = 'Parameter'`), `registerInProject`, `renumberPorts`, `hideStructuralNames`, `areaColor`, `addArea`, `alignAreaGrid`.
4. Write the controller layout helpers (`stg`, `rowChain`, `ctrlSection`, `areaAround`, `alignColumns`, `ctrlInputs`, `ctrlOutputs`, `tagColor`, `nb`, `kc`, `lagBlk`, `contLag`, `piBlk`, `ctrlHeader` with the MATLAB-code model workspace and `ParameterArgumentNames`, `ctrlSave`).
5. Write the templates: `buildWTCtrlType4A`, `buildReecCtrl`, `buildPlantCtrlRepc`, `buildHppcCtrl` (ports from `modelInterfaces`), and `buildAsset` (asset controller as a Model block with `par = ap.ctrl`, `contLag` for the current lags). Generate them once with `buildHPP(All=true)`; from then on edit the `.slx` files.
6. Write `makeLink` (gain / memory / delay steps, mask labels) and `buildDelayVariant` (Fixed and Variable choices on `HPP_delays`, activation at update diagram).
7. Write `buildNetwork` (masked, `net` non-tunable in the MATLAB Function, numbers as tunable Constants, one-step-old voltages), `buildNetworkSimscape` and its block helpers, and `buildNetworkVariants` (choices on `HPP_network`).
8. Write `buildTSO`, `buildHppcSub` (groups per bus, links per port, BESS port mapping), `buildPlantSub` (controller, links, N asset rows with Selectors by global index, Muxes, measurement bus, `SOC_plant`), `buildLogging` and `logSignalNames`.
9. Write `buildTop`: solver ode1, fixed step `p.sim.Ts`, single-tasking, decimation, `AlgebraicLoopMsg = error`; place blocks by ports; lines forward, Goto/From backward; root outports; `buildInfo`, `HPP_network`, `HPP_delays` in the model workspace; save.
10. Write `buildHPP` itself: option checks, `checkModels` at the start, controllers before assets, never overwrite without `All=true, Overwrite=true`, then `buildTop`.
11. Write `scripts/checkModels.m` (load without UI, port / expression / field checks, table and report, `Strict`) and `scripts/checkModelBuild.m` (compare `buildInfo` with `p`), and call them from `buildHPP`, `rapidTarget`, `runCase`, `runSweep` and the Run Build `check` task.
12. Verify: `checkModels()` reports nothing; `buildHPP()` leaves the ten source models unchanged; a nominal run gives the reference results (chapter B7).

## A4. Case system

A *case* is one simulation: the default parameters plus a few changed values. A *case set* is a named list of cases studied together. This chapter covers how case sets are defined, how a case row becomes a parameter struct, and every case set in the project with its purpose.

### A4.1 Overview and data model

**What.** A case set is a MATLAB function in the package folder `scripts/+caseSets/` that returns a **table**:
- each **column** is a field path in `p`, written as a string (`"grid.Pref"`, `"pc.wppc.Qmax"`, `"grid.dip.depth"`);
- each **row** is one case;
- **`NaN`** in a cell means "keep the default";
- rows may have **names** (`RowNames`), which become case labels.

README: "A case set is a function in `scripts/+caseSets/` that returns a table. Each column is a field path in `p` (for example `"grid.Pref"`), each row is one case, and `NaN` means "keep the default". Rows may have names."

**Why a table of field paths with NaN defaults.**
- *Only differences are listed.* A case says what it changes, nothing else. Defaults live in one place (`params/`), and a change to a default reaches every case that does not override it.
- *Self-describing.* The column names *are* the parameter paths, so a table printed in the Command Window shows exactly what each case does. `caseLabels` can build readable legend labels from it.
- *Generic runner.* `runCase`/`runSweep` need no knowledge of any set. They only call `applyCase(p, row)`.
- *Typo-safe.* Because columns are paths, `applyCase` can check that each one exists (A4.6).
- *Easy to grow.* With `caseList`, adding a case is one line, and adding a parameter to one case does not touch the others (commit `527ab97`).

**Alternatives.** Before `caseList` (`527ab97`), `handpicked` built the table directly with every value written out for every case. Adding a parameter then meant editing every row. `caseList` produces the same table from a list of name-value pairs per case. Struct arrays of overrides or one script per case were not used (no reason recorded). A table is easy to display, index by row name or number, and subset (`cases(caseIds, :)` in `main.m`) (inferred).

### A4.2 The package folder `scripts/+caseSets/`

**What.** A folder whose name starts with `+` is a MATLAB *package*. Its functions are called with the prefix: `caseSets.handpicked()`. One function per set, file name = set name.

**Why a package.**
- *Namespace.* Set names like `nominal` or `wt_frt` cannot clash with other functions on the path.
- *Discovery.* `listCaseSets` finds all sets with `what("caseSets")`. Adding a file adds a set, with no registry to update.
- *Path.* Only the parent `scripts/` is on the project path (A1.5).

**Consequences.** To add a set, copy a file, rename it (file name = function name = set name) and edit it. Nothing else changes (README). Set names must be valid MATLAB identifiers.

### A4.3 Table builders: `caseList`, `caseGrid`, `caseOAT`

#### `caseList(list)`: named cases, one per line

```matlab
cases = caseList({
    "nominal",  {}
    "lowRef",   {"grid.Pref", 100e6}
    "small",    {"grid.Pref", 100e6, "bess.Pmax", 15e6}
});
```

- Input: an N×2 cell array. Column 1 holds the case names, column 2 a cell of name-value pairs `{path, value, ...}`.
- Checks: shape N×2 (`caseList:shape`); names non-empty and unique (`caseList:names`); pairs come in twos (`caseList:pairs`); each value a numeric or logical **scalar** (`caseList:value`).
- Columns appear in the order of first use (`unique(params, "stable")`).
- Missing values are `NaN`. Row names are the case names.

Typical idioms in the sets: shared pairs in a variable (`T = {"sim.Tstop", 120}`), combined by cell concatenation (`[T, {"grid.Pstep.dP", -30e6}]`), and overriding a whole column after the call (`cases.("sim.Tstop")(:) = 120`).

**Consequence:** string-valued fields (`net.solverType`, `comm.delays`) cannot be set by `caseList` (numeric only). These are chosen per run through the `Network` and `Delays` options instead.

#### `caseGrid(name, values, ...)`: full factorial

```matlab
cases = caseGrid("grid.Pref", [100 150 200]*1e6, "bess.Pmax", [0 30 60]*1e6);
```

- Every combination of the given values (built with `ndgrid`). **The first parameter varies fastest.**
- Rows are unnamed; refer to them by number. `caseLabels` builds labels such as `"Pref=100 MW, Pmax=0 MW"`.
- Error `caseGrid:args` for an odd number of arguments.

#### `caseOAT(name, values, ...)`: one parameter at a time

```matlab
cases = caseOAT("grid.Pref", [50 100 200]*1e6, "bess.Pmax", [0 60]*1e6);
```

- Each row varies exactly one parameter; the other columns are NaN.
- Rows are named by `caseLabels` (`"Pref=100 MW"`, `"Pmax=0 MW"`), made unique with `matlab.lang.makeUniqueStrings` (commit `3ddc800`: "caseOAT now names its rows with caseLabels").
- Error `caseOAT:args`.

**Why three builders.** They cover the three usual study designs: hand-picked scenarios (`caseList`), interaction studies (`caseGrid`), and sensitivity studies (`caseOAT`). All three return the same table format, so the rest of the system does not care how a set was built.

### A4.4 `caseLabels`: naming

`labels = caseLabels(cases)`:
- If the table has row names, those are the labels.
- Otherwise each row lists the values it changes, `name=value` joined by `", "`. `name` is the path after the first dot (`"grid.Pref"` → `Pref`, `"grid.Pstep.dP"` → `Pstep.dP`). Names starting with `P` are printed in MW (`value/1e6`, `"Pref=100 MW"`). A row that changes nothing is `"nominal"`.

Used by `caseOAT` (row names), `plotResults` (legends) and anywhere a case needs a readable name.

**Why the MW rule.** Powers are stored in W (convention A1.10), but plots and legends read better in MW. Testing the first letter is a cheap heuristic that matches the existing power parameters (`Pref`, `Pmax`, `Pn`).

**Caveat (latent).** The heuristic would also format non-power names that start with `P` (for example `PF` or `PFsign`) in MW, e.g. `"PF=9.5e-07 MW"`. This only matters for unnamed rows (`caseGrid`, `caseOAT`) that vary such a field. No current set does: `hppc_qv` uses named rows.

### A4.5 `defineCases`: the case table of a set

```matlab
cases = defineCases()                    % "nominal"
cases = defineCases("grid_Pref_Pbess")
```

1. `setName` defaults to `"nominal"`.
2. Checks that the set exists in `listCaseSets()`. Otherwise error `defineCases:unknownSet` with the list of available sets.
3. Calls `feval("caseSets." + setName)`.
4. Checks the result is a non-empty table (`defineCases:empty`).
5. **Mixed stop time warning** (new, in the uncommitted working copy): if the table has a `sim.Tstop` column, NaN entries are replaced by the default `simulationParameters().Tstop` (60 s). If more than one distinct value remains, it warns `defineCases:mixedStopTime`: "Case set "..." mixes stop times (... s); its cases are plotted together, so give them all the same sim.Tstop."

**Why the stop-time warning.** All cases of a set are plotted together on one time axis (`plotResults`, `plotAllResults`), and `runSweep` stores them together. Before this change, `bess_soc` set `sim.Tstop = 120` only for its two "+20 MW" cases (the others ran 60 s), and in `hppc_qv` only the four Q-limit and anti-windup cases ran 90 s. The working copy fixes both sets (`cases.("sim.Tstop")(:) = 120;` and `= 90;`) and adds the warning so it cannot happen again unnoticed. It is a warning, not an error: a set with mixed stop times still runs.

**Who calls it.** `runCase`, `runSweep` (and through them `main.m`, `buildfile.m`, the HPC jobs), `main.m` directly (to select cases), `hpc/submit.sh` (to count cases), and `plotAllResults`/`plotResults` for case lists.

### A4.6 `applyCase`: from a case row to `p`

```matlab
p = applyCase(HPP_parameters(), cases(k, :));
```

**What** (`scripts/applyCase.m`):
1. For each column name of the one-row table:
   - take the value (unwrap a cell if the column is a cell);
   - if it is a numeric scalar `NaN`, skip it (**NaN = default**);
   - split the name at `.` into a field path;
   - check that the path exists in `p` (`isFieldPath`: every level must be a struct with that field). Otherwise error `applyCase:unknownField`: "Case column "..." is not an existing field of p." This is the **typo protection**;
   - set it with `setfield(p, path{:}, v)`.
2. **Re-derive:** `p = deriveParameters(p)`, so `N`, `Prated`, `comm.steps`, `p.ctrl`, `ap` and `p.net.solver` follow the new values.

**Why each step.**
- *NaN = default:* lets tables be rectangular (every case has every column) while each case lists only what it changes.
- *Typo protection:* without it, `"grid.SRC"` would silently create a new unused field, and the case would run as the nominal case with a wrong label. Applying cases only to existing paths makes such a mistake an immediate error. This is possible only because every parameter lives in one struct (A1.2).
- *Re-derivation:* the same rule as for variants (A2.1). A component change must reach every value derived from it. For example `{"grid.SCR", 3}` changes `p.net.solver.Z*`, `p.ctrl.hppc.par.dVdQ`, the plant `dVdQ` and the voltage-control gains. `{"bess.Pmax", 0}` changes `p.ctrl.bess.par.Pmax`.

**The `ctrl.*` caveat.** A column such as `"ctrl.hppc.par.Kpg"` passes the existence check (the field exists) and is set, but `deriveParameters` then **rebuilds `p.ctrl` from scratch** from `p.hppc`, `p.pc`, `p.wt.iec.ctrl` and the others. The case value is silently lost. The same holds for every derived field: `wt.N`, `wt.Prated`, `wt.ap`, `bess.ap`, `comm.steps`, `net.solver`. **Always set the source field:** `hppc.Kpg`, `pc.wppc.Qmax`, `wt.iec.ctrl.kiq`, `bess.wecc.ctrl.Imax`, `bess.SOC0` (not `bess.ap.SOC0`). The case sets follow this rule (`"pc.wppc.Frqflag"`, `"hppc.FSMflag"`).

**Other limits.**
- Per-instance values cannot be set from a case: a path like `wt.ap(1).gen.Tg` is not a plain field path (`ap(1)` is not a field name), and `ap` is derived anyway.
- Changing a layout field (`wt.nFeeders`, `wt.nPerFeeder`) changes N. `checkModelBuild` then stops the run, because `HPP.slx` was built for another N. Layout changes need `buildHPP` and are not case-set material. `nAgg` and `cable.nPar` do *not* change N and can be varied.
- `runSweep` with Fast Restart refuses columns that are fixed at compile time: `net.solverType`, `sim.Ts`, `comm.maxDelay`, `comm.delays`, anything ending in `.nFeeders`, `.nPerFeeder`, `.N`, and with fixed delays any `comm.*` (`checkTunable`, `scripts/runSweep.m` ~line 187). (The `.N` entry is redundant in practice, because `N` is derived and overwritten anyway.)

**Debugging a case in the editor** (README): `c = defineCases("handpicked"); p = applyCase(HPP_parameters(), c("noBess", :));` then open `HPP.slx` and press Run. The base-workspace `p` is then the case's `p`.

### A4.7 `listCaseSets`

```matlab
names = listCaseSets()   % sorted string array of set names
listCaseSets()           % prints each set with its number of cases
```

`what("caseSets")` lists the package's `.m` files. Names are sorted. With no output argument it calls every set to count its rows (`height(feval(...))`). This also quickly shows any set that errors. `defineCases` uses it to validate names, and `main.m` (`setName = "all"`) uses it to loop over every set.

### A4.8 How event times and stop times were chosen

Some facts about the nominal profile explain the times in the sets:
- **Wind availability** steps from 0.6 to 0.9 pu at `p.wt.tStep` = 20 s. Before the step the plant cannot reach P_ref = 150 MW: wind 0.6 × 150 = 90 MW, PV 20 MW and the BESS at most 30 MW give about 138 MW (design.md: "P_poc 137.8 MW with the BESS at its discharge limit").
- After the step, the **ramp limit** (36 MW/min, stage 5 step 3) brings the plant to 150 MW gradually. With FSM on, the BESS reserve lowers the available headroom until about 40 s (comment in `freq_step`).
- So the plant reaches a **steady operating point at about 40–45 s**. Events that should hit a steady plant are placed after that.

Recorded reasons (commit `9379f22`, design.md section 12): "frequency steps at 50 s and voltage dips at 45 s, after the plant has ramped up following the wind step at 20 s". The same commit removed open-loop cases and the no-response frequency case from the sets, because they were trimmed for the report figures. Stop times are as long as each event needs to settle: 60 s default, 70 s for frequency steps at 50 s, 90 s for Q-limit settling (~20 s after 40 s plus margin), 120 s for 30–40 MW ramps at 36 MW/min (~50 s after 40 s).

### A4.9 Every case set

Eleven sets exist. Counts: `bess_soc` 7, `comm_delay` 6, `freq_step` 9, `grid_Pref_Pbess` 9, `grid_strength` 3, `handpicked` 4, `hppc_p` 6, `hppc_qv` 16, `nominal` 1, `oat_Pref` 8, `wt_frt` 4 (73 cases in total). The Run Build `sweeps` task runs `nominal`, `freq_step`, `hppc_p`, `hppc_qv`, `wt_frt` and `bess_soc` (the sets with report figures). Results quoted below are from design.md (the code was not run for this handbook).

#### `nominal` (1 case): the project default

```matlab
cases = caseSets.handpicked();
cases = cases("nominal", :);
```

- **What:** row `nominal` of `handpicked`, with all columns NaN, so all defaults: P_ref 150 MW, Q_ref 0, wind step at 20 s, 60 s run.
- **Why a separate set:** it is the default of `defineCases`, `runCase` and `main.m`, so a bare `runCase()` runs it. Defining it as a row of `handpicked` keeps one definition of "nominal".
- **What it shows:** normal operation. The plant ramps up after the wind step at 36 MW/min and holds P_ref at the POC. The BESS covers the shortfall before the step and absorbs the surplus while the plant ramps (report figure `nominal_dispatch`). design.md: P_poc 150.00 MW for P_ref 150 MW. Open loop it was 146.5 MW: the HPPC P loop covers the losses with the BESS.

#### `handpicked` (4 cases)

| Case | Changes |
|---|---|
| `nominal` | – |
| `lowRef` | `grid.Pref` = 100e6 |
| `highRef` | `grid.Pref` = 200e6 |
| `noBess` | `bess.Pmax` = 0 |

- **Why:** the original example set from the initial commit, and the template for `caseList` ("To add a case, add one line. The other cases never need editing.").
- **What it shows:** P_ref below and above what the plant can deliver. 200 MW exceeds the 180 MW export limit (`Ppoc_max`), so P_poc is capped (inferred from the HPPC export limit). Without the BESS, nothing balances the plant before the wind step, and the losses are covered only by curtailment headroom (inferred).

#### `grid_Pref_Pbess` (9 cases): full grid

```matlab
cases = caseGrid("grid.Pref", [100 150 200]*1e6, "bess.Pmax", [0 30 60]*1e6);
```

- Rows: (100, 0), (150, 0), (200, 0), (100, 30), ..., (200, 60) MW. P_ref varies fastest. Labels like `"Pref=100 MW, Pmax=0 MW"`.
- **Why:** the example of a factorial study from the initial commit (2026-11-13), when the model had one controller and the BESS power was a plain limit. README Quick start uses it as the `runSweep` example.
- **Caveat (inferred):** since stage 4, `bess.Pmax` is the controller limit while the converter rating stays `bess.Pn` = 30 MW. A 60 MW `Pmax` gives `p.ctrl.bess.par.Pmax = 2` pu, but the converter current limit `Imax` = 1.1 pu still caps the BESS at about 33 MW. The 60 MW cases therefore do not model a 60 MW battery. To study a bigger BESS, change `bess.Pn` (and `net.bess.unit.S`) as well.

#### `oat_Pref` (8 cases): one at a time

```matlab
cases = caseOAT("grid.Pref", [50 100 150 200]*1e6, "bess.Pmax", [0 15 30 60]*1e6);
```

- Rows: `Pref=50 MW`, `Pref=100 MW`, `Pref=150 MW`, `Pref=200 MW`, `Pmax=0 MW`, `Pmax=15 MW`, `Pmax=30 MW`, `Pmax=60 MW`. `Pref=150 MW` and `Pmax=30 MW` equal the nominal case.
- **Why:** the example of a sensitivity study from the initial commit. The same `Pmax` caveat as above applies.

#### `grid_strength` (3 cases)

| Case | Changes |
|---|---|
| `SCR10` | – (default, strong grid) |
| `SCR5` | `grid.SCR` = 5 |
| `SCR3` | `grid.SCR` = 3 (weak grid) |

- **Why:** added with the network in stage 2 (`54bf4da`). The SCR changes the POC voltage, the losses and the voltage-control loop gain.
- **What it shows:** steady state at different grid strengths. design.md: P_poc 150.00 MW in all cases with the closed P loop. It also served as the first Fast Restart test: an SCR change without recompile gives the same result as a full run (V_poc 0.9653 at SCR 3, 1 × 1 layout, `fb180ee`).

#### `wt_frt` (4 cases): fault ride-through

All cases: `grid.dip.t0` = 45 (150 ms default duration, 60 s runs).

| Case | Changes |
|---|---|
| `no dip` | t0 only (depth stays 0) |
| `dip 0.5` | `grid.dip.depth` = 0.5 (source voltage to 0.5 pu) |
| `dip 0.8` | depth 0.8 (to 0.2 pu) |
| `dip 0.5 SCR3` | depth 0.5, `grid.SCR` = 3 |

- **Why:** added in stage 3 with the IEC Type 4A wind turbine (`0e70343`), originally with the dip at 30 s. Moved to 45 s in `9379f22` "so it hits a steady operating point" after the ramp.
- **Why a `no dip` case:** the reference for P and Q, so recovery can be read as a difference (report figure `frt_p`).
- **What it shows:** reactive current injection supports the POC voltage (`frt_vq`). P recovers within 0.5 s after clearing without overshoot, because the HPPC P loop is frozen during the dip (`frt_p`). design.md stage 5: P_poc back within 1.5 MW 0.13 / 0.51 / 0.29 s after clearing. Known artefact: a single-step spike at fault clearing.

#### `freq_step` (9 cases): frequency response

All cases: `grid.fev.t0` = 50, `sim.Tstop` = 70.

| Case | Changes (besides t0 and Tstop) | Tests |
|---|---|---|
| `no event` | – | reference |
| `+0.2 Hz (LFSM-O limit)` | `grid.fev.df` = 0.004 | at the LFSM-O threshold 50.2 Hz: no response |
| `+0.5 Hz LFSM-O` | df = 0.010 | LFSM-O: −21.6 MW (0.3 Hz / 50 Hz / 5 % = 12 % of Pn) |
| `-0.5 Hz LFSM-U` | df = −0.010 | LFSM-U: +21.6 MW from the BESS |
| `+0.2 Hz FSM` | df = 0.004, `hppc.FSMflag` = 1 | FSM droop: −13.68 MW |
| `-0.2 Hz FSM` | df = −0.004, FSM | +13.68 MW |
| `+0.5 Hz FSM + LFSM-O` | df = 0.010, FSM | FSM (limited to 10 %) plus LFSM-O: −39.6 MW |
| `+0.2 Hz, decentral` | df = 0.004, decentral | stage-4 setup |
| `-0.2 Hz, decentral` | df = −0.004, decentral | stage-4 setup |

`decentral = {"hppc.Pflag", 0, "hppc.Frqflag", 0, "pc.wppc.Frqflag", 1, "pc.sppc.Frqflag", 1, "pc.bessc.Frqflag", 1}`: open-loop HPPC without frequency response, and each plant controller with its own REPC_A droop (5 %, ±20 mHz) and P loop.

- **Why:** added in stage 4 for the plant controller droops, then rewritten in stage 5 step 3 for the Energinet DK1 requirements (LFSM-O, LFSM-U, FSM). The decentral cases keep the stage-4 behaviour as the comparison for central control. That comparison is the first coordination case of the research question.
- **Why at 50 s, 70 s runs:** after the ramp and the FSM reserve period (A4.8), with 20 s to settle (90 % within 0.3 s, so the margin is generous).
- **What it shows:** report figures `freq_lfsm`, `freq_fsm` and `freq_central_decentral`. Central and decentral give about the same change at the POC. Decentral reacts slightly earlier but overshoots and curtails wind; central uses the BESS and settles without overshoot.

#### `hppc_p` (6 cases): P control with ramp limit

All cases: `sim.Tstop` = 120. Steps at `grid.Pstep.t0` = 40 s (default).

| Case | Changes |
|---|---|
| `Pref 150 MW` | – (reference) |
| `step -30 MW` | `grid.Pstep.dP` = −30e6 |
| `step +20 MW` | dP = 20e6 |
| `step +40 MW (limit)` | dP = 40e6 (190 MW requested, stops at the 180 MW export limit) |
| `step -30 MW, 10 %/min` | dP = −30e6, `hppc.rampUp` = `rampDn` = 0.10 |
| `step -30 MW, no ramp` | dP = −30e6, `hppc.rampFlag` = 0 |

- **Why:** added in stage 5 step 1 (`3adff21`) for the closed P loop. Extended in step 3 for the ramp rate constraint. 120 s "so the ramps can finish": 30 MW at 36 MW/min takes about 50 s.
- **What it shows:** `ramp_steps` (steps follow 36 MW/min; +40 MW stops at 180 MW), `ramp_rates` (the setting sets the slope; without the constraint ~1 s), `ramp_dispatch` (the BESS takes the change first; wind is curtailed only when the BESS reaches its charge limit). design.md: reached after 48 / 31 / 47 s; largest change in any 60 s window 36.46 MW (limit 36).

#### `hppc_qv` (16 cases): Q, voltage and PF control

All cases run 90 s (working copy: `cases.("sim.Tstop")(:) = 90`). Building blocks:
- `vdrop = {"grid.dip.depth", 0.02, "grid.dip.t0", 40, "grid.dip.dur", 100}`: a sustained 2 % grid voltage drop from 40 s (the 100 s duration outlasts the run).
- `vctrl = {"hppc.RefFlag", 1}`: voltage control.
- `plim = {"pc.wppc.Qmax", 0.2, "pc.sppc.Qmax", 0.2, "pc.bessc.Qmax", 0.2}`: plant Q limits low enough that the plants saturate before the POC limit.

| Case | Changes | Tests |
|---|---|---|
| `Q 0` | – | Q control, Q_ref = 0 (reference) |
| `Q step +30 Mvar` | `grid.Qstep.dQ` = 30e6 | Q_ref step at 40 s |
| `Q step -30 Mvar` | dQ = −30e6 | |
| `V ctrl, grid -2 %` | vctrl, vdrop | voltage control, slope 4 %, SCR 10 |
| `V ctrl, grid -2 %, SCR3` | + `grid.SCR` = 3 | weak grid |
| `V ctrl 2 %, grid -2 %` | + `hppc.Vslope` = 0.02 | smallest slope (KcBusMin applies) |
| `V ctrl 7 %, grid -2 %, SCR3` | + Vslope 0.07, SCR 3 | largest slope, weak grid |
| `V ctrl, SCR3, set for SCR10` | + SCR 3, `hppc.SCRset` = 10 | controller tuned for the wrong grid |
| `V ctrl, grid -10 % (Q limit)` | vctrl, depth 0.10, t0 40, dur 100 | asks for more than ±0.33 pu: Q_poc must settle at +59.4 Mvar |
| `V ctrl, grid +8 % (Q limit)` | vctrl, depth −0.08, t0 40, dur 100 | −59.4 Mvar |
| `V ctrl, plants limited, anti-windup` | vctrl, plim, depth 0.10, t0 40, dur 20 | plants saturate during 40–60 s; HPPC holds its loop |
| `V ctrl, plants limited, no anti-windup` | + `hppc.QawFlag` = 0 | the HPPC order winds up |
| `Q ctrl, grid -2 %` | vdrop | Q control in the same event: Q stays 0, V drops |
| `V ctrl, Vref 1.01` | vctrl, `hppc.Vref` = 1.01 | set-point change |
| `PF 0.95 export` | `hppc.RefFlag` = 2, `hppc.PF` = 0.95 | PF control |
| `PF 0.95 import` | + `hppc.PFsign` = −1 | |

- **Why:** added in stage 5 step 2 (`763e85a`) for Q/V control, extended in step 3 (slopes, SCR mismatch, PF) and step 4 (`309edbb`: Q limit, anti-windup). The voltage-control cases check the RfG 21(3)(d) requirement: 90 % of the Q change within t1 = 1 s, settled within t2 = 5 s, tolerance 5 % of Qmax, for slopes 2–7 % and SCR 3–10.
- **Why events at 40 s:** the default `Qstep.t0` and the chosen `dip.t0`. Note that the Q and V events come at 40 s, while the ramp after the wind step is not quite finished (A4.8). Q and V are largely decoupled from the P ramp, and the PF cases are meant to show Q following P *while* P ramps (report figure `pf_control`, window 2–60 s).
- **Why 90 s:** "the Q-limit cases need the time to settle" (back within 0.3 Mvar of the limit after ~20 s). The anti-windup cases need the 20 s drop plus the recovery (window 35–75 s).
- **What it shows:** report figures `q_steps`, `v_control`, `v_scr_mismatch`, `v_qlimit`, `v_antiwindup`, `pf_control`. design.md: t90 0.40–0.59 s, settled within 0.52 s; Q limit ±59.40 Mvar; anti-windup: Q back within 3 Mvar 0.9 s after the voltage returns, against 9.5 s at +15.8 Mvar without.

#### `bess_soc` (7 cases): SOC limits

All cases run 120 s (working copy). `up = {"grid.Pstep.dP", 20e6, "sim.Tstop", 120}`.

| Case | Changes | Tests |
|---|---|---|
| `SOC0 0.5` | – | reference |
| `SOC0 0.15` | `bess.SOC0` = 0.15 | inside the derating band above SOCmin (0.1–0.2): discharge derated |
| `at SOCmin` | SOC0 = 0.1 | no discharge at all (HPPC and REEC_C) |
| `SOC0 0.85` | SOC0 = 0.85 | inside the band below SOCmax: charge derated |
| `at SOCmax` | SOC0 = 0.9 | no charge |
| `SOC0 0.5, +20 MW` | up | P_ref +20 MW at 40 s: the BESS must discharge |
| `SOC0 0.15, +20 MW` | up, SOC0 0.15 | the derated BESS limits the plant |

- **Why:** added in stage 4 (`0e70343`) for the REEC_C SOC limits (SOC0 0.5 / at SOCmin / at SOCmax). Extended in stage 5 step 3 for the HPPC's SOC-aware derating (`SOCband` 0.1).
- **How it uses the nominal profile:** the BESS discharges until the wind step at 20 s and charges the surplus afterwards. Low-SOC cases are tested in the discharge phase, high-SOC cases in the charge phase. The "+20 MW" cases add a discharge demand later in the run.
- **What it shows:** `soc_derating` (a nearly empty BESS delivers less before the wind step: 14.6 MW at SOC 0.15 against 29.7 at 0.5; 0 at SOCmin) and `soc_step` (+20 MW with SOC 0.15: P_poc 165 instead of 170 MW). Near SOCmax the BESS charges −0.8 MW and wind and PV are curtailed, with P_poc at 150 MW.

#### `comm_delay` (6 cases): communication delays

All cases: `q = {"grid.Qstep.dQ", 30e6}`, a +30 Mvar Q_ref step at 40 s (default t0, 60 s runs).

| Case | Changes |
|---|---|
| `HPPC-plant 50 ms (default)` | q |
| `HPPC-plant 0 ms` | q, `comm.hppc_to_plant` = 0 |
| `HPPC-plant 100 ms` | q, hppc_to_plant = 0.100 |
| `HPPC-plant 200 ms` | q, hppc_to_plant = 0.200 |
| `all delays x2` | q, tso 0.400, meas_hppc 0.040, hppc_to_plant 0.100, meas_plant 0.040, plant_to_asset 0.040 |
| `no delays` | q, all five delays 0 |

- **Why:** added with variable-length delays (`ac18a90`, 2026-11-19) to study how delays between the levels affect the Q loop. The Q step is used because the Q loops form the deepest cascade (HPPC → plant → asset).
- **Why it is special:** it changes `comm.*` columns, so with `p.comm.delays = "auto"` `delayMode` compiles **variable-length** delays (up to `maxDelay` = 0.5 s; the largest here is 0.4 s), and the whole set runs on one compiled model (Fast Restart or Rapid Accelerator). design.md: 15 s for the 6 cases in Rapid Accelerator against 43 s in normal mode with Fast Restart, identical results.
- Not in the README case-set table and not among the report figures (see notes).

### A4.10 Notes: docs against code

- The README case-set table lists ten sets and omits `comm_delay`. README and design.md speak of "all seven case sets (46 cases)" for the Rapid Accelerator timing. That count matches `nominal`, `grid_strength`, `wt_frt`, `freq_step`, `hppc_qv`, `hppc_p`, `bess_soc` (1+3+4+9+16+6+7 = 46), the sets that existed and were timed then. There are now eleven sets with 73 cases.
- README describes `bess_soc` without the stop time. The working copy now runs all its cases 120 s and all `hppc_qv` cases 90 s (uncommitted). The `defineCases` mixed-stop-time warning is also uncommitted.
- `applyCase` help says "Derived parameters (e.g. p.ctrl limits) are rebuilt afterwards". This is correct, and it is exactly why `ctrl.*` columns are lost (A4.6). The help does not warn about it.
- `runSweep`'s compile-time check lists `.N`, which is derived. Harmless.
- `caseLabels` formats any name starting with `P` in MW, including `PF` (latent; no current set is affected).

### Recreating it

1. Create the package folder `scripts/+caseSets/` (parent `scripts/` on the path).
2. Write `caseList` (N×2 cell → table with row names, NaN for unset, checks for shape, unique names, pairs, numeric scalars), `caseGrid` (`ndgrid`, first parameter fastest, unnamed rows) and `caseOAT` (one non-NaN per row, rows named by `caseLabels`).
3. Write `caseLabels` (row names if any; otherwise `name=value` after the first dot, `P*` in MW, `"nominal"` for an empty row).
4. Write `listCaseSets` (`what("caseSets")`, sorted; print counts when there is no output).
5. Write `defineCases(setName = "nominal")`: validate the name against `listCaseSets`, `feval("caseSets." + setName)`, check for a non-empty table, warn on mixed `sim.Tstop`.
6. Write `applyCase(p, row)`: skip NaN, require existing field paths, `setfield`, then `deriveParameters`.
7. Write the sets: `handpicked` (with row `nominal`), `nominal` (that row), `grid_Pref_Pbess`, `oat_Pref`, `grid_strength`, `wt_frt` (dip at 45 s), `freq_step` (50 s, 70 s runs, with the decentral cases), `hppc_p` (40 s, 120 s), `hppc_qv` (40 s, 90 s), `bess_soc` (120 s), `comm_delay` (Q step, `comm.*` columns).
8. Place events after the plant has settled from the wind step at 20 s and the 36 MW/min ramp (about 40–45 s), and give all cases of a set the same stop time.
9. Set only source fields in cases (`hppc.*`, `pc.*`, `wt.iec.ctrl.*`, ...), never `ctrl.*` or other derived fields.

# Part B. Power system modelling

## B1. The plant and the modelling approach

This chapter describes what physical plant the model represents, how it is reduced to a small number of converter models, which per-unit bases each control level uses, and the modelling choices that everything else rests on: positive-sequence phasor (RMS) simulation, a 1 ms fixed step with `ode1`, the sign conventions and the control hierarchy. It ends with how this structure serves the PhD question.

Sources: `params/windTurbineParameters.m`, `params/pvParameters.m`, `params/batteryParameters.m`, `params/gridParameters.m`, `params/networkParameters.m`, `params/simulationParameters.m`, `params/deriveParameters.m`, `params/controllerParameters.m`, `utilities/build/buildHPP.m` (functions `buildTop`, `buildPlantSub`, `buildHppcSub`, `buildAsset`, `makeLink`), `docs/design.md` sections 1–5, 9 and 11, and the memory note on vendor plant controllers.

### B1.1 The plant

#### What

A hybrid power plant (HPP) with three technologies behind one point of connection (POC):

| Plant | Total rating | Units in reality | Unit rating | Collector voltage | Unit transformer |
|---|---|---|---|---|---|
| Wind power plant (WPP) | 150 MW | 30 turbines | 5 MW (`p.wt.Pn`) | 10 kV feeders, then 10/33 kV wind transformer | 0.69/10 kV, 5.6 MVA per turbine |
| PV power plant (SPP) | 50 MW | 10 inverter stations | 5 MW (`p.pv.Pn`) | 33 kV feeders | 0.6/33 kV, 5.5 MVA per station |
| Battery (BESS) | 30 MW / 60 MWh | 1 unit | 30 MW (`p.bess.Pn`) | short 33 kV cable | 0.69/33 kV, 33 MVA |
| POC | export limit 180 MW (`p.grid.Pmax_poc`) | | | 132 kV (`p.grid.Vpoc`) | HPP transformer 33/132 kV, 250 MVA |

The single-line structure (from `docs/design.md` section 5 and `networkParameters`):

```
wind:  turbine 0.69 kV ─ 0.69/10 kV ─ tap ┐ 10 kV feeders ─ 10 kV wind bus ─ 10/33 kV ─┐
PV:    station 0.6 kV  ─ 0.6/33 kV  ─ tap ┘ 33 kV feeders ─────────────────────────────┤ 33 kV HPP busbar ─ 33/132 kV ─ POC ─ grid
BESS:  unit 0.69 kV    ─ 0.69/33 kV ─ short 33 kV cable ───────────────────────────────┘
```

Installed power is 230 MW against a 180 MW export limit: the plant is over-planted, which is normal for HPPs and is what makes curtailment, the BESS dispatch and the POC export limit relevant control problems.

These sizes were fixed in the review of 2026-11-16 (`docs/design.md` section 9): 150 MW wind (30 × 5 MW since stage 4b, 10 kV collector), 50 MW PV (10 × 5 MW stations), 30 MW / 60 MWh BESS, 180 MW POC export limit, POC at 132 kV, grid code Energinet DK1.

#### Why

The values are *typical*, not from a real project (`docs/design.md`: "Values marked *typical* are starting points to be replaced by project data"). The sizes give a realistic mix: a wind-dominated plant, a smaller PV plant, a BESS large enough to balance losses and give the under-frequency headroom that wind and PV cannot give while running at available power, and an export limit below the installed power.

The 10 kV wind collector with its own 10/33 kV transformer and the PV and BESS directly on the 33 kV busbar give two different plant-connection topologies, so the plant measurement points differ (behind a transformer for wind, at a feeder end for PV and BESS). This matters for the plant controllers' Q loops (chapter B4) and for how much reactive power reaches the POC.

#### Alternatives / tried and dropped

- Stage 2 (commit `54bf4da`) had a simpler network: a 0.69/33 kV unit transformer and a 33 kV cable per asset instance, wind on 33 kV with "2 parallel cables per 75 MW instance". Stage 4b (commit `8a6d58d`) replaced it with the feeder layout per plant and the 10 kV wind collector.
- A branch `layout-1wt-1pv` (commit `ae5559d`) used wind 1 × 1 unit of 30 turbines and PV 1 × 1 unit of 10 stations with `cable.nPar = 2` (3 converter models, 9 nodes) to speed up runs. The merge `61bc052` kept main's default layout (wind 1 × 2, PV 2 × 1, BESS 1 × 1) once Fast Restart made runs fast enough.

#### Consequences

Changing plant sizes means changing `Pn`, `nAgg`, `nFeeders`, `nPerFeeder` in the asset parameter files and the matching transformer and cable data in `networkParameters`. If the number of instances N changes, `HPP.slx` must be rebuilt (`buildHPP(All=true)`), because N Model blocks and fixed-width vector ports are built into it (section B1.2).

### B1.2 Layout parameters and aggregation

#### What

Each asset type has four layout parameters in its parameter file:

| Parameter | Wind (`p.wt`) | PV (`p.pv`) | BESS (`p.bess`) | Meaning |
|---|---|---|---|---|
| `Pn` | 5 MW | 5 MW | 30 MW | rating of one real unit |
| `nAgg` | 15 | 5 | 1 | real units represented by one modelled unit |
| `nFeeders` | 1 | 2 | 1 | modelled feeders |
| `nPerFeeder` | 2 | 1 | 1 | modelled units along each feeder |
| `N` (derived) | 2 | 2 | 1 | asset instances = nFeeders · nPerFeeder |
| `Prated` (derived) | 150 MW | 50 MW | 30 MW | plant rating = N · nAgg · Pn |

and the network has one more per plant, `p.net.<type>.cable.nPar` (wind 15, PV 1, BESS 1): the number of identical parallel feeders one modelled feeder stands for.

`deriveParameters` computes `N` and `Prated`:

```matlab
p.(t).N      = p.(t).nFeeders * p.(t).nPerFeeder;
p.(t).Prated = p.(t).N * p.(t).nAgg * p.(t).Pn;
```

Every modelled unit is one *asset instance*: its own asset model (`WT_asset`, `PV_asset`, `BESS_asset`), containing its own asset controller and its own converter model, at its own network node. `buildPlantSub` places N Model blocks per plant and gives instance k the model argument `ap = p.<type>.ap(k)`.

Aggregation is applied in two places:
- **Unit transformer**: the rating is scaled with `nAgg`. In `networkSolverData`: `Zt = impedance(uk, XR) * Sb / (d.unit.S * q.nAgg)`. The wind unit transformer is 5.6 MVA × 15 = 84 MVA, the PV one 5.5 MVA × 5 = 27.5 MVA.
- **Feeder cable**: the series impedance is divided by `nPar` and the charging multiplied by `nPar`: `Zc = (R + jX)·L / nPar / Zb`, `Bh = ω·C·L·nPar / 2 · Zb`.

The asset model itself does not change with `nAgg`: all asset parameters are in per unit on the instance rating `nAgg · Pn` (75 MW for a wind unit, 25 MW for a PV unit, 30 MW for the BESS), so a 75 MW aggregate and a 5 MW turbine have identical per-unit dynamics.

How the default wind layout should be read: the modelled feeder with two units is 15 identical real feeders of 2 turbines each (about 10 MW per real feeder). Unit 1 represents the 15 turbines at the first position of each real feeder (2 km from the wind bus), unit 2 the 15 turbines at the second position (1 km further). Dividing the cable impedance by 15 is exact for identical feeders carrying identical currents. The PV layout is two real feeders, each with one unit of 5 inverter stations lumped at the first tap (2 km); the 0.5 km spacing between stations (`net.pv.cable.len`) is not used while `nPerFeeder = 1`.

#### Why

A converter model with its controller costs simulation time and memory per instance, and the controllers are separate referenced models. The aim is a realistic plant (full ratings, realistic impedances, realistic losses and voltage drops) with few converter models. Scaling per-unit models by the rating is the standard aggregation approach for wind and PV plants (one equivalent machine behind an equivalent impedance).

Keeping N > 1 per plant in the default layout (wind 2, PV 2) is deliberate: the plant controllers have vector ports of width N and a dispatch over N assets, and with N = 1 that code would never be exercised. Different positions along a feeder (wind units 1 and 2) also give the units different terminal voltages.

`docs/design.md` (section 3, "N assets") records why each instance is a separate Model block made by the build script rather than a For Each subsystem: the build script is simpler to debug, and every instance stays a normal Model block, which code generation and PSCAD handle the same way.

#### What aggregation captures and what it does not

Captured:
- total power, total losses in unit transformers and cables, and the voltage at the unit terminal, for identical units at the same operating point;
- the charging of all parallel cables;
- the different positions along a feeder, as far as the modelled `nPerFeeder` resolves them;
- the per-unit dynamics of the converter and its controller (identical for all represented units).

Not captured:
- differences between the represented units: different wind speeds, wake effects, different terminal voltages within an aggregate, one unit tripping or being out of service. All wind instances get the same available-power profile (`wind k` Step block in `buildPlantSub`: `p.wt.Pavail0` → `p.wt.PavailStep` at `p.wt.tStep`); all PV instances the same constant `p.pv.Pavail`;
- interactions between the controllers of individual units (only between modelled instances);
- the effect of the nonlinear limits acting on some units but not others (a real plant hits current limits unit by unit; an aggregate hits them all at once);
- voltage profiles along the PV feeder (stations lumped at one tap).

The design document quantifies the steady-state error: a full layout (wind 15 feeders × 2 turbines, PV 2 feeders × 5 stations, all `nAgg` and `nPar` = 1: 41 converter models, 85 nodes) gave P_poc 149.46 MW against 149.40 MW for the aggregated layout in a static check with all units at the same power (`docs/design.md` section 3, "Aggregation").

#### Alternatives / tried and dropped

- For Each subsystem for the N instances: not used (reasons above).
- The 1 × 1 layout per plant (branch `layout-1wt-1pv`): faster, but no vector dispatch to test; not kept as the default.
- Per-instance parameters: "not planned, but kept possible" (`docs/design.md` section 9). Each instance already has its own `p.<type>.ap(k)`, so different turbine types in one plant only need different values in `ap(k)`. Today `deriveParameters` fills all instances with the same values (`repmat`).

#### Consequences

- Changing `nAgg` or `nPar` only changes numbers (Y matrix, transformer rating), not the model structure. With Fast Restart this does not even need a recompile, as long as N is unchanged.
- Changing `nFeeders` or `nPerFeeder` changes N, the network node count and the plant controller port widths: rebuild `HPP.slx` with `buildHPP(All=true)`. `runCase` / `runSweep` stop if `p` and the N stored in `HPP.slx` (model workspace variable `buildInfo`) differ. The plant controller DLLs are built for one N.
- When you aggregate, keep `nAgg` (unit transformer) and `nPar` (feeders) consistent with the real layout; they are independent parameters and nothing checks that they describe the same plant.

### B1.3 Per-unit bases at each control level

#### What

Every controller works in per unit on its own power base, and all unit conversions sit outside the controllers, in the "link" blocks of `HPP.slx`.

| Level | Base (`p.ctrl.<c>.Pbase`) | Default value | Set in |
|---|---|---|---|
| HPPC | POC base `p.grid.Pbase` | 180 MW | `controllerParameters`, `ctrl.hppc.Pbase = p.grid.Pbase` |
| Plant controllers WPPC, SPPC, BESSC | plant rating `p.<type>.Prated` | 150 / 50 / 30 MW | `plantCtrl(p.wt.Prated, ...)` |
| Asset controllers WT, PV, BESS | instance rating `nAgg · Pn` | 75 / 25 / 30 MW | `assetCtrl(p.sim.Ts, p.wt.nAgg * p.wt.Pn)` |
| Network (system base) | `p.grid.Pbase` (as MVA) | 180 MVA | `networkSolverData`, `Sb = p.grid.Pbase` |

Voltages are in per unit of the nominal voltage of their level (0.69, 0.6, 10, 33, 132 kV). With ideal transformer ratios (no taps), a per-unit voltage or current is the same on both sides of a transformer, so the network can be solved in one per-unit system. Frequency is in per unit of 50 Hz (`p.grid.f0`). SOC is 0–1 and the saturation flags `Q_sat` are +1 / 0 / −1, without unit.

The `HPP.slx` buses between the subsystems are in SI units (W, var); voltages are already in pu. Each subsystem converts to the base of the controller it contains. The link blocks (`makeLink` in `buildHPP.m`, ~line 1313) carry the conversion as a Gain whose expression references the bases:

| Signal path | Link steps (in order) | Built in |
|---|---|---|
| TSO P_ref, Q_ref → HPPC | `gain 1/p.ctrl.hppc.Pbase`, `delay comm.tso` (initial output = initial set-point in pu) | `buildHppcSub` |
| TSO f_meas → HPPC | none (direct line) | `buildHppcSub` |
| POC P, Q → HPPC | `gain 1/hppc.Pbase`, `memory`, `delay comm.meas_hppc` | `buildHppcSub` |
| POC V → HPPC | `memory`, `delay comm.meas_hppc` (initial 1) | `buildHppcSub` |
| Plant P, Q, P_avail, P_chg_max, P_dis_max → HPPC | `gain 1/hppc.Pbase`, `memory`, `delay comm.meas_hppc` | `buildHppcSub` |
| SOC, Q_sat → HPPC | `memory`, `delay comm.meas_hppc` | `buildHppcSub` |
| HPPC P_set, Q_set → set-point bus | `gain hppc.Pbase` (pu → W / var) | `buildHppcSub` |
| set-point bus → plant controller | `delay comm.hppc_to_plant`, `gain 1/<plant>.Pbase` | `buildPlantSub` |
| Plant P, Q at the busbar (network) → plant controller | `gain 1/<plant>.Pbase`, `memory`, `delay comm.meas_plant` | `buildPlantSub` |
| Busbar V → plant controller | `memory`, `delay comm.meas_plant` (initial 1) | `buildPlantSub` |
| Asset P_avail_k → plant controller | `gain <asset>.Pbase/<plant>.Pbase`, `memory`, `delay comm.meas_plant` | `buildPlantSub` |
| Asset SOC_k → BESSC | `memory`, `delay comm.meas_plant` | `buildPlantSub` |
| Grid f → plant controller | `memory` | `buildPlantSub` |
| Plant controller P_set_k, Q_set_k → assets | `delay comm.plant_to_asset`, `gain <plant>.Pbase/<asset>.Pbase` | `buildPlantSub` |
| Plant controller P, Q, P_avail, ... → measurement bus | `gain <plant>.Pbase` (pu → W) | `buildPlantSub` |
| Asset current I_re, I_im → network | `gain <asset>.Pbase/p.grid.Pbase` (asset pu → system pu) | `buildPlantSub` |
| Network P, Q (system pu) → buses | `gain net.Sb` (→ W / var) | `buildNetwork` |

The mask icon of each link shows its steps, for example `to pu | mem | comm.meas_plant`; `makeLink` labels a gain `1/x` as "to pu", `x/y` as "pu base" and `x` as "to SI".

The asset current conversion uses the fact that, with ideal ratios, the per-unit current on the asset base times the asset base equals the per-unit current on the system base times the system base: $i_{sys} = i_{asset} \cdot S_{asset}/S_{sys}$.

#### Why

`docs/design.md` section 4: every controller follows the same pattern — per unit on its own base, scalar or fixed-width `double` ports, tunable parameters in one struct `par`, fixed `Ts`. The reasons:

- **Code generation and PSCAD.** Every controller is a future IEEE/CIGRE DLL. A DLL in per unit on its own rating can be reused for a plant of another size by changing the base outside it. The W/var ↔ pu conversions sit outside the controllers: in `HPP.slx` in Simulink, and on the PSCAD side in PSCAD. Commit `467b5f2` introduced this interface ("Per-unit, tunable controller interface for code generation").
- **Realism.** A vendor plant controller knows its own plant rating, not the HPP's; an asset controller works on its converter rating; IEC 61400-27-1 and WECC parameters are given in pu on the unit rating, so the standard tables can be checked directly. The HPPC works on the POC base because the grid code requirements are expressed on Pn at the POC (Pn = 180 MW in Energinet's terms, `docs/design.md` stage 5 step 3).
- **One system base for the network** equal to the POC base keeps the network output directly comparable with the HPPC's quantities.

#### Alternatives

The model before stage 1 had one controller in W. The per-unit interface (`467b5f2`) matched the earlier results to within 4e-9 W.

#### Consequences

- When a controller gets a new power input or output, the conversion belongs in a link block in `buildHPP.m`, not inside the controller.
- The bases are fixed at build time in the sense that they are not part of `par` (`Ts` and `Pbase` are "fixed settings", `controllerParameters` header). They are still evaluated from `p` when the model compiles, so a plant size change works without editing the controllers.
- The HPPC splits Q over the plants by the rating share `qshare = Prated / sum(Prated)` (150/230, 50/230, 30/230), computed in `controllerParameters`.

### B1.4 The phasor (RMS) modelling approach

#### What

The whole model is a positive-sequence phasor ("RMS") model:
- network voltages and currents are complex phasors at 50 Hz in per unit; the network is algebraic (no electromagnetic states): $V = Z \cdot I$ every step (chapter B2);
- each converter is a controlled current source whose phasor is aligned with its terminal voltage: $I = (i_p - j\,i_q) \cdot V/|V|$, so $P = |V|\,i_p$ and $Q = |V|\,i_q$ (function `generator` in `buildAsset`);
- the converter's inner current control is represented by first-order lags (`Tg`) and rate limits on $i_p$, $i_q$ (IEC Type 4 generator system, WECC REGC_A);
- grid frequency is a signal (a Step) given to the controllers; the network itself is always solved at 50 Hz.

`docs/design.md` section 1: "Simulation type in Simulink: **RMS (positive-sequence phasor)**. EMT validation is done in PSCAD." Specialized Power Systems is not used.

#### Why

- **Purpose.** The model is for studying the control hierarchy: set-point tracking, frequency response, Q/V control, coordination between the HPPC and closed-loop plant controllers, FRT behaviour at the controller level. These dynamics run from tens of milliseconds to minutes. Electromagnetic transients, switching and harmonics are much faster and do not change the control interactions under study.
- **Speed.** Many cases are needed (case sets of up to 16 cases, sweeps). A 60 s case runs in about 1.4–2.2 s in Rapid Accelerator with the nodal phasor network (`docs/design.md` section 13); an EMT model of a 230 MW plant would take orders of magnitude longer (inferred: an EMT model needs time steps of tens of microseconds).
- **Standard models are RMS models.** IEC 61400-27-1 and the WECC generic models are defined for positive-sequence phasor simulation, so using them in an RMS framework is their intended use.
- **EMT validation elsewhere.** The controllers are built to be code-generated as DLLs and run in PSCAD against PSCAD's own EMT plant models (everything "below the dashed line" in the hierarchy diagram is never code-generated, `docs/design.md` section 2). EMT detail is therefore obtained by swapping the plant side, not by making the Simulink model EMT.

#### Limitations (what an RMS model cannot show)

- Unbalanced faults and negative-sequence behaviour (positive sequence only).
- Harmonics, converter switching, fast current-controller and PLL dynamics: the converter's terminal voltage angle is taken from the previous step "as a PLL would" (chapter B2), with no PLL dynamics.
- Network electromagnetic transients: cable charging and transformer magnetizing are constant admittances at 50 Hz. The instantaneous network with one-step-delayed current sources gives single-step spikes at fault inception and clearing (chapter B2, B2.8).
- Frequency-dependent network: a frequency step changes only the controllers' input, not the network reactances or the phasor reference.

#### Alternatives

- Specialized Power Systems (phasor mode): not used (`docs/design.md` section 1); no reason is recorded (inferred: it would add another library on top of Simscape Electrical, and its phasor mode is less transparent for the code-generation path than an own solver).
- Simscape Electrical in frequency-and-time mode: built as the second network model (chapter B2.6). It is also a phasor-like solution (phasors around 50 Hz from three-phase components), not full EMT.
- EMT in Simulink: not considered for this purpose; EMT is PSCAD's role.

### B1.5 Time step and solver: 1 ms, `ode1`

#### What

`simulationParameters`: `sim.Ts = 1e-3` s (base fixed step), `sim.Tstop = 60` s (case sets often set 70 or 90 or 120 s), `sim.logDecimation = 10` (log every 10 ms).

`buildTop` sets on `HPP.slx`: `SolverType` Fixed-step, `Solver` `ode1` (forward Euler, one derivative evaluation per step), `FixedStep` `p.sim.Ts`, `SolverMode` SingleTasking, `Decimation` `p.sim.logDecimation`, `AlgebraicLoopMsg` `error`.

The continuous states in the model are few: the converter current lags (`contLag`: gain 1/Tg and an Integrator), and the BESS SOC integrator, all in the asset models. The controllers are discrete (`FixedStepDiscrete` in their own models, sample time `Ts`). The asset controllers run at the simulation step (`assetCtrl(p.sim.Ts, ...)`), the plant controllers at 20 ms, the HPPC at 100 ms (chapter B6). `controllerParameters` asserts that every controller `Ts` is an integer multiple of `sim.Ts`.

#### Why

- **1 ms.** The asset controllers represent the converters' fast control and run at the simulation step. The fastest relevant time constants are the converter lags (`Tg` 20 ms in the WECC data) and the voltage filters (`Trv` 20 ms), so 1 ms gives at least 20 steps per time constant, enough for forward Euler to be accurate (inferred). The simulation parameter file gives the PSCAD analogy: "The controller sample time (p.ctrl.Ts) must be an integer multiple of sim.Ts. This mirrors PSCAD, where the controller DLL runs at a multiple of the EMT time step." Communication delays are whole multiples of the step (chapter B6), so 1 ms is also the delay resolution.
- **`ode1` instead of `ode4`** (`docs/design.md` section 11): one evaluation per step instead of four. With so few continuous states and a 1 ms step it costs nothing in accuracy. Together with discrete delays and logging every 10 ms this made runs about 15 % faster (commit `ae5559d`: about 90 s instead of 105 s per run at the time), steady state unchanged, dip values within 0.7 MW.
- **Single-tasking.** All rates run in one task, so the 1 ms links can feed the 20 ms and 100 ms controllers without Rate Transition blocks (inferred: in single-tasking mode Simulink accepts direct connections between rates; in multitasking it would demand rate transitions for data integrity).
- **`AlgebraicLoopMsg = error`.** Any algebraic loop stops the build instead of being solved iteratively (slow and fragile in fixed step). See chapter B6.4.

#### Alternatives / tried and dropped

- Making the asset models fully discrete (discrete current lags and SOC): no speed gain, and P during a dip moved by 2–3 MW, so it was not kept (`docs/design.md` section 11).
- A smaller step would remove the single-step spikes at dip clearing (`docs/design.md` open points), at proportionally higher run time. Not done.

#### Consequences

- `sim.Ts` is a compile-time value (part of the compile signature for Fast Restart and the Rapid Accelerator target, chapter A5). Changing it changes the asset controller rate and every delay length in steps.
- Every controller `Ts` and every delay must be a multiple of `sim.Ts` (`controllerParameters` asserts the first; `deriveParameters` rounds the delays to whole steps).
- The asset models store `ode4` as their own solver for standalone use (`buildAsset`), but as referenced models they run with the top model's `ode1`.

### B1.6 Sign conventions

#### What

`docs/design.md` section 4: "generator convention, + P = export / discharge, + Q = export (capacitive). SOC in 0–1."

| Quantity | Positive means | Where it is defined |
|---|---|---|
| Asset P, Q | delivered by the asset into the network | `generator`: $P = |V|\,i_p$, $Q = |V|\,i_q$ with $I = (i_p - j\,i_q)V/|V|$ |
| Plant P, Q (`P_plant`, `Q_plant`) | flowing from the plant into the 33 kV HPP busbar | `solve` in `buildNetwork`: $S_{br} = V_b \cdot \overline{I_{br}}$ with $I_{br}$ into the busbar |
| POC P, Q | flowing into the grid | `solve`: $S_{poc} = V_p \cdot \overline{(V_p - E)/Z_g}$ |
| BESS P | discharge (export) | `batteryParameters` ("positive power = discharging"); SOC: $dSOC/dt = -k_{SOC} \cdot P$ |
| Q | export, i.e. capacitive / over-excited behaviour, raising the voltage | |
| Q_sat | +1 at the upper limit, −1 at the lower limit | plant controllers |
| Frequency | pu of 50 Hz (1 = nominal) | |

With $I = (i_p - j i_q)\,e^{j\theta}$ and $V = |V| e^{j\theta}$: $S = V\bar I = |V|(i_p + j i_q)$, so positive $i_q$ gives positive Q (export).

The SOC equation in `BESS_asset`: Gain `-ap.kSOC`, Integrator with initial condition `ap.SOC0`, where `kSOC = p.ctrl.bess.Pbase / Eunit` (`deriveParameters`), `Eunit = p.bess.E / p.bess.N`. With 30 MW and 60 MWh: $k_{SOC} = 30\cdot10^6 / 216\cdot10^9 \approx 1.39\cdot10^{-4}$ /s per pu, i.e. 1 pu for 2 h empties the battery. No efficiency or self-discharge is modelled.

#### Why

The generator convention is natural for a generating plant and matches IEC 61400-27-1, WECC and grid code texts. Positive BESS power = discharge keeps the BESS consistent with the other generators. Q positive = export is the grid code convention (Energinet's PF "export" and "import").

#### Consequences

When adding a load-like element (STATCOM, auxiliary consumption), keep the generator convention at its port. The PF control uses `PFsign` (+1 export, −1 import, chapter B5).

### B1.7 The control hierarchy

#### What

```
 TSO / market:  P_ref, Q_ref (V_ref and PF as HPPC parameters for now), frequency
        │                                                        (comm.tso 200 ms)
 HPPC  (HPP controller, POC base 180 MW)                         Ts = 100 ms
        │  P/Q set-points per plant (comm.hppc_to_plant 50 ms)   ▲ plant P, Q, P_avail, SOC, Q_sat; POC P, Q, V
   ┌────┴──────────────┬───────────────────────┐                   (comm.meas_hppc 20 ms)
 WPPC               SPPC                     BESSC               Ts = 20 ms
 (WECC REPC_A)      (WECC REPC_A)            (WECC REPC_A + SOC)
   │ P/Q per asset (comm.plant_to_asset 20 ms)    ▲ P_avail_k, SOC_k; plant P, Q, V_bus (comm.meas_plant 20 ms)
 WT controller ×N    PV controller ×N         BESS controller ×N  Ts = 1 ms (= sim step)
 (IEC 61400-27-1     (WECC REEC_B)            (WECC REEC_C)
  Type 4A control)
   │ i_p, i_q commands (local, no delay)
 ── plant side (not DLLs) ────────────────────────────────────────────────
 WT generator ×N     PV converter ×N          BESS converter + battery ×N
 (IEC Type 4 gen.)   (WECC REGC_A)            (WECC REGC_A + SOC)
   └── unit transformers ── feeders ── 33 kV busbar ── 33/132 kV ── POC ── grid
```

Each controller is a separate referenced model (`models/ctrl/*.slx`) and a future DLL. The asset models (`models/assets/*.slx`) each contain one asset controller instance plus the converter model. `HPP.slx` holds six subsystems: TSO, HPP controller, Wind / PV / BESS plant, Network, Logging (`buildTop`).

Measurement points (chapter B2.5): the HPPC measures P, Q, V at the POC; each plant controller measures its plant's P and Q where the plant feeds the 33 kV HPP busbar, and the busbar voltage; each asset controller measures its own terminal voltage and its own P, Q.

Why the BESS has its own asset controller (`docs/design.md` section 2): the plant controller (EMS) decides how much power over minutes to hours and manages SOC; the asset controller decides how it is delivered (current control, converter and BMS limits). Different time scales and functions, and it keeps the three asset types structurally the same — in WECC terms, REPC (plant) and REEC_C (asset).

Since stage 5 the default is **central control** for P and frequency: the HPPC closes the P loop at the POC and gives the frequency response of the whole plant; the plant controllers pass their P set-points through (`Frqflag` = 0) but keep their own Q / V loops closed. Voltage control is a cascade: plant controllers control the busbar voltage with a droop, the HPPC slowly trims their offsets (chapters B4, B5). The stage-4 setup with each plant's own droop and P loop remains as the `decentral` cases.

### B1.8 Relation to the PhD question

#### What and why

Memory note "phd-vendor-plant-controllers" (decided 2026-11-16) and `docs/design.md` stage 4 ("Closed loops at every level"): every plant controller (WPPC, SPPC, BESSC) has full closed loops in WECC REPC_A style — P, Q/V, frequency droop — on its own measurement, and the HPPC is an outer loop at the POC ("option 2, state of the art HPP").

The reason: in real HPPs, vendors supply their assets together with their own plant controller. The HPPC must coordinate these black-box, closed-loop controllers: cascaded loops interacting, parallel voltage control by several plants on one busbar, duplicated frequency response (HPPC and plant both reacting), competing integrators. This coordination problem is what the PhD project addresses.

#### Consequences for the model

- The plant controllers are kept realistic and self-contained, like vendor DLLs with their own loops, flags and measurement points. Interactions with the HPPC are not "simplified away"; they are made visible and testable in case sets (e.g. `freq_step` central vs. decentral, `hppc_qv` cascade, anti-windup over plant limits via `Q_sat`).
- Communication delays and different controller rates between the levels are part of the model from stage 1, because they shape these interactions (chapter B6).
- The open question in `docs/design.md` section 10: how central HPPC loops interact with plant controllers that keep their own P and frequency loops (decentral).

### Recreating it

1. Fix the plant data: ratings, unit ratings, voltages, POC export limit, grid code (Energinet DK1).
2. Write one parameter function per asset type with `Pn`, `nAgg`, `nFeeders`, `nPerFeeder`, and derive `N = nFeeders·nPerFeeder` and `Prated = N·nAgg·Pn` in `deriveParameters`.
3. Choose the bases: HPPC on the POC base (`p.grid.Pbase` = 180 MW), plant controllers on `Prated`, asset controllers on `nAgg·Pn`, network system base = POC base, voltages on nominal per level.
4. Put all W ↔ pu conversions in link blocks between the levels; asset currents to the network with gain asset base / system base.
5. Model each converter as a current source aligned with the previous-step terminal voltage, $I = (i_p - j i_q)V/|V|$.
6. Set the top model to fixed step 1 ms, `ode1`, single-tasking, logging decimation 10, algebraic loops as errors.
7. Use the generator convention throughout; BESS + = discharge; SOC integrator with $-k_{SOC}$.
8. Build the hierarchy TSO → HPPC (100 ms) → plant controllers (20 ms) → asset controllers (1 ms) → converter models, each controller a separate referenced model with `par` as model argument.
9. Keep the plant controllers closed-loop and self-contained (vendor-like); make every HPPC–plant interaction testable in a case set.

---

## B2. Network and grid

This chapter covers the electrical network between the converters and the grid: the nodal phasor solver (default), how the converters couple to it, the measurement points, the grid Thevenin equivalent and the grid events. It then covers the alternative Simscape Electrical network in frequency-and-time mode, its validation against the nodal model, and the known numerical artefacts.

Sources: `params/networkParameters.m`, `params/gridParameters.m`, `params/networkSolverData.m`, `params/deriveParameters.m`, `utilities/build/buildHPP.m` (functions `buildNetworkVariants` ~line 2410, `buildNetwork` ~line 367, `buildNetworkSimscape` ~line 2439, `trBlock`, `piBlock`, `meterBlock`, `vMeterBlock`, `convBlock`, `phasorBody`, `buildAsset`, `buildTSO`), `scripts/networkCode.m`, `experiments/simscapeGrid/ftGridTest.m`, `docs/design.md` sections 5, 8, 10, 11, 13, commits `54bf4da`, `8a6d58d`, `08685a1`, `fb180ee`, the memory note "grid-simscape-ft".

### B2.1 Overview: two network models in one Variant Subsystem

#### What

The `Network` block in `HPP.slx` is a Variant Subsystem (`buildNetworkVariants`). A *Variant Subsystem* holds several alternative implementations ("choices") with the same ports; a variant control expression selects which one is active. The two choices:

| Choice | Variant control | Implementation |
|---|---|---|
| `Nodal` | `HPP_network == 1` | own nodal phasor solver, $V = Z\,I$ in one MATLAB Function block (`buildNetwork`) |
| `Simscape` | `HPP_network == 2` | standard Simscape Electrical three-phase blocks in frequency-and-time mode (`buildNetworkSimscape`) |

`VariantActivationTime` is "update diagram": only the active choice is compiled; switching needs no rebuild of `HPP.slx`. The variable `HPP_network` comes from `networkCode(p.net.solverType)` ("nodal" → 1, "simscape" → 2) and is set per run by `runCase(..., Network="nodal" | "simscape")`, `runSweep(..., Network=...)` and the `network` setting in `main.m`. The default is `p.net.solverType = "nodal"` (commit `08685a1`); `buildTop` also stores `HPP_network` in the model workspace for runs started from the editor.

Ports (same for both):
- inputs 1–3: current injection buses per plant (`I_re`, `I_im`, vectors of width N, system pu);
- output 1 `network` bus: `V_re`, `V_im` (all asset terminals, one step old), `P_plant`, `Q_plant` (3 × 1, W and var), `V_bus` (pu), `f` (pu);
- output 2 `POC meas` bus: `P_poc` (W), `Q_poc` (var), `V_poc` (pu).

#### Why two models

The nodal solver is fast and is what all case sets use. The Simscape model is component-wise (transformer, cable, source blocks you can see), it independently validates the hand-coded nodal model, and it is the starting point for a planned component-based grid (section B2.6).

### B2.2 The nodal solver

#### B2.2.1 Node ordering

`networkSolverData(p)` (`params/networkSolverData.m`) builds the network from the layout parameters. Asset order (global index k): wind, PV, BESS; within a plant feeder by feeder; along each feeder from the collector bus outwards.

| Nodes | Meaning |
|---|---|
| 1 … nA | asset terminals (LV side of each unit transformer) |
| nA+1 … 2nA | feeder taps (MV side of each unit transformer); tap of asset k is node nA+k |
| 2nA+1 … | plant collector buses, one per plant with a plant transformer (`plant.S > 0`; only wind by default) |
| nNodes − 1 (`s.bus`) | 33 kV HPP busbar |
| nNodes (`s.poc`) | POC (132 kV) |

`nNodes = 2·nA + (number of plant transformers) + 2`. Default layout: nA = 5, one plant transformer, so 13 nodes (asset terminals 1–5, taps 6–10, wind collector 11, HPP busbar 12, POC 13). The full layout of B1.2 has 85.

#### B2.2.2 Y-matrix construction

The admittance matrix is assembled branch by branch with two helpers:

```matlab
function Y = addSeries(Y, a, b, Z)       % series branch a-b
    y = 1 / Z;
    Y(a,a) = Y(a,a) + y;  Y(b,b) = Y(b,b) + y;
    Y(a,b) = Y(a,b) - y;  Y(b,a) = Y(b,a) - y;
end
function Z = impedance(zmag, XR)          % |Z| and X/R -> complex Z
    R = zmag / sqrt(1 + XR^2);  Z = complex(R, R*XR);
end
```

Per plant t (loop over wind, PV, BESS):

1. Collector voltage base: $Z_b = V_{col}^2 / S_b$ (Ω), $S_b$ = 180 MVA.
2. If the plant has a plant transformer (wind): a new collector node `col`; series impedance $Z_p = z(u_k, X/R)\cdot S_b/S_{plant}$ between `col` and the busbar; magnetizing shunt at `col` (the collector side). This branch is the plant's measurement branch (`brFrom = col`, `brB = 0`). Otherwise `col = bus`.
3. Unit transformer impedance $Z_t = z(u_k, X/R)\cdot S_b/(S_{unit}\cdot n_{Agg})$.
4. For each feeder f and each unit i along it (asset index k):
   - unit transformer between terminal k and tap nA+k: `addSeries(Y, k, nA+k, Zt)`;
   - feeder segment (π) from the previous node `prev` (collector bus for i = 1, previous tap otherwise) to tap nA+k, length `len1` for the first segment, `len` for the others:
     $Z_c = (R + jX)\,L/n_{Par}/Z_b$, half charging $B_h = \omega C L\, n_{Par}/2 \cdot Z_b$ added at both ends;
   - if i = 1 and there is no plant transformer (PV, BESS), this first segment is a plant measurement branch (`brFrom = nA+k`, `brZ = Zc`, `brB = Bh`).
5. HPP transformer between busbar and POC: $Z_m = z(u_k, X/R)\cdot S_b/S_{main}$, magnetizing shunt at the busbar (33 kV side).
6. Grid impedance at the POC as a shunt to the (ideal) source: $Z_g = z(1, X/R)\cdot S_b/(SCR\cdot P_{max,poc})$, `Y(poc,poc) += 1/Zg`.

Then `Z = inv(Y)` is computed once, and `networkSolverData` returns real arrays (so they can be block parameters):

| Field | Content |
|---|---|
| `nA`, `nNodes`, `bus`, `poc` | sizes and node indices |
| `plantOf` | plant (1 wind, 2 PV, 3 BESS) of each asset |
| `Zre`, `Zim` | real and imaginary part of Z (nNodes × nNodes) |
| `brFrom`, `brZre`, `brZim`, `brB`, `brPlant` | plant measurement branches: from-node, series impedance, bus-end half charging, plant index |
| `Zg_re`, `Zg_im` | grid impedance |
| `E` | grid source voltage `p.grid.E` |
| `Sb` | system base (VA) |

`deriveParameters` stores this as `p.net.solver`, so it is recomputed whenever a case changes a parameter (`applyCase` calls `deriveParameters`).

#### B2.2.3 Default element values

From `networkParameters` and `gridParameters`, with the per-unit values on 180 MVA computed by the formulas above (computed by hand for this handbook from the code; not logged by the model):

| Element | Data | Per unit on 180 MVA |
|---|---|---|
| Wind unit transformer (per unit of 15 turbines) | 0.69/10 kV, 5.6 MVA × 15, u_k 7 %, X/R 7 | 0.0212 + j0.1485 |
| Wind feeder, first segment | 10 kV, R 0.05, X 0.095 Ω/km, C 0.5 µF/km, 2 km, nPar 15 | 0.0120 + j0.0228; B/2 = 0.00131 |
| Wind feeder, second segment | 1 km | 0.0060 + j0.0114; B/2 = 0.00065 |
| Wind transformer 10/33 kV | 170 MVA, u_k 10 %, X/R 30, Rm 1000, Xm 500 pu | 0.0035 + j0.1058; shunt 0.00094 − j0.00189 |
| PV unit transformer (per unit of 5 stations) | 0.6/33 kV, 5.5 MVA × 5, u_k 6 %, X/R 7 | 0.0555 + j0.3888 |
| PV feeder, first segment | 33 kV, R 0.10, X 0.12 Ω/km, C 0.25 µF/km, 2 km, nPar 1 | 0.0331 + j0.0397; B/2 = 0.00048 |
| BESS unit transformer | 0.69/33 kV, 33 MVA, u_k 8 %, X/R 8 | 0.0541 + j0.4330 |
| BESS cable | 33 kV, R 0.05, X 0.10 Ω/km, C 0.35 µF/km, 0.3 km | 0.0025 + j0.0050; B/2 = 0.0001 |
| HPP transformer 33/132 kV | 250 MVA, u_k 12 %, X/R 40, Rm 1000, Xm 500 pu | 0.0022 + j0.0864; shunt 0.00139 − j0.00278 |
| Grid | SCR 10 on 180 MW (S_sc = 1800 MVA), X/R 10, E = 1.0 pu | 0.0100 + j0.0995 |

Notes:
- The magnetizing shunt is $y_m = (1/R_m - j/X_m)\cdot S/S_b$ (function `magnetizing`): Rm = 1000 pu is 0.1 % no-load loss, Xm = 500 pu 0.2 % magnetizing current, "typical for large power transformers" (`networkParameters` comment).
- PV and BESS have `plant.S = 0` (no plant transformer); their `Rm`, `Xm` fields are placeholders.
- The HPP transformer reactance 0.086 pu on 180 MVA is the value the voltage-control cascade subtracts from the POC slope (`controllerParameters`, chapter B5).
- Tap changers and reactive compensation (STATCOM, shunt reactors) are not modelled (open point; "can be added later as extra nodes or branches").

#### B2.2.4 The solve block

`buildNetwork` creates a masked subsystem with one MATLAB Function block `solve`. Its body (abridged):

```matlab
Z  = complex(Zre, Zim);  nA = numel(I_re);  n = size(Z, 1);
Zg = complex(Zg_ri(1), Zg_ri(2));
I = complex(zeros(n,1), zeros(n,1));
I(1:nA) = complex(I_re(:), I_im(:));
I(net.poc) = I(net.poc) + E / Zg;     % grid Norton source at the POC
V = Z * I;
```

followed by the measurements (B2.5). The grid Thevenin source $E$ behind $Z_g$ is turned into a Norton source $E/Z_g$ injected at the POC, with $1/Z_g$ already in Y. Only the asset terminal nodes and the POC have injections; all other nodes have zero injection, so the full $V$ of all nodes comes out of one matrix–vector product.

**Tunability.** The mask parameter `net` (= `p.net.solver`) is tunable, but inside the MATLAB Function the parameter `net` is set non-tunable (`d.Tunable = false`): it provides only the *structure* (node indices `poc`, `bus`, `brFrom`, `brPlant`, `nA`). The *numbers* enter as inputs from Constant blocks: `Zre`, `Zim`, `[Zg_re, Zg_im]`, `brZre`, `brZim`, `brB`, and `E` comes from the dip logic (B2.7). Constant block values are tunable, so grid strength (SCR, X/R), E, cable, transformer and unit data change between runs with Fast Restart or Rapid Accelerator without recompiling (commit `fb180ee`). Fixed at compile time: the node structure, i.e. the layout.

#### Why a precomputed Z = inv(Y)

- The network is linear and constant during a run (no taps, no switching). Inverting Y once and doing one complex matrix–vector product per step (13 × 13 by default) is the cheapest possible solution.
- It is plain Simulink: no Simscape licence or solver, compiles in Rapid Accelerator and as C code.
- Before stage 4b, a hand-calculated load flow matched the simulation exactly (commit `54bf4da`: at 150 MW injected, P_poc 146.4 MW, 2.4 % losses, Q_poc −21.3 Mvar, V_poc 0.993 pu, V_bus 0.987 pu, "identical in simulation").

#### Alternatives / tried and dropped

- **Whole network data as one tunable parameter structure** (instead of Constant blocks): every run became 0.5–0.75 s per simulated second slower; dropped (`docs/design.md` section 11, commit `fb180ee`).
- **Solving Y·V = I each step** (LU) instead of storing Z: not recorded as tried (inferred: no benefit for a constant small network).
- **Simscape** as the default: slower (B2.6); the nodal model became the default in `08685a1`.

#### Consequences

- A switching event inside the network (a breaker opening, a tap change) would need a new Z: either recompute Z inside the solve block when the event happens, or precompute several Z. Today the only network event is the source voltage E.
- Adding an element (STATCOM, shunt reactor, line) means adding nodes/branches in `networkSolverData`, and possibly injections in `solve` and a new port; for a current-injecting device, add it as an extra injection node like an asset.
- Because only the structure is fixed, a sweep over SCR, X/R, cable length, transformer data or `nAgg`/`nPar` runs on one compiled model.

### B2.3 Converter coupling: current sources with a one-step delay

#### What

Each asset model receives the terminal voltage phasor `V_re`, `V_im` of its node and returns a current phasor `I_re`, `I_im` (asset pu). In `buildAsset`, function `generator`:

```matlab
v  = max(sqrt(V_re^2 + V_im^2), 1e-3);
cr = V_re / v;  ci = V_im / v;
I_re = ip*cr + iq*ci;     % I = (ip - j iq) * V/|V|
I_im = ip*ci - iq*cr;
P = v * ip;  Q = v * iq;
```

The voltage it uses is the network's voltage of the **previous step**: in the network output section, `V_re` and `V_im` pass through Memory blocks (`V_re prev` initial condition `ones(nA,1)`, `V_im prev` initial `zeros(nA,1)`, i.e. a flat start at 1∠0) before they go onto the network bus. The plant subsystem selects each asset's element (Selector by global index), the asset computes its current, the link converts it to system pu (gain asset base / 180 MVA), and the network uses it in the same step.

Also inside the asset: `V_term = |V|` (MATLAB Function `terminal`) goes to the asset controller, and the asset's P and Q pass through Memory blocks (`P meas`, `Q meas`) before the controller.

#### Why

- **No algebraic loop.** The network is direct feedthrough ($V$ depends on $I$ in the same step) and the generator is direct feedthrough ($I$ depends on $V$ in the same step through the alignment $V/|V|$ and the $1/|V|$ in the controllers). Without a delay, $V = Z\,I(V)$ is an algebraic loop that Simulink would have to solve iteratively each step (and `AlgebraicLoopMsg` is `error`). The Memory block breaks it.
- **Physical analogue.** A converter's PLL tracks the voltage angle with a delay; using the previous step's angle "as a PLL would" (`docs/design.md` section 5) is a minimal PLL model. At 1 ms, the angle change per step in normal operation is negligible.
- **Simplicity.** A current-source converter with $P = |V| i_p$, $Q = |V| i_q$ is exactly the interface assumed by WECC REGC_A and the IEC Type 4 generator system.

#### Consequences

- During fast voltage changes (dip inception and clearing), the current for one step is aligned to and scaled by the old voltage: this gives single-step spikes (B2.8).
- The terminal voltage seen by the asset controller is one step old; P and Q measured by the asset controller also one step old. At 1 ms this is negligible for the control.
- The small floor `1e-3` on |V| avoids division by zero at zero voltage.

### B2.4 Transformers

#### What

In the nodal model every transformer is a series impedance from $u_k$ and X/R on its rating, at ideal ratio (per-unit voltages equal on both sides), plus, for the plant and HPP transformers, a magnetizing shunt from `Rm`, `Xm`:

| Transformer | Series | Magnetizing shunt placed at |
|---|---|---|
| Unit transformers | $z(u_k, X/R)\cdot S_b/(S\cdot n_{Agg})$ | none |
| Wind transformer 10/33 kV | $z(u_k, X/R)\cdot S_b/S$ | collector (10 kV) side |
| HPP transformer 33/132 kV | $z(u_k, X/R)\cdot S_b/S$ | busbar (33 kV) side |

#### Why the magnetizing branch

The Simscape model in frequency-and-time mode needs a finite magnetizing branch when an ideal current source feeds a transformer (memory note: "Two-Winding Transformer (Three-Phase) needs the magnetizing branch (Xm, Rm included) when an ideal current source feeds it"). To make both models describe the same transformers, the nodal model got the same branch as a shunt. Before that, P_poc differed by 0.84 MW between the models: the magnetizing loss with 500 pu (`docs/design.md` section 5). The magnetizing shunt sits at one side (an approximation of the T-equivalent, inferred), so for the wind transformer its loss is upstream of the wind plant measurement point.

With the magnetizing branch, the nominal P_poc dropped from 146.95 MW (stage-4b layout without branch) to 146.525 MW (`docs/design.md` section 5, "Current results").

#### Consequences

Unit transformers have no magnetizing branch, and in the Simscape model they are series R-L blocks, not transformer blocks (B2.6), so the two models agree. No tap changers: adding one would change the ratio and therefore Y (and would need per-step Z updates or an off-nominal-ratio π model).

### B2.5 Measurement points: plant, POC, busbar, frequency

#### What

After $V = Z I$, the solve block computes (abridged):

```matlab
Vb  = V(net.bus);
Ibr = (V(net.brFrom) - Vb) ./ complex(brZre, brZim) - 1i * brB .* Vb;
Sbr = Vb .* conj(Ibr);                 % into the busbar, per plant branch
P_plant(brPlant(j)) += real(Sbr(j));  Q_plant(brPlant(j)) += imag(Sbr(j));
V_bus = abs(Vb);
Vp = V(net.poc);
Spoc = Vp * conj((Vp - E) / Zg);       % power into the grid
P_poc = real(Spoc);  Q_poc = imag(Spoc);  V_poc = abs(Vp);
```

| Output | Definition | Goes to |
|---|---|---|
| `P_plant`, `Q_plant` (3 × 1) | sum of the flows of each plant's branches at the busbar end: wind — its 10/33 kV transformer (33 kV side); PV — its feeders' first segments, including the bus-end half charging; BESS — its cable, including bus-end charging | each plant controller (its own element via a Selector), logging |
| `V_bus` | |V| at the 33 kV HPP busbar | plant controllers (`V_meas`), logging |
| `P_poc`, `Q_poc`, `V_poc` | power from the POC into the grid branch, |V| at the POC | HPPC, logging |
| `f` | Step from 1 to 1 + `p.grid.fev.df` at `p.grid.fev.t0` (not computed from the network) | plant controllers |
| `V_re`, `V_im` | all asset terminal voltages, one step old | assets |

Powers come out in system pu and are converted to W / var with gain `net.Sb` (the network bus is in SI units).

#### Why

These are the measurement points of a real HPP: the HPPC meter at the POC, the plant controller meters on the plant's feeder(s) into the HPP busbar (which is where a vendor's plant controller would measure in a shared substation), the busbar voltage for the plant voltage control, and each converter's terminal. Including the bus-end charging in the PV/BESS branch flow makes the measured value equal to what a meter at the busbar end of the cable would read.

#### Consequences

- The plant Q loops close at the busbar feeder; Q losses between there and the POC (mainly the 33/132 kV transformer) are left for the HPPC (Q_poc −10.9 Mvar with the plant loops closed and the HPPC loop open, `docs/design.md`).
- To move a measurement point, change `brFrom`/`brB` in `networkSolverData` (and the meter position in the Simscape model).
- Frequency is not a network quantity; the network has no frequency dynamics (B2.7).

### B2.6 The Simscape Electrical alternative (frequency-and-time mode)

#### What

`buildNetworkSimscape` builds the same network from standard Simscape Electrical (`ee_lib`) three-phase blocks, solved in **frequency-and-time** mode: the Solver Configuration block has `EquationFormulation = NE_FREQUENCY_TIME_EF`, a local Backward Euler solver at `p.sim.Ts`, `DoFixedCost = on`, `MaxNonlinIter = 1`. In this mode Simscape solves for phasors around a nominal frequency (50 Hz) rather than instantaneous waveforms, so it can run at 1 ms.

Component mapping (from the code):

| Element | Simscape implementation |
|---|---|
| Converter (one subsystem per asset, `convBlock`) | Controlled Current Source (Three-Phase), `input_option singleFreqAC`, inputs rms magnitude (A) and phase (rad) from the asset's current phasor: $I_{mag} = |I|\cdot I_b$, $I_b = S_b/(\sqrt3 V_{col})$; the terminal voltage from a Phase Voltage Sensor turned into a phasor by the space-phasor transform (below) |
| Unit transformer | series R-L (RLC (Three-Phase), `SeriesRL`) at the collector voltage, $Z_u = u_k V_{col}^2/(S\, n_{Agg})$ split by X/R |
| Feeder segment (`piBlock`) | subsystem: series R-L ($R L/n_{Par}$, $X L/(\omega n_{Par})$) and half the charging capacitance ($C L n_{Par}/2$) to ground at each end, from RLC (Three-Phase) blocks |
| Wind transformer 10/33 kV and HPP transformer 33/132 kV (`trBlock`) | Two-Winding Transformer (Three-Phase), Yg/Yg, leakage and winding resistance split equally between the windings, magnetizing branch from `Rm`, `Xm` |
| Plant meters and POC meter (`meterBlock`) | Power Sensor (Three-Phase) between the plant node and `HPP_bus` (or POC and the grid impedance), P and Q in W / var |
| Bus and POC voltage (`vMeterBlock`) | Phase Voltage Sensor → phasor → magnitude, pu of the nominal voltage |
| Grid | Controlled Voltage Source (Three-Phase), `singleFreqAC`, magnitude $E\cdot V_{poc}$ (line rms, with the dips), phase 0, behind a series R-L $Z_g = V_{poc}^2/(SCR\cdot P_{max,poc})$ split by X/R |
| Nodes | Connection Labels (`wpp_collector`, `tap_<k>`, `HPP_bus`, `POC`) |

The phasor of a sensor's phase voltages (`phasorBody`):

```matlab
a = complex(-0.5, sqrt(3)/2);
s = 2/3*(v(1) + a*v(2) + a^2*v(3)) * exp(complex(0, pi/2 - w0*t));
V = s * sqrt(3/2) / Vn;
```

i.e. the space phasor rotated back at the fixed nominal frequency $\omega_0 t$ (time from a Digital Clock at `p.sim.Ts`) and by +90° so the grid source (sine reference of the library) is at angle 0, scaled to pu of the nominal line voltage. No PLL is needed, because the mode returns exact 50 Hz samples. As in the nodal model, the terminal voltages go through Memory blocks (one step old), and the frequency signal `f` is the same Step.

All R, L, C and transformer data are run-time block parameters (`*_conf = runtime`), so they change between runs without recompiling in normal mode with Fast Restart.

#### Validation against the nodal model

From `docs/design.md` section 5 and commit `8a6d58d`, default layout, both models with the same magnetizing branch:

| Quantity | Simscape | Nodal |
|---|---|---|
| Nominal P_poc | 146.526 MW | 146.525 MW |
| Nominal Q_poc | −10.900 Mvar | −10.883 Mvar |
| Nominal P_wpp | 132.133 MW | 132.131 MW |
| Dip 0.5: V_poc | 0.564 pu | 0.564 pu |
| Dip 0.5: Q_wpp | 51.33 Mvar | 51.35 Mvar |
| Dip 0.5: P_poc during the dip | 85.55 MW | 85.50 MW |
| Largest P_poc difference over the dip | 0.10 MW (one step at fault clearing), median 0.001 MW | |

+0.2 Hz and the BESS at SOCmin also gave the same results. No ripple (below 1e-10 MW); plant Q loops settle to 0. Before the magnetizing branch was added to the nodal model, P_poc differed by 0.84 MW.

#### Why it is slower

Measured run times (60 s nominal case): about 40 s per further run with Fast Restart at first (`docs/design.md` section 11), 107–127 s per case in normal mode in the later Rapid Accelerator test (section 13; the reason for the difference "is not measured yet", section 10). `scripts/networkCode.m` says "about 5 times slower". The nodal model's network costs one 13 × 13 complex product per step; Simscape solves a three-phase physical network with a state per inductor and capacitor, sensors and Simulink–PS converters, through its own solver (inferred explanation). Measurements from the standalone tests (memory note, `experiments/simscapeGrid/ftGridTest.m`):

- 5 converters: 2.3 s per simulated second (0.61 s with `DoFixedCost` on and `MaxNonlinIter` 1, identical results); 20 converters ran out of memory (16 GB machine).
- Unit transformers as transformer blocks: 1.13 s per simulated second against 0.32 s with series R-L at 33 kV (same P/Q within 0.4 MW); in HPP the network was 3.5 times slower with real transformer blocks (`docs/design.md` section 5).
- The partitioning solver is not allowed in this mode.

#### Rules learned in the tests (memory note "grid-simscape-ft", `docs/design.md`)

- The grid source must be a sinusoidal source in this mode (the test found that Programmable Voltage Source is not; the HPP model uses Controlled Voltage Source with `singleFreqAC`).
- Two-Winding Transformer needs the magnetizing branch when fed by an ideal current source; converter side Yg (delta has no ground).
- Transmission Line (Three-Phase) produced a constant, non-physical 100 Hz power ripple (0.6 MW at 147 MW) even with constant converter currents; the R-L-C π has none (and is faster). Hence `piBlock`.
- Phase convention: current phase = voltage space-phasor angle + π/2 gives unity power factor (handled by the +π/2 in `phasorBody`).
- No three-phase tap changer block exists (Tap-Changing Transformer is single-phase).
- Dips can be made with Fault (Three-Phase) and a temporal fault; HPP instead modulates the source magnitude, like the nodal model.
- Do not set `SimulationCommand` to `stop` on HPP while it is terminating: that left all referenced models stuck "being compiled" until a MATLAB restart.

#### Why it cannot run in Rapid Accelerator

Measured (`docs/design.md` section 13): the target builds (167 s) and the nominal case matches normal mode (1e-12), but other cases (dip 0.5, SCR 3) silently gave the nominal results: no dip, V_poc stayed at 1.0048 (the SCR 10 value) instead of 0.9880. The Simscape network's data and grid source are fixed when the program is built. Rebuilding per case (167 s) would be slower than normal mode. `rapidTarget` therefore refuses the Simscape network, and `runCase` / `runSweep` fall back to normal mode with Fast Restart (chapter A5).

#### Why it is kept

- Independent validation of the hand-coded solver: it found the magnetizing-loss difference, and it confirmed the dip response to within 0.10 MW.
- Readability: a component diagram (PowerFactory-like) is what the user finds easier to follow than the code-based grid (memory note).
- It is the basis for the planned grid rework (branch name `grid-components`): the memory note records the conclusion that option B (standard Simscape blocks in F&T mode) is fine as a small reference model but does not scale, and that the plan recommends option A (an own Simscape phasor domain) or option C (masked blocks over the existing Y solver).

#### Consequences

- Use the Simscape network for checks and demonstrations, not for case-set sweeps.
- Any network change must be made in both `networkSolverData` and `buildNetworkSimscape`, then validated by comparing the nominal and a dip case.
- The Simscape network takes E directly from `p.grid.E`, the nodal from `p.net.solver.E` (copied in `networkSolverData`); both see the same value since `deriveParameters` runs after every case change.

### B2.7 Grid: Thevenin equivalent and events

#### B2.7.1 Thevenin equivalent

`gridParameters`: `Vpoc = 132 kV`, `SCR = 10` (short-circuit power / `Pmax_poc`, so S_sc = 1800 MVA), `XR = 10`, `E = 1.0` pu at angle 0, `f0 = 50` Hz. $|Z_g| = S_b/(SCR\cdot P_{max,poc})$ = 0.1 pu on 180 MVA. The weak-grid cases use SCR 3 (`grid_strength`: SCR 10 / 5 / 3; `wt_frt` and `hppc_qv` have SCR 3 cases).

Why SCR on `Pmax_poc`: the SCR is conventionally defined against the plant's rated power at the POC; with $S_b = P_{max,poc}$, $|Z_g| = 1/SCR$ pu, so the voltage sensitivity is $dV/dQ \approx 1/SCR$, which the voltage controllers use to normalise their gains (chapter B5).

#### B2.7.2 Voltage dips and sustained voltage changes (`p.grid.dip`)

In both network models:
```
E(t) = E0 − depth·step(t0) + depth·step(t0 + dur)
```
(blocks `E0`, `dip start`, `dip end`, `E grid` with signs `+-+`). Parameters: `p.grid.dip.t0` (default 30 s), `dur` (0.15 s), `depth` (0 = no dip). A dip reduces the **source** voltage; the POC voltage then depends on the plant's reactive current and the grid impedance (dip 0.5 → V_poc ≈ 0.56 pu with the plant's support).

- Fault ride-through (`wt_frt`): `t0` 45 s, 150 ms, depth 0.5 and 0.8, and 0.5 at SCR 3.
- A **long duration with small depth is a sustained voltage change**: `hppc_qv` uses depth 0.02, `t0` 40 s, `dur` 100 s (longer than the 90 s run) for a −2 % grid voltage step; depth 0.10 and −0.08 for the Q-limit cases (a negative depth raises the voltage).

Why the source voltage: it is the simplest way to impose a grid-side voltage event in a Thevenin equivalent, works identically in both network models, and is tunable (Step block parameters), so dip cases run on one compiled model. Alternatives: a fault impedance at the POC (needs a Y change) or Simscape's Fault block (only in Simscape); not used.

#### B2.7.3 Frequency steps (`p.grid.fev`) and how frequency reaches the controllers

`p.grid.fev.t0` (default 40 s, case set `freq_step` uses 50 s) and `df` (pu; 0.004 = +0.2 Hz, 0.010 = +0.5 Hz). Two identical Step blocks produce the frequency: `f_meas` in the TSO subsystem (to the HPPC) and `f grid` in the network output (to the plant controllers via the network bus). The HPPC takes `f_meas` directly (no link, no delay, no memory: "measured locally"); each plant controller through a one-step memory only (its last input, `f_meas`). The asset controllers have no frequency input.

The network itself stays at 50 Hz: the frequency is a signal, not a state. There is no inertia, no frequency dependence of the impedances, and the plant's power change does not affect the grid frequency (infinite grid in frequency).

#### B2.7.4 P_ref and Q_ref steps

In the TSO subsystem (`buildTSO`): `P_ref` is a Step from `p.grid.Pref` (150 MW) to `Pref + Pstep.dP` at `Pstep.t0` (40 s); `Q_ref` from `p.grid.Qref` (0) to `Qref + Qstep.dQ` at `Qstep.t0` (40 s). Both in W / var, converted to pu and delayed by `comm.tso` in the HPPC subsystem (chapter B6). Used by `hppc_p` (P steps), `hppc_qv` and `comm_delay` (Q steps).

### B2.8 Known artefacts

#### Single-step spikes at dip inception and clearing

- Stage 3 (`docs/design.md`): during the first ≈ 30 ms of a dip and at clearing the network "rings for a few steps (single-step spikes up to ≈ 180 Mvar)".
- Stage 4: the converter lag `Tg` = 20 ms removed most of the ringing at the dip start; "the single-step spike at fault clearing remains (Q_poc ≈ 125 Mvar for one step)".
- It is also the largest Simscape–nodal difference (0.10 MW, one step at clearing), and it is excluded when reporting overshoots ("max 150.2 MW apart from the known one-step spike at clearing").

Cause (recorded): "the instantaneous network with one-step-delayed current sources". Mechanism (inferred): the network is algebraic, so when E jumps, every node voltage jumps in the same step; the converter currents in that step are still the ones computed with the previous step's voltage — aligned to the old angle and sized for the dip (large reactive dip current $i_q$). For that one step, the high reactive current flows at the restored voltage, giving a large Q (and a phase error in the current gives a P/Q error). In the next step the currents are realigned, and the controllers reduce $i_q$ with their own rate limits.

Remedies (`docs/design.md` open points): a smaller time step or a filter on the network voltage seen by the converters. Neither is implemented.

#### Start-up transient

The controllers start from zero measurements (memories with initial value 0, open point "Steady-state initialisation"), and wind starts at 0.6 pu available power with a step to 0.9 pu at 20 s. Case sets put events after the plant has settled (dips at 45 s, frequency steps at 50 s).

### Recreating it

1. Write `networkParameters` (collector voltages, unit/plant/HPP transformer data with Rm/Xm, cable R/X/C per km, lengths, nPar) and `gridParameters` (Vpoc, SCR, X/R, E, f0, dip, fev, Pstep, Qstep).
2. Write `networkSolverData`: node ordering (terminals, taps, collectors, busbar, POC), `addSeries` for transformers and cable segments, π half-charging at both segment ends, magnetizing shunts, grid admittance at the POC; store real/imag of `inv(Y)`, plant branch data and indices. Call it from `deriveParameters`.
3. Build the nodal network subsystem: injections → MATLAB Function `solve` with `net` non-tunable (structure) and Constant inputs for the numbers → V of terminals through Memory blocks (IC 1 and 0) → network bus; powers to W/var; frequency Step; POC bus.
4. In each asset, make the generator a current source aligned with the incoming (one-step-old) voltage; convert currents to system pu in the plant subsystem.
5. Implement the dip as E0 − depth·step(t0) + depth·step(t0+dur) and the frequency as a Step in both the TSO and the network.
6. Check against a hand load flow (P_poc, Q_poc, V_poc, V_bus).
7. Optionally build the Simscape choice with the component mapping of B2.6 (R-L-C π feeders, R-L unit transformers, Yg/Yg transformers with magnetizing branch, Controlled Current/Voltage Sources with `singleFreqAC`, space-phasor transform at fixed ω0 t, Solver Configuration in F&T mode with local Backward Euler, fixed cost, one iteration), wrap both in a Variant Subsystem on `HPP_network`, and compare nominal and dip results.

---

## B3. Asset models

This chapter describes the three asset types at the bottom of the hierarchy: the wind turbine (IEC 61400-27-1 Type 4A), the PV inverter station (WECC REEC_B + REGC_A) and the battery (WECC REEC_C + REGC_A + an energy/SOC model). For each one it covers the block structure, the equations, every parameter and its value, what was simplified, and why. It starts with what the three have in common: the interface, the per-unit system, the sample times and the coupling to the network.

Sources used: the models themselves (`models/ctrl/WT_ctrl.slx`, `PV_ctrl.slx`, `BESS_ctrl.slx`, `models/assets/WT_asset.slx`, `PV_asset.slx`, `BESS_asset.slx`, read from their XML), the templates in `utilities/build/buildHPP.m` (`buildWTCtrlType4A` ~line 1730, `buildReecCtrl` ~line 1826, `buildAsset` ~line 755, helpers `lagBlk`, `contLag`, `piBlk` ~lines 1658–1692), `params/windTurbineParameters.m`, `pvParameters.m`, `batteryParameters.m`, `controllerParameters.m`, `deriveParameters.m`, `utilities/build/modelInterfaces.m`, `docs/design.md` (sections 2, 4.3, 5, 8 stages 3–4, 10, 11, 13) and the commit messages (mainly `0e70343` "Stages 3-4", `54bf4da` "Stage 2", `61bc052`/`ae5559d` (ode1), `4a15dea`/`ee3b136` (tunable expressions, `contLag`)).

The models and the templates agree block for block and line for line (see "Templates and models" at the end of the chapter). Where `docs/design.md` and the code disagree, the code is described and the difference is noted.

### B3.1 What an "asset" is in this model

#### What

An asset is one modelled generating unit: one converter with its controller, connected at its own network node. Each asset type is one parameterised Simulink model (`<type>_asset.slx`), and `buildHPP` places it N times in `HPP.slx` (N = `nFeeders · nPerFeeder` per plant; default wind 2, PV 2, BESS 1). Each asset model contains:

1. the **asset controller** (`<type>_ctrl.slx`) as a referenced model (Model block named `ctrl`);
2. the **generator / converter model**: current dynamics (rate limits, a first-order lag `Tg`, for PV/BESS also LVPL/LVACM) and a **current source aligned with the terminal voltage** (MATLAB Function `generator`);
3. **terminal measurements**: |V| from the network voltage phasor (MATLAB Function `terminal`), and P and Q of the generator delayed by one step (Memory blocks `P meas`, `Q meas`);
4. for the BESS only, the **battery energy model**: an integrator for SOC.

One asset can stand for `nAgg` identical physical units at the same node (aggregation, see B1). Its rating, and so its per-unit base, is `nAgg · Pn`: wind 15 × 5 MW = 75 MW, PV 5 × 5 MW = 25 MW, BESS 1 × 30 MW = 30 MW (`controllerParameters`, `assetCtrl(p.sim.Ts, p.<type>.nAgg * p.<type>.Pn)`; README table of controllers).

#### Why the controller sits inside the asset model

- **Controller and plant side are separated by model boundaries.** Every controller is a separate referenced model and a future DLL (design.md section 2: "Every controller is a separate referenced model and a future DLL. Everything below the dashed line is the 'plant' in Simulink; in PSCAD it is replaced by PSCAD's own EMT models"). The asset controller is therefore its own model file (`models/ctrl/<type>_ctrl.slx`), but it is *instantiated* inside the asset model.
- **It is local to its converter.** The controller's measurements (terminal voltage, P, Q, SOC) come from its own converter with no communication delay (design.md section 6: "Asset measurements → asset controller: 0 (local)"), and its outputs (current commands) go straight to the converter. Placing it inside the asset model keeps this tight loop inside one block and away from the link blocks with delays in `HPP.slx`.
- **One Model block per asset in `HPP.slx`.** The build script only has to place and wire N asset blocks per plant, each with one instance argument `ap = p.<type>.ap(k)` (design.md section 3). Each unit then looks like the physical thing it represents: a converter with its own controller.
- **It mirrors the PSCAD case** (inferred): in PSCAD the converter model is PSCAD's own EMT model, and the asset controller DLL sits next to it in the same unit. The `ctrl` Model block inside the asset is exactly the piece that becomes the DLL.

#### Alternatives / tried and dropped

- A **For Each subsystem** instead of N Model blocks was considered and not used: "the build script is simpler to debug, and every instance stays a normal Model block, which code generation and PSCAD handle the same way" (design.md section 3).
- Placing the asset controllers next to the plant controllers in `HPP.slx` (outside the asset models) is not recorded as considered. It would need extra local links per asset and would mix controller wiring with plant wiring (inferred).

#### Consequences

- To change a converter model, edit `<type>_asset.slx`. To change the control, edit `<type>_ctrl.slx`. `HPP.slx` does not change unless the ports change (then also `utilities/build/modelInterfaces.m` and the wiring in `buildHPP.m`; see A3).
- The BESS has its own asset controller even though the BESS plant controller (EMS) exists. design.md section 2 gives the reason: the EMS decides *how much* power over minutes to hours (SOC management); the asset controller decides *how* it is delivered (current control, converter limits and the BMS limits for SOC and current). These are different time scales and functions, and keeping them separate makes the three asset types structurally the same. In the WECC set this split is REPC (plant) and REEC_C (asset).

### B3.2 The common interface

#### Ports

The ports are defined once in `utilities/build/modelInterfaces.m`. `HPP.slx` connects by port number, and `scripts/checkModels.m` compares the hand-maintained models against this list.

**Asset controllers** (`WT_ctrl`, `PV_ctrl`, `BESS_ctrl`), model argument `par`:

| # | Input | Unit | Meaning |
|---|---|---|---|
| 1 | `P_set` | pu (asset) | active power set-point from the plant controller |
| 2 | `Q_set` | pu (asset) | reactive power set-point from the plant controller |
| 3 | `V_term` | pu | terminal voltage magnitude |
| 4 | `P_meas` | pu (asset) | generator active power, one step old (not used by any controller, see below) |
| 5 | `Q_meas` | pu (asset) | generator reactive power, one step old |
| 6 | `P_wind_avail` / `P_pv_avail` / `SOC` | pu / pu / – | available power (WT, PV) or state of charge (BESS) |

| # | Output | Unit | Meaning |
|---|---|---|---|
| 1 | `ip_cmd` | pu (asset) | active current command |
| 2 | `iq_cmd` | pu (asset) | reactive current command (+ = capacitive / export) |
| 3 | `P_avail` | pu (asset) | available power reported to the plant controller |

**Asset models** (`WT_asset`, `PV_asset`, `BESS_asset`), model argument `ap`:

| # | Input | Unit | Source in `HPP.slx` |
|---|---|---|---|
| 1 | `P_set` | pu (asset) | plant controller output `P_set_k(k)`, after the plant→asset link (delay `p.comm.plant_to_asset`, gain plant base / asset base) |
| 2 | `Q_set` | pu (asset) | same for `Q_set_k(k)` |
| 3 | `V_re` | pu (system voltage base = nominal voltage) | network bus, element of the asset's node, **one step old** |
| 4 | `V_im` | pu | same |
| 5 | `P_wind_avail` (WT) / `P_pv_avail` (PV) | pu (asset) | Step block `wind k` (WT) or Constant `irradiance k` (PV) in the plant subsystem; BESS has no fifth input |

| # | Output | Unit | Destination in `HPP.slx` |
|---|---|---|---|
| 1 | `P` | pu (asset) | terminated (not used; plant measurements come from the network) |
| 2 | `Q` | pu (asset) | terminated |
| 3 | `P_avail` | pu (asset) | Mux → `P_avail_k` → plant controller (gain asset/plant base, memory, delay `p.comm.meas_plant`) |
| 4 | `I_re` | pu (asset) | Mux → gain asset base / system base (`p.ctrl.<type>.Pbase / p.grid.Pbase`) → network current injection |
| 5 | `I_im` | pu (asset) | same |
| 6 | `SOC` (BESS only) | – | Mux → `SOC_k` → BESSC (memory, delay) and `SOC_plant` (logging) |

The wiring is in `buildPlantSub` (`utilities/build/buildHPP.m`, ~line 963).

The design table in design.md section 4.3 also lists an input `omega_gen` for a Type 4B drive train. It does not exist in the models: only Type 4A is built (design.md section 9: "Wind model: IEC 61400-27-1 Type 4A (no drive-train model) for now").

#### Per unit

- **What:** every asset controller works in per unit on its own unit rating `Pbase = nAgg · Pn` (`p.ctrl.wt.Pbase` = 75 MW, `p.ctrl.pv.Pbase` = 25 MW, `p.ctrl.bess.Pbase` = 30 MW). The asset model uses the same base for currents and powers. Voltages are in pu on the nominal voltage of the node, which is the same in the asset and the network, so voltages need no conversion. Currents do: the asset's current in asset pu is multiplied by `Pbase_asset / 180 MVA` before it enters the network (link `I_re to system pu`), because the network works on the 180 MVA system base.
- **Why:** all controllers follow the same rule (design.md section 4): per unit on their own base, scalar `double` ports, tunable `par`, fixed `Ts`, with the W/var ↔ pu conversions outside the controllers ("in `HPP.slx` in Simulink, and in the PSCAD case in PSCAD"). An asset controller's parameters are then the same for any `nAgg`, and the controller can become a DLL that does not know the plant layout.
- **Consequences:** a current of 1 pu is the rated current of the whole aggregated unit (`nAgg` turbines). Limits like `imax` = 1.1 pu therefore apply to the aggregate. P = |V|·i_p is in pu of the unit rating.

#### Instance arguments `ap`

Simulink concepts used here: a **model argument** is a model-workspace variable that the parent can set per Model block (per instance); the parent's value is an **instance argument**. Each asset model declares one argument, `ap` (`ParameterArgumentNames = ap`), and the controller inside declares `par`. The asset's `ctrl` Model block passes `par = ap.ctrl` (`ParameterArgumentValues = ap.ctrl`), so the whole parameter set of one unit, controller included, travels in one struct.

`deriveParameters` builds the per-instance struct arrays (lines 30–40):

```matlab
p.wt.ap = repmat(struct("ctrl", p.ctrl.wt.par, "gen", p.wt.iec.gen), p.wt.N, 1);
p.pv.ap = repmat(struct("ctrl", p.ctrl.pv.par, "gen", p.pv.wecc.gen), p.pv.N, 1);
p.bess.ap = repmat(struct("ctrl", p.ctrl.bess.par, "gen", p.bess.wecc.gen, ...
    "SOC0", p.bess.SOC0, "kSOC", p.ctrl.bess.Pbase / Eunit), p.bess.N, 1);
```

| Field | WT | PV | BESS | Used by |
|---|---|---|---|---|
| `ap.ctrl` | `p.ctrl.wt.par` | `p.ctrl.pv.par` | `p.ctrl.bess.par` | the asset controller (`par`) |
| `ap.gen` | `p.wt.iec.gen` | `p.pv.wecc.gen` | `p.bess.wecc.gen` | generator / converter blocks |
| `ap.SOC0` | – | – | `p.bess.SOC0` | SOC integrator initial condition |
| `ap.kSOC` | – | – | `Pbase / Eunit` [1/s] | SOC integrator gain |

`HPP.slx` sets instance k to `p.<type>.ap(k)` (`setArg(..., "ap", sprintf("p.%s.ap(%d)", pl.t, k))`). The model workspace of each asset model holds a default for standalone use: `ap = HPP_parameters().<type>.ap(1)` and `Tsim = p.sim.Ts` (the default `src` in `modelInterfaces` is `"wt.ap(1)"` etc.).

- **Why one struct with the controller inside:** all instances are identical today, but per-instance values are "not planned, but kept possible" (design.md section 9): different turbine types in one plant only need different values in `ap(k)`; the model does not change.
- **Tunability rule:** compiled code (accelerator, Rapid Accelerator, the later DLLs) keeps a block parameter that uses `par` or `ap` tunable only if it is simple arithmetic (+ − × ÷ and field access). That is why the WT controller uses a derived flag `par.MqG1 = double(MqG == 1)` from `controllerParameters`, and why the converter lags are built from a gain `1/(ap.gen.Tg)` and an integrator instead of a Transfer Fcn with denominator `[ap.gen.Tg 1]` (design.md section 13; commit `ee3b136`). Code inside MATLAB Function blocks is exempt, so `ap.gen.Lvpl1` etc. can be used freely in the LVPL function.

#### Sample times and solver

- **Asset controllers run at 1 ms**, the simulation step: `controllerParameters` sets `ctrl.wt.Ts = ctrl.pv.Ts = ctrl.bess.Ts = p.sim.Ts` (= 1e-3 s). The controller models are fixed-step discrete (`Solver = FixedStepDiscrete`, `FixedStep = Ts`), and all their inports have `SampleTime = Ts`. Their filters and PIs are discrete blocks.
- **Why 1 ms:** design.md section 6 lists asset controllers at "1 ms (= sim step)". In PSCAD the controller DLL runs at a multiple of the plant time step (`simulationParameters.m`: "This mirrors PSCAD, where the controller DLL runs at a..."); `controllerParameters` asserts that every `Ts` is an integer multiple of `p.sim.Ts`. Converter controls are the fastest level, so they run at the plant step (inferred).
- **Asset models are continuous/hybrid.** Their inports have inherited sample time. The current lags `Tg` and the BESS SOC integrator are continuous Integrator blocks. The rate limiters sit downstream of the 1 ms controller outputs and inherit that rate.
- **Solver: ode1 in `HPP.slx`, not ode4.** The asset models' own configuration says `Solver = ode4`, `FixedStep = Tsim` (set by the template `buildAsset`). But the top model `HPP.slx` uses `ode1` (forward Euler), single-tasking, step `p.sim.Ts` (`buildTop`, ~line 237). design.md section 11: "Solver ode1 (one evaluation per step) instead of ode4, single-tasking. The asset models keep their continuous current lags (`Tg`) and SOC integrator; making them discrete gave no speed gain and moved P during a dip by 2–3 MW, so it was not kept." The asset models do not enable a local solver (no `UseModelRefSolver` in their configuration), so inside `HPP.slx` their continuous states are integrated by the top model's ode1. The ode4 setting only matters when an asset model runs on its own (inferred from Simulink's rules for referenced models; not measured in the repository).
- **Consequences:** with ode1 and h = 1 ms, the lag $1/(T_g s+1)$ becomes $x_{k+1} = x_k + (h/T_g)(u_k - x_k)$. That is accurate for `Tg` = 10–20 ms, but `Tg` must stay well above 1 ms (forward Euler is unstable for h > 2·Tg). Discrete lags inside the controllers have the same constraint (`PoleZ = 1 − Ts/T` needs T > Ts; see below).

#### How the assets connect to the network: current out, voltage back

**What:** each asset is a **current source**. Its output is the current phasor `I_re + j·I_im` in asset pu, aligned with the terminal voltage phasor of the previous step:

$$I = (i_p - j\,i_q)\,\frac{V}{|V|}, \qquad |V| \ge 10^{-3}$$

(MATLAB Function `generator`, identical in all three asset models):

```matlab
v  = max(sqrt(V_re^2 + V_im^2), 1e-3);
cr = V_re / v;  ci = V_im / v;
I_re = ip*cr + iq*ci;
I_im = ip*ci - iq*cr;
P = v * ip;
Q = v * iq;
```

With $S = V I^*$ this gives $S = |V|(i_p + j i_q)$, so $P = |V|\,i_p$ and $Q = |V|\,i_q$. Positive $i_p$ is export (generator convention); positive $i_q$ is capacitive (export of Q), matching the sign convention of design.md section 4 ("+ P = export / discharge, + Q = export (capacitive)").

The network (B2) takes all asset currents, solves $V = Z(I_{inj} + I_{grid})$, and returns the terminal voltages through Memory blocks (`V_re prev`, `V_im prev`, initial condition 1 + j0) in the `Network` subsystem: the asset always sees **the voltage of the previous step**.

**Why:**
- A current source behind the voltage angle is the standard RMS representation of a grid-following converter with fast current control: the converter's PLL tracks the voltage angle, and its current controller sets the d/q currents. Both IEC 61400-27-1 Type 4 and WECC REGC_A model the converter this way (design.md section 5: "Generator interface ... current source aligned with the terminal voltage").
- Using the previous step's voltage acts like an ideal PLL with one step of delay and "also avoids an algebraic loop through the network" (design.md section 5; commit `54bf4da`: "placeholder generator is a current source aligned with the previous step's terminal voltage (PLL-like, avoids an algebraic loop)"). Without it, the current would depend on V, and V on the current, within the same step.
- The voltage floor 1e-3 avoids a division by zero in a bolted fault (inferred).

**Alternatives:** no PLL model is used. In the Simscape network variant the converter is a Controlled Current Source driven by the same phasor, and "No PLL is needed because the mode returns exact 50 Hz samples" (design.md section 5).

**Consequences:**
- No PLL dynamics, no angle jumps, no loss of synchronism. Phase-angle jumps during faults are followed exactly one step later.
- The one-step delay of current sources against an instantaneous network causes the **single-step spike at fault clearing** (Q_poc ≈ 125 Mvar for one step) and, before `Tg` was 20 ms in PV/BESS, ringing at the dip start (design.md stage 3 limit (b), stage 4 checks, section 10). "A smaller step or a filter on the network voltage would remove it."

#### Terminal measurements inside the asset

- `V_term = sqrt(V_re^2 + V_im^2)` (MATLAB Function `terminal`) goes directly to the controller. It is already one step old because the network voltage is.
- `P_meas` and `Q_meas` are the generator outputs P and Q through Memory blocks (one step, 1 ms). The template comment says "terminal measurements: |V|, P and Q one step old".
- **Why the Memory on P/Q** (inferred): it puts a sampling delay between the converter and its controller, the same pattern as the one-step memories on every measurement path into a controller in `HPP.slx` (design.md section 6: without them "the loops through the controllers' direct feedthrough are algebraic loops when a delay is 0"). Inside the asset the continuous `Tg` lag already breaks the loop, but the memory keeps the controller free of direct feedthrough from the generator, so the loop stays well-posed if `Tg` is removed or made discrete.

### B3.3 Building blocks and their equations

The asset controllers use a few standard blocks. Their discrete forms matter when a parameter is changed.

| Block (helper in `buildHPP.m`) | Simulink block | Equation at step k (Ts = 1 ms) |
|---|---|---|
| First-order lag `lagBlk(sys, name, T, y0)` | Discrete *Transfer Fcn First Order*, `PoleZ = 1 − Ts/T`, `ICPrevOutput = y0` | $y_k = p\,y_{k-1} + (1-p)\,u_k$, $p = 1 - T_s/T$, i.e. $y_k = y_{k-1} + \frac{T_s}{T}(u_k - y_{k-1})$ (DC gain 1, direct feedthrough) |
| Continuous lag `contLag(sys, name, T)` | subsystem: Sum → Gain `1/(T)` → Integrator, feedback | $\dot x = (u - x)/T$, $y = x$, $x(0) = 0$; with ode1: $x_{k+1} = x_k + \frac{h}{T}(u_k - x_k)$ |
| PI `piBlk(sys, name, Kp, Ki, lim)` | *Discrete PID Controller*, PI, parallel form, Forward Euler, output limits ±lim, clamping anti-windup, sample time inherited | $u_k = \mathrm{sat}(K_p e_k + x_k)$, $x_{k+1} = x_k + K_i T_s e_k$ unless clamping stops integration (output saturated and $e$ pushing further) |
| Rate limiter | *Rate Limiter* (rising, falling slew in pu/s) | $\Delta y \le R_{rise}\,T_s$, $\Delta y \ge R_{fall}\,T_s$ |
| Dynamic saturation | *Saturation Dynamic* (ports up, u, lo) | $y = \min(\max(u, lo), up)$ |
| Dead zone | *Dead Zone* (`LowerValue`, `UpperValue`) | $y = 0$ for $lo \le u \le up$, $u - up$ above, $u - lo$ below |

Consequences of these forms:
- Every lag time constant must be larger than Ts = 1 ms. T = Ts gives a pass-through (p = 0); T < Ts gives a negative pole (oscillating); T = 0 divides by zero. To switch a filter off, set T just above Ts, not to 0 (inferred).
- The PI's "freeze" in a dip is done by feeding it zero error (a Switch in front of it), not by an enable or reset: the integrator holds, and the proportional part drops to zero for the duration of the dip (see B3.4).
- Rate limiters start from their block default initial condition (not set in the models), and all lags in the P path start at 0. The voltage filters start at 1 (`ICPrevOutput = 1`) so the dip detection does not trip at t = 0. Since the controllers start from zero measurements (design.md section 10: "Steady-state initialisation at start-up" is an open point), every run begins with a short ramp-up of currents.

### B3.4 Wind turbine: IEC 61400-27-1 Type 4A

#### What the standard defines, and what is implemented

IEC 61400-27-1 defines generic RMS models of wind turbines for stability studies. Type 4 is a full-converter turbine. Type 4A ignores the mechanical system (aerodynamic power is taken as constant, no drive train); Type 4B adds a two-mass drive train so that power oscillations in a fault are visible. The standard's Type 4 model consists of measurement, P control, Q control, current limitation, the generator system and (for 4B) the mechanical model; protection and a WPP-level controller are separate modules (background on the standard; the structure used in the repository is described in the template header of `buildWTCtrlType4A`: "Structure after IEC 61400-27-1 (measurement, P control type 4A, Q control, current limitation); block details and parameter values to be checked against the standard").

| Module of the standard | In the model | Where |
|---|---|---|
| Measurement (U, P, Q filters) | yes, first-order discrete lags `Tufilt`, `Tpfilt`, `Tqfilt`; P filter output unused | `WT_ctrl` "Measurement and dip detection" |
| Voltage dip detection | yes, `U < udip`, no hysteresis, no post-fault hold | `WT_ctrl` block `dip` |
| P control type 4A | yes: P_ref limited to available power, ramp `dpmaxp4A`, lag `Tpordp4A`, i_p = p/U | `WT_ctrl` "P control type 4A" |
| Q control | modes MqG = 1 (closed-loop reactive power) and MqG = 2 (open loop) only; dip current `Kqv·(uref0 − U)` | `WT_ctrl` "Q control ..." |
| Current limitation | simplified: |i| ≤ `imax`, P priority normally, Q priority in a dip; no voltage-dependent limits | MATLAB Function `current limitation` |
| Generator system Type 4 | yes: rate limits `dipmax` (rise only) / `diqmax`, lag `Tg`, current source | `WT_asset` |
| Mechanical model (4B) | no | – |
| Protection, WPP control | no (the WPPC is WECC REPC_A, see B4) | – |

**Why Type 4A:** decided after review on 2026-11-16 (design.md section 9). The study is about plant-level and HPP-level control on time scales of 0.1 s and longer; a drive train adds oscillation modes that matter for fault studies of the turbine itself, not for the coordination of plant controllers (inferred). The 4B option stays visible in the design (design.md section 2: "opt. 4B mechanics", input `omega_gen`).

**What is left to check** (design.md stage 3 known limit (a) and section 10): "filter forms, dip logic with hysteresis/post-fault hold, `Kqv` reference, current-limit options, the `uqdip`/`Mqfrt` modes". All parameter values are "typical / placeholder values, NOT taken from the standard or a turbine" (`windTurbineParameters.m`).

#### Measurement and dip detection

Inputs: `V_term`, `P_meas`, `Q_meas`.

$$U_k = U_{k-1} + \frac{T_s}{T_{ufilt}}(V_{term,k} - U_{k-1}), \quad U_0 = 1$$
$$Q_{filt,k} = Q_{filt,k-1} + \frac{T_s}{T_{qfilt}}(Q_{meas,k} - Q_{filt,k-1}), \quad Q_{filt,0} = 0$$
$$P_{filt}: \text{same form with } T_{pfilt}, \text{ output to a Terminator}$$
$$U_{floor} = \max(U, V_{min}), \qquad \mathrm{dip} = (U < u_{dip})$$

- `U_floor` is used to divide (p/U, q/U); `U` itself is used for the dip flag and the dip current.
- The P filter exists but its output is terminated (`P filt unused`): "no P loop in Type 4A" (template comment). It is kept so the structure follows the standard's measurement module and a P loop could be added (inferred).
- **Dip logic:** a plain comparison. No hysteresis, no hold time after the voltage returns. When U crosses `udip` the controller switches mode in the same step. This is listed as a point to check against the standard.

#### P control type 4A

$$P_{lim} = \min(\max(P_{set}, 0), P_{wind,avail})$$
$$P_{ramp}: \text{rate limit } \pm dp_{maxp4A} \text{ [pu/s]}$$
$$p_{ord,k} = p_{ord,k-1} + \frac{T_s}{T_{pordp4A}}(P_{ramp,k} - p_{ord,k-1})$$
$$i_{p,req} = p_{ord} / U_{floor}$$

- **What:** open-loop power control. The set-point is limited to the available wind power (Saturation Dynamic with upper limit `P_wind_avail`, lower limit 0), rate-limited in both directions, delayed by the power-order lag, and converted to an active current with the filtered voltage.
- **Why no P loop:** "Active power stays open loop in both standards (the converter current control, `Tg`, tracks it); there is no P loop at asset level" (design.md stage 4). Losses between the converter and the plant measurement point are corrected by the plant controller and the HPPC, not by the turbine.
- **Consequences:** the ramp `dpmaxp4A` = 1 pu/s applies to every set-point change, including curtailment by the WPPC/HPPC and the wind availability step (0.6 → 0.9 pu takes 0.3 s at the turbine). The constant-power division by U means a voltage drop raises i_p until the current limit takes over.

#### Q control: open loop plus PI (MqG), dip current

$$i_{q,ff} = Q_{set} / U_{floor}$$
$$e_q = \begin{cases} 0 & \text{dip} \\ MqG1\cdot(Q_{set} - Q_{filt}) & \text{otherwise} \end{cases}$$
$$i_{q,PI} = \mathrm{PI}(e_q;\ k_{pq},\ k_{iq}),\ \text{output limited to } \pm i_{max}\ \text{(clamping)}$$
$$i_{q,ol} = i_{q,ff} + i_{q,PI}$$
$$i_{q,req} = \mathrm{sat}_{\pm iqH1}\begin{cases} i_{q,ol} + K_{qv}(u_{ref0} - U) & \text{dip} \\ i_{q,ol} & \text{otherwise} \end{cases}$$

Block chain in `WT_ctrl`: `q_ref to iq` → `iq ff + PI` (→ tag `iq_ol`); `q error` → switch `q loop on?` (control `MqG = 1` constant = `par.MqG1`, else `no q loop` = 0) → switch `freeze in dip` (0 during a dip) → PI `kpq kiq` (→ tag `iq_PI`); `uref0 − U` → gain `Kqv` → `iq + dip` → switch `in dip?` → saturation `iqH1`.

- **What, MqG:** `MqG` = 1 closes a reactive-power loop around the open-loop current command: the PI acts on `Q_set − Q_filt` and its output is added to `Q_set / U`. `MqG` = 2 (any value other than 1 in practice, since the switch uses `MqG1 = double(MqG == 1)`) gives open-loop Q: the PI input is 0 and its integrator stays at its initial value 0, so `i_q = Q_set / U`. The standard's other MqG modes (voltage control, power factor control) are not implemented.
- **Why closed loop at the asset (decided 2026-11-16):** "As in state-of-the-art HPPs, where vendors supply the assets together with their own plant controller, every level closes its own loops; the HPPC (stage 5) then has to coordinate closed-loop, vendor-style plant controllers. This interaction is deliberate (it is the problem the PhD project addresses) and should stay visible in the model" (design.md stage 4). The feed-forward `Q_set / U` gives the fast response; the PI only removes the small error (e.g. from the voltage changing after the division).
- **PI gains:** `kpq` = 0.1 pu current per pu Q error, `kiq` = 10 /s. The integral gain was raised from 5 to 10 /s in stage 5 step 4 for the **cascade timing**: every level at least `cascadeSep` = 3 times slower than the one below, speed estimated as $(1 + K_p)/K_i$; asset Q PIs 0.11 s, plant Q 1.1 s, plant voltage 0.6 s, HPPC 3.3–5.5 s (design.md stage 5 step 4; `controllerParameters` computes `tAsset` from these gains and warns if the separation is violated).
- **Freeze in a dip:** the PI gets zero error while `dip` is true, so the integrator holds its pre-fault value. Its proportional part is zero during the dip, so `iq_PI` jumps by $-k_{pq} e_q$ at dip entry and back at exit; with `kpq` = 0.1 this is small (inferred from the block structure). Without the freeze, the integrator would wind up against the large Q error caused by the dip current.
- **Dip current:** during a dip an extra reactive current $K_{qv}(u_{ref0} - U)$ is added to the pre-dip current command (the feed-forward with the new voltage plus the frozen PI). With `Kqv` = 2 and U = 0.5 that is +1.0 pu. There is no deadband in the WT path (unlike REEC_B/C). The reference `uref0` is a fixed parameter (1.0), not the pre-fault voltage; whether the standard uses the pre-fault voltage is one of the points to check ("`Kqv` reference").
- **iqH1:** in the model, the saturation `iqH1` (±1.05 pu) is applied to `iq_req` **always**, not only during a dip. design.md stage 3 says "plus dip current `Kqv·(uref0 − U)` during a dip, limited to `iqH1`"; the code limits the reactive current request in normal operation too. In practice the current limitation (`imax` = 1.1 with P priority) is the tighter limit in normal operation, so this rarely matters.
- **Q_filt vs Q_meas:** the WT loop uses the filtered Q (`Tqfilt`), while the PV/BESS loops use the unfiltered `Q_meas` (as in REEC, where the loop acts on Q_gen).

#### Current limitation

MATLAB Function `current limitation` (`current_limitation(ip_req, iq_req, in_dip, par)`):

```matlab
imax = par.imax;
if in_dip                                   % reactive current first
    iq_cmd = min(max(iq_req, -imax), imax);
    ipmax  = sqrt(max(imax^2 - iq_cmd^2, 0));
    ip_cmd = min(max(ip_req, 0), ipmax);
else                                        % active current first
    ip_cmd = min(max(ip_req, 0), imax);
    iqmax  = sqrt(max(imax^2 - ip_cmd^2, 0));
    iq_cmd = min(max(iq_req, -iqmax), iqmax);
end
```

$$\text{normal: } i_p = \mathrm{clip}(i_{p,req}, 0, i_{max}),\ i_q = \mathrm{clip}(i_{q,req}, \pm\sqrt{i_{max}^2 - i_p^2})$$
$$\text{dip: } i_q = \mathrm{clip}(i_{q,req}, \pm i_{max}),\ i_p = \mathrm{clip}(i_{p,req}, 0, \sqrt{i_{max}^2 - i_q^2})$$

- **What:** the total current magnitude is held at or below `imax` (1.1 pu). In normal operation active current has priority, in a dip reactive current has priority (as grid codes require reactive current support in faults). Active current cannot be negative: the turbine cannot absorb active power.
- **Simplified relative to the standard:** the standard's current limitation has more options (voltage-dependent limits on i_p and i_q, a priority flag, limits on the change of currents after a fault; "current-limit options" in design.md's list of checks). Here it is one static rule.
- **Consequence in a dip:** with U ≈ 0.5, `iq_req` hits `iqH1` = 1.05, leaving $\sqrt{1.1^2 - 1.05^2} \approx 0.33$ pu for i_p, so the turbine's P drops to about 0.16 pu during the dip.

#### Outputs

`ip_cmd`, `iq_cmd`, and `P_avail = P_wind_avail` (passed straight through). In Type 4A the aerodynamic power is an input; there is no rotor model that could limit it further.

#### Generator system (in `WT_asset`)

$$i_p^{r}: \text{rate limit, rising } dip_{max},\ \text{falling } -10^6\ \text{(practically unlimited)}$$
$$i_q^{r}: \text{rate limit } \pm diq_{max}$$
$$\dot i_p = (i_p^r - i_p)/T_g, \qquad \dot i_q = (i_q^r - i_q)/T_g$$
$$I = (i_p - j i_q)\,V/|V|$$

- **What:** the IEC Type 4 generator system: rate limits on the current commands, then a first-order lag `Tg` on each current (continuous, `contLag`), then the current source.
- **Why only the rise of i_p is limited:** active current may drop instantly (e.g. at fault inception) but must recover at a limited rate after a fault ("active current: only its rise is limited", template comment). `dipmax` = 1 pu/s sets the post-fault active power recovery: "after clearing, P ramps back at `dipmax` = 1 pu/s and is back at 146.4 MW within about 0.6 s" (design.md stage 3 checks).
- **Why diqmax = 100 pu/s:** reactive current must rise within tens of milliseconds in a fault; 100 pu/s reaches 1 pu in 10 ms, about the same as `Tg` (inferred).

#### Wind availability profile

**What:** each wind asset gets its available power from a Step block in the plant subsystem (`wind k`, `buildPlantSub`): `Before = p.wt.Pavail0` = 0.6 pu, `After = p.wt.PavailStep` = 0.9 pu, `Time = p.wt.tStep` = 20 s, continuous sample time. It is in pu of the asset rating, the same for all instances: 0.6 × 150 MW = 90 MW before and 0.9 × 150 MW = 135 MW after the step for the plant (design.md stage 4 checks: "wind 135 → 124.2 MW" at +0.2 Hz).

**Why:** a placeholder profile ("Placeholder available-power profile (same for all instances)", `windTurbineParameters.m`). The step at 20 s exercises the plant and HPPC dispatch: before it the plant cannot reach P_ref = 150 MW even with the BESS at its discharge limit (P_poc 137.8 MW), after it the BESS absorbs the surplus (design.md stage 5 step 1). The case sets put their events after it (dips at 45 s, frequency steps at 50 s) so they hit a steady operating point (design.md section 12; `wt_frt.m` header).

**Consequences / known effect:** at the step, P_poc dips by about 13 MW for 0.3 s, because the HPPC moves the BESS at once on the jump in availability while the wind output follows only within about 0.5 s (design.md section 10, "Dispatch on availability"). Part of that 0.5 s is the turbine's own ramp `dpmaxp4A` = 1 pu/s (0.3 pu takes 0.3 s) plus the lags.

#### WT parameters

All in pu on the asset rating (75 MW), times in s. Source `params/windTurbineParameters.m`; `ctrl` fields are copied into `p.ctrl.wt.par` by `controllerParameters`, `gen` fields into `p.wt.ap(k).gen` by `deriveParameters`.

| Parameter | Value | Meaning | Where used |
|---|---|---|---|
| `p.wt.Pn` | 5e6 W | rated power per turbine | Pbase = nAgg·Pn |
| `p.wt.nAgg` | 15 | turbines per modelled unit | Pbase 75 MW |
| `p.wt.nFeeders` × `nPerFeeder` | 1 × 2 | layout, N = 2 | `HPP.slx` |
| `p.wt.Pavail0` | 0.6 | available power before the step | Step block |
| `p.wt.PavailStep` | 0.9 | available power after the step | Step block |
| `p.wt.tStep` | 20 | time of the step | Step block |
| `ctrl.Tufilt` | 0.01 | voltage measurement filter | `U filt` |
| `ctrl.Tpfilt` | 0.01 | active power filter (output unused) | `P filt` |
| `ctrl.Tqfilt` | 0.01 | reactive power filter | `Q filt` |
| `ctrl.udip` | 0.90 | dip (FRT) threshold on filtered U | `dip` |
| `ctrl.dpmaxp4A` | 1.0 pu/s | power order ramp, both directions | `dpmaxp4A` |
| `ctrl.Tpordp4A` | 0.01 | power order lag | `Tpordp4A` |
| `ctrl.uref0` | 1.0 | reference voltage of the dip current | `uref0 − U` |
| `ctrl.Kqv` | 2.0 pu/pu | dip reactive current gain | `Kqv` |
| `ctrl.iqH1` | 1.05 | reactive current request limit (±) | `iqH1` |
| `ctrl.imax` | 1.1 | total current limit; also Q PI output limit | `current limitation`, PI |
| `ctrl.MqG` | 1 | Q control mode: 1 closed loop, 2 open loop | → `MqG1` |
| `ctrl.MqG1` | 1 (derived) | `double(MqG == 1)`, block-parameter flag | switch `q loop on?` |
| `ctrl.kpq` | 0.1 | Q PI proportional gain [pu i / pu q] | `kpq kiq` |
| `ctrl.kiq` | 10 /s | Q PI integral gain | `kpq kiq` |
| `ctrl.Vmin` | 0.01 | voltage floor for the current commands (set for all asset controllers in `assetCtrl`) | `U floor` |
| `gen.Tg` | 0.01 | current generation time constant | `Tg ip`, `Tg iq` |
| `gen.dipmax` | 1.0 pu/s | active current rise limit | `dipmax` |
| `gen.diqmax` | 100 pu/s | reactive current rate limit (±) | `diqmax` |
| `p.ctrl.wt.Ts` | 1e-3 | controller sample time (= `p.sim.Ts`) | model workspace `Ts` |

### B3.5 PV: WECC REEC_B + REGC_A

#### What is modelled

The WECC second-generation generic renewable models split a converter-connected plant into REPC (plant controller), REEC (electrical control of the converter) and REGC (the converter/generator interface). For PV, `PV_ctrl.slx` is REEC_B and the converter part of `PV_asset.slx` is REGC_A. Template header (`buildReecCtrl`): "Qflag 1: closed-loop Q (PI), 0: open loop; no voltage-dependent current limit (VDL), no state freeze during a dip except the Q PI."

**Why WECC for PV and BESS:** decided at the start (design.md section 1): "PV and BESS: WECC second-generation generic models (REGC / REEC / REPC)". These are the de facto generic models for inverter-based resources, and their parameter names make the values checkable against WECC tables ("Each standard model's parameter list is kept with the standard's own names", design.md section 7).

#### Voltage filter and dip detection

$$V_k = V_{k-1} + \frac{T_s}{T_{rv}}(V_{term,k} - V_{k-1}),\quad V_0 = 1$$
$$V_{floor} = \max(V, V_{min}), \qquad \mathrm{dip} = (V < V_{dip}) \lor (V > V_{up})$$

The WECC "Voltage_dip" flag is set for both low and high voltage. No hysteresis or hold.

#### P path

$$P_{lim} = \min(\max(P_{set}, 0), P_{pv,avail})$$
$$P_{ramp}: \text{rate limit, rising } dP_{max},\ \text{falling } dP_{min}$$
$$P_{ord,k} = P_{ord,k-1} + \frac{T_s}{T_{pord}}(P_{ramp,k} - P_{ord,k-1})$$
$$I_{p,req} = P_{ord}/V_{floor}$$

Limits 0 … available PV power. With `dPmax`/`dPmin` = ±99 pu/s the ramp is effectively off. `P_meas` is terminated ("no P loop in REEC_B/C"). REEC_B's `Pflag` (P·ω) is not modelled.

#### Q path: Qflag, PI, Tiq, reactive current injection

$$e_Q = \begin{cases} 0 & \text{dip} \\ Q_{flag}(Q_{set} - Q_{meas}) & \text{otherwise}\end{cases}$$
$$I_{q,PI} = \mathrm{PI}(e_Q;\ K_{qp},\ K_{qi}),\ \pm I_{max}\ \text{(clamping)}$$
$$I_{q,lag,k} = I_{q,lag,k-1} + \frac{T_s}{T_{iq}}\left(\frac{Q_{set}}{V_{floor}} + I_{q,PI} - I_{q,lag,k-1}\right)$$
$$I_{qinj} = \begin{cases}\mathrm{sat}_{[I_{ql1},\,I_{qh1}]}\left(K_{qv}\cdot \mathrm{dz}_{[dbd1,\,dbd2]}(V_{ref0} - V)\right) & \text{dip}\\ 0 & \text{otherwise}\end{cases}$$
$$I_{q,req} = I_{q,lag} + I_{qinj}$$

- **Qflag:** `Qflag` = 1 (default) closes the Q loop on the unfiltered generator Q, with the open-loop current `Q_set / V` fed forward. `Qflag` = 0 gives the open-loop path only. In REEC_B the closed-loop Q path is a PI whose output is a voltage reference for a second (voltage) PI, selected by `Vflag`; here a single PI produces a reactive current directly, added to the feed-forward. This keeps the asset Q loop the same in all three asset types (inferred; the design describes it as "PI `Kqp`, `Kqi` on Q_set − Q_gen", stage 4).
- **History:** stage 4 first had `Qflag` = 0 only (open loop, `Iq = Q_set / V`); the closed loop was added afterwards and became the default (design.md stage 4: "closed-loop Q added afterwards (see below: Qflag = 1 is now the default)").
- **PI freeze:** as in the WT, the PI gets zero error in a dip. In REEC the states are frozen during Voltage_dip; here only the Q PI is ("no state freeze during a dip except the Q PI").
- **Reactive current injection:** the voltage error has a deadband of ±0.05 pu (`dbd1` = −0.05, `dbd2` = +0.05), then gain `Kqv` = 2, limited to [−1.05, 1.05]. For V = 0.5: Iqinj = 2·(0.5 − 0.05) = 0.9 pu. For high voltage (V > `Vup` = 1.1) the injection is negative (absorption). The injection is added **after** the `Tiq` lag, so it acts at once.
- **No post-dip hold:** WECC's `Thld`/`Thld2` (keep the injection or hold the P path for a time after the dip) are not modelled.

#### Current limit logic

MATLAB Function `current limit logic` (`current_limit_logic(ip_req, iq_req, par)`):

```matlab
ipmin_soc = 0;  ipmax_soc = par.Imax;   % PV: no absorption
Imax = par.Imax;
if par.PQflag == 0   % Q priority
    iq_cmd = min(max(iq_req, -Imax), Imax);
    ipmax  = sqrt(max(Imax^2 - iq_cmd^2, 0));
    ip_cmd = min(max(ip_req, max(-ipmax, ipmin_soc)), min(ipmax, ipmax_soc));
else                 % P priority
    ip_cmd = min(max(ip_req, max(-Imax, ipmin_soc)), min(Imax, ipmax_soc));
    iqmax  = sqrt(max(Imax^2 - ip_cmd^2, 0));
    iq_cmd = min(max(iq_req, -iqmax), iqmax);
end
```

- `PQflag` = 0 (default): Q priority all the time, not only in a dip (this is how WECC's PQflag works: a static priority). `PQflag` = 1: P priority.
- PV cannot absorb: i_p ≥ 0. (The variable names `ipmin_soc`, `ipmax_soc` come from the shared template with the BESS.)
- **Simplified:** WECC REEC_B derives Iqmax/Ipmax from voltage-dependent tables VDL1 (Iq vs V) and VDL2 (Ip vs V). Here the only limit is the circle |I| ≤ `Imax` ("without VDL tables").
- **Consequence of Q priority in normal operation:** Q set-points from the plant controller take current away from P when the converter is near its limit. At the default PV operating point (0.4 pu) this does not bind.

#### Outputs

`ip_cmd`, `iq_cmd`, `P_avail = P_pv_avail` (straight through). The PV availability is a Constant block `irradiance k` with `p.pv.Pavail` = 0.4 pu (10 MW per unit, 20 MW for the plant), constant over the run ("constant placeholder").

#### REGC_A converter (in `PV_asset`)

Chain for the active current: `Rrpwr` (rate limiter, rising `Rrpwr`, falling −1e6) → `Tg ip` (continuous lag) → `LVPL LVACM` (MATLAB Function) → generator. For the reactive current: `Iqrmax` (rate limiter, rising `Iqrmax`, falling `Iqrmin`) → `Tg iq` → generator.

$$\dot i_p = (\mathrm{rate}_{\le R_{rpwr}}(i_{p,cmd}) - i_p)/T_g, \qquad \dot i_q = (\mathrm{rate}_{[I_{qrmin}, I_{qrmax}]}(i_{q,cmd}) - i_q)/T_g$$

LVPL and LVACM (`lvpl_lvacm(ip, V_re, V_im, ap)`), on the unfiltered terminal voltage $v = |V|$:

$$i_{pl}(v) = \begin{cases} 0 & v \le Z_{erox} \\ L_{vpl1}\dfrac{v - Z_{erox}}{B_{rkpt} - Z_{erox}} & Z_{erox} < v < B_{rkpt} \\ 10^6\ (\text{no limit}) & v \ge B_{rkpt}\ \text{or}\ L_{vplsw} = 0\end{cases}$$
$$k(v) = \mathrm{clip}\left(\frac{v - L_{vpnt0}}{L_{vpnt1} - L_{vpnt0}}, 0, 1\right)$$
$$i_{p,out} = k(v)\cdot \mathrm{clip}(i_p, \pm i_{pl}(v))$$

- **LVPL** (low voltage power logic) limits the active current during low voltage: 0 at or below 0.4 pu, rising linearly to 1.22 pu at 0.9 pu, no limit above. **LVACM** (low voltage active current management) scales the active current to zero between 0.8 and 0.4 pu: it represents the converter's loss of ability to inject active current at very low voltage.
- **Effect seen:** in a 0.5 dip "PV P drops to ≈ 13 MW (LVACM)"; in a 0.8 dip "PV P ≈ 0.8 MW" (design.md stage 4 checks).
- **Simplifications vs REGC_A** (design.md stage 4 "Simplifications"): no high-voltage reactive current management; no voltage filter `Tfltr` (LVPL works on the unfiltered voltage, "LVPL static on the unfiltered voltage"); "REGC_A rate limits applied before the lag instead of on its state". In addition, the code applies LVPL to the lag *output*, together with LVACM, and the `Rrpwr` ramp acts on every rise of the active current command, not only during recovery from a fault (observed in the block structure; whether the WECC specification limits `Rrpwr` to the recovery is one of the points to check). With `Rrpwr` = 10 pu/s and `Iqrmax`/`Iqrmin` = ±99 pu/s these limits rarely bind outside a fault.
- **Why `Tg` = 20 ms:** typical WECC value. It also helped numerically: "The converter lag Tg = 20 ms also removed most of the ringing at the dip start seen in stage 3; the single-step spike at fault clearing remains" (design.md stage 4).

#### PV parameters

All in pu on the asset rating (25 MW), times in s. Source `params/pvParameters.m`; "typical / placeholder values from common WECC example data sets, NOT from a specific inverter".

| Parameter | Value | Meaning | Block |
|---|---|---|---|
| `p.pv.Pn` | 5e6 W | rated power per inverter station | Pbase |
| `p.pv.nAgg` | 5 | stations per modelled unit | Pbase 25 MW |
| `p.pv.nFeeders` × `nPerFeeder` | 2 × 1 | layout, N = 2 | `HPP.slx` |
| `p.pv.Pavail` | 0.4 | available power (constant) | Constant `irradiance k` |
| `ctrl.Trv` | 0.02 | terminal voltage filter | `Vt filt` |
| `ctrl.Vdip` | 0.90 | low-voltage dip threshold | `V < Vdip` |
| `ctrl.Vup` | 1.10 | high-voltage dip threshold | `V > Vup` |
| `ctrl.Vref0` | 1.0 | reference of the reactive current injection | `Vref0 − V` |
| `ctrl.dbd1` | −0.05 | deadband lower edge (on Vref0 − V) | `dbd1 dbd2` |
| `ctrl.dbd2` | 0.05 | deadband upper edge | `dbd1 dbd2` |
| `ctrl.Kqv` | 2.0 pu/pu | reactive current injection gain | `Kqv` |
| `ctrl.Iqh1` | 1.05 | max injected reactive current | `Iqh1 Iql1` |
| `ctrl.Iql1` | −1.05 | min injected reactive current | `Iqh1 Iql1` |
| `ctrl.Qflag` | 1 | 1 closed-loop Q, 0 open loop | `Q loop on?` |
| `ctrl.Kqp` | 0.1 | Q PI proportional gain | `Kqp Kqi` |
| `ctrl.Kqi` | 10 /s | Q PI integral gain | `Kqp Kqi` |
| `ctrl.Tiq` | 0.02 | reactive current lag | `Tiq` |
| `ctrl.dPmax` | 99 pu/s | P ramp up | `dPmax dPmin` |
| `ctrl.dPmin` | −99 pu/s | P ramp down | `dPmax dPmin` |
| `ctrl.Tpord` | 0.02 | power order lag | `Tpord` |
| `ctrl.Imax` | 1.1 | converter current limit; also Q PI output limit | current limit logic, PI |
| `ctrl.PQflag` | 0 | 0 Q priority, 1 P priority | current limit logic |
| `ctrl.Vmin` | 0.01 | voltage floor for current commands | `V floor` |
| `gen.Tg` | 0.02 | converter current lag | `Tg ip`, `Tg iq` |
| `gen.Rrpwr` | 10 pu/s | active current rise limit | `Rrpwr` |
| `gen.Iqrmax` | 99 pu/s | reactive current ramp up | `Iqrmax` |
| `gen.Iqrmin` | −99 pu/s | reactive current ramp down | `Iqrmax` |
| `gen.Lvplsw` | 1 | LVPL on/off | `LVPL LVACM` |
| `gen.Zerox` | 0.4 | LVPL: Ip limit 0 at and below | `LVPL LVACM` |
| `gen.Brkpt` | 0.9 | LVPL: Ip limit reaches Lvpl1 | `LVPL LVACM` |
| `gen.Lvpl1` | 1.22 | LVPL: Ip limit at Brkpt | `LVPL LVACM` |
| `gen.Lvpnt0` | 0.4 | LVACM: Ip = 0 below | `LVPL LVACM` |
| `gen.Lvpnt1` | 0.8 | LVACM: full Ip above | `LVPL LVACM` |

### B3.6 BESS: WECC REEC_C + REGC_A + battery energy

#### What is different from PV

`BESS_ctrl.slx` is built by the same template (`buildReecCtrl("BESS", ...)`) and has the same voltage filter, dip detection, Q path, reactive current injection and REGC_A converter (`BESS_asset.slx`) with the same parameter values (`params/batteryParameters.m`). REEC_C is REEC_B plus state-of-charge limits. The differences:

| Item | PV (REEC_B) | BESS (REEC_C) |
|---|---|---|
| Input 6 | `P_pv_avail` | `SOC` (from the battery model inside the asset) |
| P limits | 0 … `P_pv_avail` | −`Pmax` … +`Pmax` (constants `Pmin = -par.Pmax`, `Pmax = par.Pmax`) |
| i_p range in current logic | ≥ 0 (no absorption) | ±`Imax`, and SOC limits (below) |
| `P_avail` output | `P_pv_avail` | constant `par.Pmax` (converter limit) |
| Asset model input 5 | `P_pv_avail` | none |
| Asset model output 6 | – | `SOC` |
| Battery energy | – | SOC integrator |

`par.Pmax = (p.bess.Pmax / p.bess.N) / p.ctrl.bess.Pbase` = (30 MW / 1) / 30 MW = 1.0 pu (`controllerParameters`). `par.SOCmin` and `par.SOCmax` are copied from `p.bess.SOCmin` = 0.1 and `p.bess.SOCmax` = 0.9.

#### P path

$$P_{lim} = \mathrm{clip}(P_{set}, -P_{max}, P_{max})$$

then the same ramp (`dPmax`/`dPmin`), lag `Tpord` and division by `V_floor` as PV. Negative P = charging.

#### SOC limits in the current limit logic

```matlab
ipmin_soc = -par.Imax;  ipmax_soc = par.Imax;
if SOC <= par.SOCmin, ipmax_soc = 0; end      % no discharge
if SOC >= par.SOCmax, ipmin_soc = 0; end      % no charge
```

then the same priority logic as PV (with `PQflag` = 0, Q priority):

$$i_p = \mathrm{clip}\left(i_{p,req},\ \max(-i_{p,max},\ i_{p,min}^{SOC}),\ \min(i_{p,max},\ i_{p,max}^{SOC})\right),\quad i_{p,max} = \sqrt{I_{max}^2 - i_q^2}$$

- **What blocks charge/discharge at the limits at asset level:** this function, and only this. At SOC ≤ SOCmin the active current is limited to ≤ 0 (charging still allowed); at SOC ≥ SOCmax to ≥ 0 (discharging allowed). It is a hard switch on the current SOC, with no ramp and no hysteresis.
- **Higher levels also act on SOC** (described in B4 and B5): the BESSC uses SOC-aware charge/discharge limits and dispatches only to assets inside their SOC range, and the HPPC derates the BESS limits linearly over `SOCband` = 0.1 above SOCmin and below SOCmax (design.md stage 5 step 3). The asset limit is the last line of defence (the "BMS limit", design.md section 2).
- **Why at the asset:** "The asset controller decides how it is delivered: current and power control, converter limits and the BMS limits for SOC and current" (design.md section 2). In REEC_C the SOC limits act on the active current limits in the same way.
- **Consequences:** because the switch is on the current SOC and the converter lag follows, SOC can pass the limit by a tiny amount before the current reaches zero, and at the limit the current may toggle between zero and the request from step to step if the request keeps pushing (inferred; not reported as a problem). The checks showed the intended behaviour: "At SOCmin the BESS gives 0 MW against a 30 MW discharge set-point and still charges afterwards; at SOCmax it discharges, and charging stops when SOC reaches 0.9" (design.md stage 4).

#### Battery energy model (in `BESS_asset`)

$$\frac{d\,SOC}{dt} = -k_{SOC}\,P, \qquad SOC(0) = SOC_0, \qquad k_{SOC} = \frac{P_{base}}{E_{unit}}$$

Blocks: generator output `P` → Gain `-ap.kSOC` → Integrator `SOC int` (initial condition `ap.SOC0`, no limits) → `SOC` (to the controller and the asset output).

- `E_unit = p.bess.E / p.bess.N` = 60 MWh = 2.16e11 J; `kSOC` = 30e6 / 2.16e11 = 1.39e-4 1/s. At P = 1 pu (30 MW) the SOC changes by 1 in 7200 s (2 h, the 60 MWh / 30 MW ratio).
- **Sign convention:** P > 0 is discharge (export), so SOC falls; P < 0 is charging. This matches `batteryParameters.m` ("positive power = discharging (export to grid)") and design.md section 4.
- **P is the converter terminal power** ($|V|\,i_p$ from the generator), so the model has no efficiency, no converter or battery losses, no self-discharge, no SOC-dependent voltage or current limits (inferred from the block structure).
- **Continuous integrator:** kept continuous like the `Tg` lags; making the asset dynamics discrete gave no speed gain (design.md section 11).

#### BESS parameters

Same REEC/REGC_A values as PV (`ctrl.Trv` 0.02, `Vdip` 0.9, `Vup` 1.1, `Vref0` 1.0, `dbd1`/`dbd2` ±0.05, `Kqv` 2.0, `Iqh1`/`Iql1` ±1.05, `Qflag` 1, `Kqp` 0.1, `Kqi` 10, `Tiq` 0.02, `dPmax`/`dPmin` ±99, `Tpord` 0.02, `Imax` 1.1, `PQflag` 0, `Vmin` 0.01; `gen.Tg` 0.02, `Rrpwr` 10, `Iqrmax`/`Iqrmin` ±99, `Lvplsw` 1, `Zerox` 0.4, `Brkpt` 0.9, `Lvpl1` 1.22, `Lvpnt0` 0.4, `Lvpnt1` 0.8). Specific to the BESS:

| Parameter | Value | Meaning | Where used |
|---|---|---|---|
| `p.bess.Pn` | 30e6 W | converter rating of one unit | Pbase 30 MW |
| `p.bess.nAgg`, `nFeeders`, `nPerFeeder` | 1, 1, 1 | one unit | N = 1 |
| `p.bess.Pmax` | 30e6 W | max charge/discharge power, plant total | `par.Pmax` = 1.0 pu |
| `p.bess.E` | 60e6·3600 J (60 MWh) | energy capacity, plant total | `kSOC` |
| `p.bess.SOC0` | 0.5 | initial SOC | `ap.SOC0` |
| `p.bess.SOCmin` | 0.1 | no discharge at or below | `par.SOCmin` |
| `p.bess.SOCmax` | 0.9 | no charge at or above | `par.SOCmax` |
| `ap.kSOC` | 1.39e-4 1/s (derived) | dSOC/dt per pu power | Gain `kSOC` |

### B3.7 Parameter overview across the three asset types

| Function | WT (IEC) | PV (REEC_B / REGC_A) | BESS (REEC_C / REGC_A) |
|---|---|---|---|
| Unit rating (Pbase) | 75 MW | 25 MW | 30 MW |
| Voltage filter | `Tufilt` 0.01 | `Trv` 0.02 | `Trv` 0.02 |
| Q measurement filter | `Tqfilt` 0.01 | none (raw Q_meas) | none |
| Dip detection | U < `udip` 0.9 | V < `Vdip` 0.9 or V > `Vup` 1.1 | same as PV |
| P limits | 0 … P_wind_avail | 0 … P_pv_avail | ±`Pmax` (1.0) |
| P ramp | `dpmaxp4A` ±1 pu/s | `dPmax`/`dPmin` ±99 | ±99 |
| P order lag | `Tpordp4A` 0.01 | `Tpord` 0.02 | `Tpord` 0.02 |
| Q loop | `MqG` 1, `kpq` 0.1, `kiq` 10 | `Qflag` 1, `Kqp` 0.1, `Kqi` 10 | same as PV |
| Q current lag | none | `Tiq` 0.02 | `Tiq` 0.02 |
| Dip current | `Kqv` 2·(`uref0` − U), no deadband | `Kqv` 2·dz(`Vref0` − V, ±0.05) | same as PV |
| Reactive current limit | `iqH1` ±1.05 (always) | `Iqh1`/`Iql1` ±1.05 (on Iqinj) | same as PV |
| Current limit | `imax` 1.1 | `Imax` 1.1 | `Imax` 1.1 |
| Priority | P normally, Q in a dip | `PQflag` 0: Q always | same as PV |
| i_p sign | ≥ 0 | ≥ 0 | ± (SOC limits) |
| Converter lag | `Tg` 0.01 | `Tg` 0.02 | `Tg` 0.02 |
| i_p rate limit | `dipmax` 1 pu/s (rise) | `Rrpwr` 10 pu/s (rise) | same as PV |
| i_q rate limit | `diqmax` ±100 pu/s | `Iqrmax`/`Iqrmin` ±99 | same as PV |
| LVPL / LVACM | no | yes | yes |
| Energy model | no | no | SOC integrator |

### B3.8 Behaviour seen in the cases

All numbers below are from `docs/design.md` and the commit messages, measured at the time stated. Later stages changed the plant and HPPC controls, so some values differ in today's runs; the asset models themselves have not changed in structure since stage 4, apart from the asset Q PI gain (`kiq`, `Kqi` 5 → 10 /s in stage 5 step 4).

#### Stage 3 (wind asset only, 2026-11-16), case set `wt_frt`

`wt_frt` (`scripts/+caseSets/wt_frt.m`): grid source voltage dips for 150 ms (`p.grid.dip.dur` = 0.15 s), "no dip", "dip 0.5", "dip 0.8", "dip 0.5 SCR3". Today the dip starts at 45 s (`grid.dip.t0` = 45 in the case set), after the plant has ramped up following the wind step.

- Steady state identical to stage 2 (P_poc 146.358 MW, Q_poc −21.3 Mvar).
- Dip 0.5: V_poc ≈ 0.53 pu, the wind plant injects ≈ 50 Mvar, P_poc ≈ 92 MW; after clearing P ramps back at `dipmax` = 1 pu/s and is back at 146.4 MW within about 0.6 s.
- Dip 0.8: V_poc ≈ 0.26 pu, Q ≈ 51 Mvar, P ≈ 30 MW.
- SCR 3, dip 0.5: V_poc ≈ 0.62 pu, Q ≈ 53 Mvar, P ≈ 62 MW.
- Ringing in the first ≈ 30 ms of a dip and at clearing, single-step spikes up to ≈ 180 Mvar (one-step-delayed current sources against an instantaneous network).

#### Stage 4 (PV and BESS assets, 2026-11-16)

- Steady state identical to stage 3 (P_poc 146.358 MW, Q_poc −21.28 Mvar at 55 s).
- Dip 0.5: V_poc ≈ 0.56 pu (0.53 in stage 3: the PV and BESS now support the voltage); Q wind ≈ 47, PV ≈ 17, BESS ≈ 11 Mvar; PV P drops to ≈ 13 MW (LVACM); all back to pre-fault values within ≈ 0.1 s.
- Dip 0.8: V_poc ≈ 0.32 pu, PV P ≈ 0.8 MW.
- `bess_soc`: at SOCmin the BESS gives 0 MW against a 30 MW discharge set-point and still charges afterwards; at SOCmax it discharges, and charging stops when SOC reaches 0.9.
- `Tg` = 20 ms in PV/BESS removed most of the ringing at the dip start; the single-step spike at fault clearing remains (Q_poc ≈ 125 Mvar for one step).
- With the closed asset and plant Q loops: dip 0.5, Q wind ≈ 56, PV ≈ 17, BESS ≈ 10 Mvar; P_poc back at 146.1 MW 0.35 s after clearing.

#### Stage 5 (HPPC closed loops)

- `wt_frt`: P_poc back at 150 MW 0.10 s (dip 0.5), 0.46 s (dip 0.8), 0.45 s (dip 0.5, SCR 3) after clearing, no overshoot (step 1). After step 3: P_poc relative to the no-dip case back within 1.5 MW 0.13 / 0.51 / 0.29 s after clearing. The report figure statements (`scripts/reportFigures.m`, `frt_vq`, `frt_p`): "reactive current injection supports the POC voltage" and "P recovers within 0.5 s after clearing, without overshoot (HPPC P loop frozen during the dip)".
- `bess_soc` (stage 5 step 3): SOC0 0.15 → BESS discharge derated to 14.6 MW (29.7 at SOC 0.5) by the HPPC; at SOCmin 0 MW.

#### Availability step

The wind availability step at 20 s (0.6 → 0.9 pu) is in every case. Measured effects: before the step the plant cannot reach 150 MW (P_poc 137.8 MW with the BESS at its discharge limit, stage 5 step 1); at the step P_poc dips by about 13 MW for 0.3 s because the BESS is redispatched on availability before the wind output has followed (open point in design.md section 10); with the grid-code ramp limit the HPPC ramps the plant up at 36 MW/min after the step.

### B3.9 Known limitations

From design.md (stages 3–4, section 10) and from the code:

1. **Parameter values are placeholders**, typical values from WECC example sets and not from a specific turbine or inverter; IEC values are not from the standard's tables.
2. **IEC Type 4A details unchecked**: filter forms, dip logic (no hysteresis, no post-fault hold), the reference of `Kqv` (fixed `uref0` instead of pre-fault voltage), current-limitation options (voltage-dependent limits, priority flags), the `uqdip` / `Mqfrt` FRT modes, Q control modes other than MqG 1/2.
3. **WECC simplifications**: no VDL1/VDL2, no state freeze and no `Thld`/`Thld2` hold during a dip (only the Q PI is frozen), no `Pflag` (P·ω), no `Vflag` voltage PI in the Q path, no high-voltage reactive current management, no `Tfltr` in REGC_A, LVPL static on the unfiltered voltage and applied after the lag, rate limits before the lag instead of on its state.
4. **No PLL** and no converter inner-loop dynamics beyond `Tg`; the voltage angle is followed exactly with one step of delay.
5. **Single-step spike at fault clearing** from the one-step-delayed current sources (Q_poc ≈ 125 Mvar for one step).
6. **No P loop at asset level**; `P_meas` is unused in all three controllers.
7. **Wind turbine cannot absorb active power** (i_p ≥ 0), and its availability is a profile input with no aerodynamics or drive train (Type 4A).
8. **PV availability is constant** (0.4 pu); no irradiance dynamics.
9. **Battery energy model is ideal**: no efficiency, losses, self-discharge or SOC-dependent limits; the SOC limit is a hard switch with no hysteresis.
10. **No initialisation**: controllers start from zero measurements, currents ramp up from zero at the start of each run.
11. **No protection** (under/over-voltage or frequency trips), and the assets do not respond to frequency themselves (frequency response sits in the plant controllers and the HPPC).
12. **One parameter set for all instances** of a type (per-instance values are possible through `ap(k)` but not used).

### B3.10 Templates and models

- **Logic:** the MATLAB Function code in the six `.slx` files (`current_limitation` in `WT_ctrl`, `current_limit_logic` in `PV_ctrl` and `BESS_ctrl`, `generator`, `terminal` and `lvpl_lvacm` in the asset models) is identical to the template strings in `buildWTCtrlType4A`, `buildReecCtrl` and `buildAsset`. The block sets, block parameters (gains, limits, pole expressions, initial conditions, PI settings) and connections in the models match what the templates create. No hand edits were found in these six models.
- **Model configuration:** the asset models' own solver setting (ode4, `Tsim`) comes from the template; inside `HPP.slx` the top model's ode1 is used (see B3.2).
- **Docs vs code** (the code is described above): design.md stage 3 says the WT reactive current is limited to `iqH1` "during a dip"; the code applies `iqH1` always. design.md section 4.3 lists `omega_gen` (4B) as a WT controller input; it does not exist. design.md section 2 says asset controllers run at "Ts = sim step"; that is what the code does (`ctrl.<asset>.Ts = p.sim.Ts`).

### B3.11 Recreating it

1. **Parameters.** Write `params/windTurbineParameters.m`, `pvParameters.m`, `batteryParameters.m` with the layout fields (`Pn`, `nAgg`, `nFeeders`, `nPerFeeder`), the availability inputs (`Pavail0`, `PavailStep`, `tStep`; `Pavail`), the BESS energy fields (`Pmax`, `E`, `SOC0`, `SOCmin`, `SOCmax`) and the standard parameters under `iec.ctrl` / `iec.gen` and `wecc.ctrl` / `wecc.gen` with the values in the tables above.
2. **Controller parameters.** In `controllerParameters`, create `ctrl.wt`, `ctrl.pv`, `ctrl.bess` with `Ts = p.sim.Ts`, `Pbase = nAgg·Pn`, `par.Vmin = 0.01`; copy the `ctrl` fields into `par`; add `par.MqG1 = double(MqG == 1)` (WT), `par.Pmax` (pu), `par.SOCmin`, `par.SOCmax` (BESS). Keep every `par` expression used in block parameters to + − × ÷.
3. **Instance arguments.** In `deriveParameters`, build `p.<type>.ap` (N × 1) with `ctrl`, `gen`, and for the BESS `SOC0` and `kSOC = Pbase / (E/N)`.
4. **Asset controllers.** Create `WT_ctrl.slx`, `PV_ctrl.slx`, `BESS_ctrl.slx`: fixed-step discrete, `FixedStep = Ts`, model workspace code that loads `Ts` and `par` from `HPP_parameters()`, model argument `par`; six `double` inports with `SampleTime = Ts` and three outports in the order of `modelInterfaces.m`. Add the blocks of B3.4 / B3.5 / B3.6: discrete first-order lags (`PoleZ = 1 − Ts/T`, voltage filter IC 1), voltage floor, dip comparison (plus `V > Vup` and OR for REEC), Saturation Dynamic for the P limits, Rate Limiter, P-order lag, division by the floored voltage; Q feed-forward, Q error, the two switches (Q loop on, freeze in dip), the discrete PI with clamping and ±`imax`/`Imax` limits, the dip-current path (deadband only for REEC), the `iqH1` or `Iqh1/Iql1` saturation, `Tiq` for REEC; the current-limitation MATLAB Function with `par` as a Parameter. Or run `buildHPP(All=true)` with the models missing to generate them from the templates.
5. **Asset models.** Create `<type>_asset.slx`: fixed step `Tsim`, model workspace loading `ap` and `Tsim`, model argument `ap`; inports `P_set`, `Q_set`, `V_re`, `V_im` (+ availability for WT/PV); MATLAB Function `terminal` (|V|); Memory blocks on P and Q; Model block `ctrl` referencing the controller with `par = ap.ctrl`; the generator system (WT: `dipmax` rising only, `diqmax` ±; REGC_A: `Rrpwr` rising only, `Iqrmax`/`Iqrmin`, then LVPL/LVACM after the active-current lag); lags as Gain `1/(ap.gen.Tg)` + Integrator with feedback; MATLAB Function `generator` (current aligned with V, P = |V| i_p, Q = |V| i_q); for the BESS the Gain `-ap.kSOC` and Integrator with IC `ap.SOC0`; outports `P`, `Q`, `P_avail`, `I_re`, `I_im` (+ `SOC`).
6. **Interface list.** Enter the ports and arguments in `utilities/build/modelInterfaces.m` and run `checkModels` (ports, tunable expressions, `par`/`ap` fields).
7. **Wiring in `HPP.slx`.** In `buildPlantSub`: one Model block per asset with `ap = p.<type>.ap(k)`; set-points from the plant controller through the plant→asset link (delay, gain plant/asset base); `V_re`/`V_im` from the network bus (one step old, Memory with IC 1 + j0 in the `Network` subsystem); a Step block (wind: 0.6 → 0.9 pu at 20 s) or Constant (PV: 0.4 pu) for the availability; asset `P`/`Q` terminated; `P_avail` and `SOC` muxed back to the plant controller (gain, memory, delay); `I_re`/`I_im` muxed and scaled by asset base / 180 MVA into the network.
8. **Solver.** Top model fixed-step ode1, single-tasking, step `p.sim.Ts` = 1 ms; keep every time constant well above 1 ms.
9. **Check.** Run the nominal case (steady state; asset Q loops settle to their set-points), `wt_frt` (dip 0.5: reactive current injection, P back within about 0.5 s after clearing) and `bess_soc` (no discharge at SOCmin, charging stops at SOCmax) and compare with the numbers in B3.8.

## B4. Plant controllers (WPPC, SPPC, BESSC: WECC REPC_A)

### B4.1 Role in the hierarchy

**What.** There are three plant controllers: `models/ctrl/WPPC.slx` (wind), `SPPC.slx` (PV) and `BESSC.slx` (battery). Each one sits between the HPPC and the N asset controllers of its plant. It closes its own loops on its own measurement point, which is the plant's feeder into the 33 kV HPP busbar. Then it splits its P and Q orders over its N assets.

All three are built from one template, `buildPlantCtrlRepc(name, folder)` in `utilities/build/buildHPP.m` (~line 2212). The structure follows the WECC second-generation plant controller **REPC_A**: a P path with frequency droop and a PI, and a Q path that either tracks a Q set-point or controls a voltage, also with a PI. BESSC has a few extra blocks for the state of charge (SOC). WPPC and SPPC have identical diagrams. Only their parameter values differ, and those come in through the model argument `par`.

**Why closed loops in the plant controllers.** The decision was made on 2026-11-16 (design.md section 8, stage 4, "Closed loops at every level"; memory note `phd-vendor-plant-controllers.md`). In real hybrid plants the vendors deliver the assets together with their own plant controller. That controller regulates P, Q/V and frequency at its own point and is a black box to the plant owner. The HPP controller then has to coordinate several closed-loop controllers that it cannot change. This brings cascade interaction, parallel voltage control and duplicated frequency response. Coordinating them is the problem the PhD project addresses. So the plant controllers are kept realistic and self-contained on purpose, and their interaction with the HPPC is not simplified away.

**Alternatives considered.** The first option was an open-loop "splitter" plant controller: the stage-1 placeholder (`controllerSpecs` in `buildHPP.m`, ~line 150) only split the set-points by availability. It was dropped because the coordination problem would then not exist. The second option was the IEC 61400-27-1 WPP control model for the wind plant. It is postponed (design.md section 10: "the IEC 61400-27-1 WPP control model is a later refinement"), because it has the same functions as REPC_A, and one template for all three plants is simpler.

**Consequences.** Each plant controller is a future DLL. Its ports are fixed by `utilities/build/modelInterfaces.m`, so a port change needs changes in `modelInterfaces.m`, in the build script's wiring and in the model, all together. The HPPC can only reach a plant through the plant's P and Q set-points. It sees the plant through the measurements and flags the plant controller sends back.

### B4.2 Sample time, per-unit base and model argument

| Item | Value | Where |
|---|---|---|
| Sample time `Ts` | 0.02 s (20 ms) | `controllerParameters.m`, `plantCtrl()`: `c.Ts = 0.02` |
| Per-unit base `Pbase` | plant rating `p.<type>.Prated`: wind 150 MW, PV 50 MW, BESS 30 MW | `plantCtrl(Prated, N)` |
| Asset count `N` | width of the vector ports (default wind 2, PV 2, BESS 1) | model workspace: `N = numel(par.share)` |
| Model argument | `par` = `p.ctrl.wppc.par` / `sppc.par` / `bessc.par` | `ctrlHeader(..., withN=true)` |

A **model argument** is a parameter of a referenced model (a model used inside another model through a Model block) that each Model block instance can set to its own value. Here the top model `HPP.slx` passes `p.ctrl.<plant>.par` to each plant controller's Model block. Inside the controller, every block parameter refers to fields of `par`.

The plant controller works in per unit on its own plant rating. The conversions from W and var to pu happen outside the controller, in the "link" blocks of `HPP.slx` (chapter A3). Voltages and frequency are already in pu. The 20 ms rate is five times faster than the HPPC (100 ms) and twenty times slower than the asset controllers (1 ms = the simulation step). Why these rates were chosen and how the delays are applied is described in chapter B6.

A discrete first-order lag is used for every filter: `lagBlk(sys, name, T, y0)` creates a "Transfer Fcn First Order" block with pole $p = 1 - T_s/T$, i.e. $y_k = p\,y_{k-1} + (1-p)\,u_k$. Note that with $T = T_s$ the pole is 0 and the "filter" passes its input unchanged. This is the case for the plant's `Tfltr` = `Tp` = 0.02 s at `Ts` = 0.02 s, so the plant measurement filters currently do nothing (inferred from the block formula; it is not commented anywhere). A filter constant below `Ts` would make the pole negative, so time constants must be at least `Ts`.

### B4.3 Ports (from `utilities/build/modelInterfaces.m`)

| # | Input | Width | Unit | Source in `HPP.slx` (link) |
|---|---|---|---|---|
| 1 | `P_set` | 1 | pu (plant) | HPPC, delay `comm.hppc_to_plant` 50 ms, then W → pu |
| 2 | `Q_set` | 1 | pu (plant) | HPPC, same link |
| 3 | `P_meas` | 1 | pu | plant flow into the 33 kV busbar; pu, memory, delay `comm.meas_plant` 20 ms |
| 4 | `Q_meas` | 1 | pu | same |
| 5 | `V_meas` | 1 | pu | **33 kV busbar voltage `V_bus`** (common to all three plants); memory, delay 20 ms (initial 1) |
| 6 | `P_avail_k` | N | pu (plant) | asset availabilities; asset → plant base, memory, delay 20 ms |
| 7 | `SOC_k` (BESSC only) | N | – | asset SOC; memory, delay 20 ms |
| last | `f_meas` | 1 | pu of 50 Hz | network frequency, memory only (measured locally, no communication delay) |

| # | Output | Width | Meaning |
|---|---|---|---|
| 1 | `P_set_k` | N | P set-points to the assets |
| 2 | `Q_set_k` | N | Q set-points to the assets |
| 3 | `P_avail` | 1 | sum of `P_avail_k` (to the HPPC) |
| 4 | `P_plant` | 1 | = `P_meas`, passed through to the HPPC |
| 5 | `Q_plant` | 1 | = `Q_meas`, passed through |
| 6–8 (BESSC) | `P_chg_max`, `P_dis_max`, `SOC` | 1 | plant charge / discharge limits and share-weighted SOC |
| last | `Q_sat` | 1 | +1 / 0 / −1: Q order at its upper limit / free / at its lower limit |

The wiring is in `buildPlantSub` (`buildHPP.m`, ~lines 995–1034). One detail matters for the voltage control: all three plant controllers measure **the same busbar voltage**. They are parallel voltage controllers on one node. This is why a droop is needed and why it must be positive (section B4.6).

### B4.4 Measurements, availability and P limits

```
P_filt = lag(P_meas, Tp)      Q_filt = lag(Q_meas, Tfltr)      V_filt = lag(V_meas, Tfltr, y0 = 1)
P_avail = sum(P_avail_k)
```

The upper P limit is `P_avail` for WPPC and SPPC. The lower limit is the constant `Pmin = 0`, because wind and PV cannot absorb power.

BESSC has a MATLAB Function block, `bess_limits` ("BESS limits"):

```matlab
dis = P_avail_k .* (SOC_k > par.SOCmin);
chg = P_avail_k .* (SOC_k < par.SOCmax);
P_dis_max = sum(dis);   P_chg_max = sum(chg);
SOC = sum(par.share .* SOC_k);
```

An asset may discharge only above `SOCmin` and charge only below `SOCmax`. For a BESS asset, `P_avail_k` is its converter power limit (REEC_C `Pmax`, from `buildReecCtrl`). The upper P limit of BESSC is `P_dis_max` and the lower limit is `−P_chg_max` (gain "charge limit" = −1). `SOCmin` = 0.1 and `SOCmax` = 0.9 come from `p.bess` (copied in `controllerParameters`). The same hard limits are applied a second time inside each BESS asset controller (REEC_C: no discharge at SOC ≤ SOCmin, no charge at SOC ≥ SOCmax). The HPPC adds a soft derating on top (B5.4). Three layers of SOC protection may look redundant. They reflect the real split: the BMS and converter protect the cells, the plant EMS knows which units can act, and the HPPC plans ahead (the reasoning is in design.md section 2, "Why the battery has its own asset controller").

### B4.5 P path: frequency response and P loop (`Frqflag`)

**What.** The frequency response follows REPC_A: a deadband, then separate droop gains for over- and under-frequency.

$$\Delta f = f_{ref} - f_{meas}, \qquad \Delta f_{db} = \mathrm{deadzone}(\Delta f;\ f_{dbd1}, f_{dbd2})$$
$$P_{ref} = P_{set} + D_{up}\max(\Delta f_{db},0) + D_{dn}\min(\Delta f_{db},0)$$

The Simulink Dead Zone block outputs $u - f_{dbd2}$ above the band (and $u - f_{dbd1}$ below it). So the droop is measured from the edge of the deadband, not from 50 Hz. Under-frequency gives $\Delta f > 0$ and raises P; over-frequency lowers it.

The P loop:

$$e_P = \mathrm{sat}(P_{ref} - P_{filt};\ f_{emin}, f_{emax})$$
$$u_{PI} = \mathrm{PI}(e_P;\ K_{pg}, K_{ig}),\quad \text{limited to } [P_{lo} - P_{ref},\ P_{hi} - P_{ref}]$$
$$P_{ord} = \mathrm{lag}\big(\mathrm{sat}(\textit{Frqflag}\ ?\ P_{ref} + u_{PI} : P_{set};\ P_{lo}, P_{hi}),\ T_{lag}\big)$$

$P_{hi}$ is `P_avail` (or `P_dis_max` for BESSC) and $P_{lo}$ is 0 (or `−P_chg_max`). The switch "closed loop?" chooses between the closed loop (`Frqflag` ≠ 0) and passing `P_set` through unchanged. With `Frqflag` = 0 there is **no droop and no P loop**: `P_set` is only limited and lagged. The PI block still runs in the background, but its output is not used. Its clamping keeps the integrator bounded.

**The PI block.** Every PI in the plant controllers and in the HPPC is a "Discrete PID Controller" in PI mode (`piBlk` in `buildHPP.m`). It uses the Forward Euler integrator, external saturation limits (two extra input ports) and **clamping anti-windup**: the integrator stops while the output is saturated and the error would drive it further. The limits on the PI output are "limit minus feed-forward", so the sum of feed-forward and PI stays inside the limits.

**Deviation from REPC_A: feed-forward.** In the WECC REPC_A the PI output alone is the order. Here the reference is added in front of the PI ("ref + PI"), and the PI only corrects the remaining error (losses, asset deviations). Why: with the REPC_A example gains, the PI alone would take tens of seconds to reach a set-point step. Feed-forward is also common in vendor plant controllers (design.md stage 4; comment in `plantControlParameters.m`).

**Why `Frqflag` = 0 by default (central control).** Since stage 5, step 1 (2026-11-17, commit 3adff21), the HPPC closes the P loop at the POC and gives the frequency response of the whole plant. If the plant controllers kept their own droop and P loops, the frequency response would be counted twice, and two integrators (HPPC and plant) would fight over the same power. In REPC_A, `Frqflag` switches both the droop and the plant P loop, so `Frqflag` = 0 opens both. The stage-4 behaviour (every plant with its own droop and P loop, open-loop HPPC) is kept as the **decentral** cases (B5.15).

**Why `Dup` = 0 for wind and PV.** Both run at available power, so they have no headroom to increase output on under-frequency (`plantControlParameters.m`: `pc.wppc.Dup = 0; pc.sppc.Dup = 0`). Only the BESS responds upwards in the decentral setup.

### B4.6 Q path: Q control and voltage control (`RefFlag`)

**What.** The Q path has two modes, chosen by `par.RefFlag`:

- **Q control** (`RefFlag` = 0, default): the error is $Q_{set} - Q_{filt}$.
- **Voltage control** (`RefFlag` = 1): the error is the distance from a V-Q droop line,
$$e_V = V_{ref} - \big(V_{filt} + K_c\,(Q_{filt} - Q_{set})\big).$$
In steady state the PI drives $e_V$ to zero, so
$$Q = Q_{set} + \frac{V_{ref} - V}{K_c}.$$
$Q_{set}$, sent by the HPPC, is the **offset** of the droop line, and $K_c$ is its slope.

The rest of the chain is the same in both modes:

```
e     = RefFlag ? e_V : (Q_set - Q_filt)
e     = deadzone(e; dbd1, dbd2)
e     = sat(e; emin, emax)
e     = (V_filt < Vfrz) ? 0 : e                 % freeze in a dip
Q_PI  = PI(e; Kp_eff, Ki_eff), limited to [Qmin - Q_set, Qmax - Q_set]
Q_u   = Q_set + Q_PI                            % feed-forward of Q_set in both modes
Q_ord = lag(Q_u, Tfv)
Q_sat = q_saturation(Q_u)
```

The "Q feed-forward" switch has its control input tied to the constant 0, so `Q_set` is always fed forward. This is a leftover of the step-2 design, where voltage control had no feed-forward. Since step 4 the offset is fed forward in both modes, and the switch was kept with a constant (template comment: "Q_set fed forward in both modes").

**Mode-dependent PI gains.** The PI gains are a single tunable expression, because block parameters with model arguments may only use + − × ÷ (chapter A2/A3, design.md section 13):

```
P = par.Kp + par.RefFlag*(par.KpV / (par.Kc + par.dVdQ) - par.Kp)
I = par.Ki + par.RefFlag*(par.KiV / (par.Kc + par.dVdQ) - par.Ki)
```

With `RefFlag` = 0 this gives `Kp`, `Ki`. With `RefFlag` = 1 it gives $K_{pV}/(K_c + dV/dQ)$ and $K_{iV}/(K_c + dV/dQ)$.

**Why the voltage gains are normalised.** The loop gain from the Q order to the voltage error is $\partial e_V/\partial Q = -(K_c + dV/dQ)$, where $dV/dQ$ is how much the busbar voltage moves per unit of plant Q. Dividing the PI gains by this loop gain makes the closed-loop speed about the same for any droop and any grid strength. `KpV` and `KiV` are therefore "normalised" gains: 0.5 and 2.5 /s give a time constant of about $(1+K_p)/K_i$ = 0.6 s.

**The busbar droop line (set by `controllerParameters`).** With `hppc.RefFlag` = 1 and `hppc.Vlocal` = 1 (the default for voltage control), `controllerParameters.m` (~lines 52–96) overwrites three fields of every plant controller's `par`:

| Field | Value | Meaning |
|---|---|---|
| `RefFlag` | 1 | plant in voltage control |
| `Vref` | `p.hppc.Vref` (1.0) | same voltage reference as the HPPC |
| `Kc` | `KcBus · sumP` | droop on the plant's own rating |
| `dVdQ` (always set) | `xBus · sumP` | busbar dV/dQ of all plants together, on the own rating |

with

$$x_{Tr} = u_k\frac{X/R}{\sqrt{1+(X/R)^2}}\cdot\frac{S_b}{S_{Tr}} = 0.12\cdot\frac{40}{\sqrt{1601}}\cdot\frac{180}{250} = 0.0864 \text{ pu on 180 MVA}$$
$$x_{Bus} = 1/SCR + x_{Tr}, \qquad K_{c,POC} = V_{slope}/Q_{max}, \qquad K_{c,Bus} = \max(K_{c,POC} - x_{Tr},\ K_{cBusMin})$$
$$\textit{sumP} = \sum P_{rated}/S_b = 230/180 = 1.278$$

($S_b$ = `p.grid.Pbase` = 180 MVA; $1/SCR$ uses the POC base, because SCR = $S_{sc}/P_{max,poc}$; the grid resistance is ignored.)

**Why the droop is "POC slope minus transformer reactance".** The grid code asks for a slope at the POC: $V_{POC}$ changes by `Vslope` for a Q change of `Qmax`. The plants regulate the 33 kV busbar instead. Between busbar and POC sits the 33/132 kV transformer with reactance $x_{Tr}$, so $V_{bus} \approx V_{POC} + x_{Tr} Q$. If the busbar droop is $K_{c,POC} - x_{Tr}$, the plants alone already put V and Q at the POC close to the required slope. The HPPC then only needs to trim slowly (B5.10). For the default 4 % slope: $K_{c,POC}$ = 0.04/0.33 = 0.1212, $K_{c,Bus}$ = 0.1212 − 0.0864 = 0.0348 pu on 180 MVA (design.md: "0.121 − 0.086 = 0.035").

**Why a minimum droop `KcBusMin` = 0.02.** For slopes below about 2.9 % the difference becomes negative (2 %: 0.0606 − 0.0864 < 0). Three parallel controllers on one node with zero or negative droop have no unique Q split: integrators push Q from one plant to another ("circulating Q", `controllerParameters.m` comment). A positive droop gives each plant a unique operating point. With the floor active, the plants give a steeper slope than required, and the HPPC trim corrects it within its about 3 s (design.md step 4: "At a 2 % slope the needed busbar droop is negative, so `KcBusMin` applies and the trim pulls the slope right within its ~3 s").

**Why "times sumP" and equal droop on each rating.** Each plant has its droop on its own rating. When all plants have the same per-unit droop $k$, each plant gives $Q_i/P_{rated,i} = (V_{ref}-V)/k$, so the Q split follows the ratings. The sum in pu of $S_b$ is $(V_{ref}-V)\,\textit{sumP}/k$, so $k = K_{c,Bus}\cdot\textit{sumP}$ gives exactly $K_{c,Bus}$ for all plants together. The same reasoning gives `dVdQ = xBus · sumP`: all plants move together (same voltage, same law), so one plant's per-unit Q change comes with the total change $q\cdot\textit{sumP}$ on $S_b$.

Resulting values (4 % slope):

| | SCR 10 | SCR 3 |
|---|---|---|
| $x_{Bus}$ [pu on 180 MVA] | 0.1864 | 0.4197 |
| plant `Kc` (own rating) | 0.0445 | 0.0445 |
| plant `dVdQ` (own rating) | 0.238 | 0.536 |
| effective `Kp` = 0.5/(Kc+dVdQ) | 1.77 | 0.86 |
| effective `Ki` = 2.5/(Kc+dVdQ) [1/s] | 8.85 | 4.30 |

(computed from the formulas and the default parameters, not from a simulation log)

SCR here means the commissioned value: `p.hppc.SCRset`, or the actual `p.grid.SCR` when `SCRset` is NaN (the default). A controller set for the wrong grid is one of the `hppc_qv` cases (B7).

**Why the plant Q limits are not tightened.** Plant order limits are `Qmax` = 0.5, `Qmin` = −0.5 pu on the plant rating. When the POC Q limit was needed in voltage control (step 4), tighter plant order limits were tried and dropped. The plant limits act on the Q *order to the assets*, i.e. on Q at the asset terminals. The wind plant loses up to about 40 % of that Q in its unit transformers, 10 kV cables and 10/33 kV transformer. With tightened limits the wind plant stopped at 28 instead of 49 Mvar at the busbar, below its share (design.md step 4; comment in `controllerParameters.m`, ~lines 69–73). The POC limit is held by the HPPC instead (B5.11).

**Freeze in a dip.** "V < Vfrz" compares the filtered busbar voltage with `Vfrz` = 0.7 pu. While it is true, the switch "freeze" sets the PI error to 0, so the integrator holds and the proportional part is zero. During a fault, the asset controllers inject reactive current on their own (FRT, chapter B3). A plant PI that kept integrating would wind up and disturb the recovery.

**Filters and lag.** `Tfltr` = 0.02 s on Q and V (no effect at Ts = 0.02 s, see B4.2). `Tfv` = 0.05 s is a first-order lag on the Q order. It stands for the REPC_A lead-lag with `Tft` = 0 (`plantControlParameters.m` comment).

### B4.7 `Q_sat`: the saturation flag

**What.** A MATLAB Function block "Q saturation" (`q_saturation`):

```matlab
tol = 1e-4;
Q_sat = double(Q_u >= par.Qmax - tol) - double(Q_u <= par.Qmin + tol);
```

It reads `Q_u`, the order before the `Tfv` lag, and gives +1 at the upper limit, −1 at the lower limit and 0 in between. It is the last output of every plant controller.

**Why.** The HPPC sends a Q offset to all plants. When all of them are at their limits, more order cannot change Q, but the HPPC's integrator does not know this. It keeps integrating and winds up. After the event it then takes a long time to unwind (9.5 s measured, B5.12). The HPPC's own order limits do not help, because in the voltage cascade the order range is wide on purpose (B5.10). So the plant has to *report* that it is saturated. Real vendor plant controllers often provide such a status signal (inferred). Here it was added as a DLL port in step 4 (commit 309edbb).

**Alternatives.** The HPPC could estimate saturation from the measured plant Q against an assumed capability. That is less reliable, because Q at the busbar differs from Q at the terminals by the losses, as above (inferred). Tracking-mode anti-windup (sending the plant's actual order back) would need an analogue signal and more interface (inferred).

**Consequences.** The flag changed the interface: plant controllers got one more output, and the HPPC grew to 20 inputs. The link from the plant to the HPPC for flags (`flag` in `buildHppcSub`) has a memory and the 20 ms measurement delay but no unit conversion.

### B4.8 Dispatch to the assets

A MATLAB Function block "dispatch" (`dispatch`) splits the orders:

```matlab
a = P_avail_k;                              % WPPC, SPPC
% BESSC: a = P_avail_k .* (SOC_k > SOCmin) if P_ord >= 0, else .* (SOC_k < SOCmax)
s = sum(a);
if s > 1e-6, w = a / s; else, w = par.share; end
P_set_k = P_ord * w;
Q_set_k = Q_ord * par.share;   % Q by rating share
```

`par.share` = ones(N,1)/N (all units in a plant have the same rating). P is split by availability, so units with more wind or sun get more. Q is split by rating, even when nothing is available, because a converter can give Q at zero P. For BESSC, only units that may discharge (or charge) share a discharge (or charge) order.

### B4.9 Parameter table (`params/plantControlParameters.m`, function `repcDefaults`)

Per unit on the plant rating, times in s. All three plants use the same values, except `Dup` = 0 for WPPC and SPPC. Values are typical placeholders, not from a vendor.

| Parameter | Value | Meaning |
|---|---|---|
| `Tfltr` | 0.02 | V and Q measurement filter (no effect at Ts 0.02 s) |
| `RefFlag` | 0 | 0 Q control, 1 voltage control (set to 1 by `controllerParameters` in the voltage cascade) |
| `Vref` | 1.0 | voltage reference (overwritten with `hppc.Vref` in the cascade) |
| `Kc` | 0.0 | droop [pu V / pu Q] (set by `controllerParameters` in the cascade) |
| `dbd1`, `dbd2` | 0, 0 | deadband on the V / Q error |
| `emax`, `emin` | 0.1, −0.1 | error limits |
| `Kp`, `Ki` | 0.1, 1.0 /s | Q PI (time constant (1+0.1)/1 = 1.1 s) |
| `KpV`, `KiV` | 0.5, 2.5 /s | normalised voltage PI (about 0.6 s) |
| `dVdQ` | 0.3 (default, overwritten) | busbar dV/dQ on the own rating, from `controllerParameters` |
| `Vfrz` | 0.7 | Q/V PI frozen while V_filt < Vfrz |
| `Qmax`, `Qmin` | 0.5, −0.5 | Q order limits |
| `Tfv` | 0.05 | Q order lag |
| `Tp` | 0.02 | P measurement filter (no effect at Ts 0.02 s) |
| `Frqflag` | 0 | 1: droop and P loop (decentral), 0: P_set passed on (central) |
| `Freq_ref` | 1.0 | frequency reference [pu] |
| `fdbd1`, `fdbd2` | −0.0004, 0.0004 | frequency deadband (±20 mHz) |
| `Ddn` | 20 | over-frequency droop gain = 1/5 % |
| `Dup` | 20 (BESS), 0 (wind, PV) | under-frequency droop gain |
| `femax`, `femin` | 0.5, −0.5 | P error limits |
| `Kpg`, `Kig` | 0.1, 1.0 /s | P PI |
| `Tlag` | 0.1 | P order lag |
| `share` | ones(N,1)/N | fallback split (`controllerParameters`) |
| `SOCmin`, `SOCmax` (BESSC) | 0.1, 0.9 | from `p.bess` |

### B4.10 Verification of the plant controllers (stage 4, recorded in design.md)

- Each plant meets its own set-point at its busbar feeder: Q_wpp, Q_spp, Q_bess → 0 within about 5 s; P_bess = −5.00 MW exactly. Q_poc improved from −21.3 to −10.4 Mvar. The rest is on the 132 kV side and is for the HPPC.
- Frequency +0.2 Hz with plant droop: each plant lowers P by 5 % × (0.2 Hz − 20 mHz) = 7.2 % of its rating (wind 135 → 124.2 MW, PV 20 → 16.4 MW, BESS −5 → −7.16 MW). At −0.2 Hz only the BESS responds (−5 → −2.84 MW).

These numbers are from stage 4, before the HPPC closed its loops. They remain valid for the decentral cases.

### B4.11 Contradictions and notes

- README ("Every controller is the same kind of model: inports → one MATLAB Function block `logic` → outports") describes the stage-1 placeholders. The controllers are now block diagrams with several MATLAB Function blocks. The code (the `.slx`) wins.
- The `.slx` files and the template `buildPlantCtrlRepc` agree: same blocks, same block parameters (PI expressions, lag poles, limits) and the same MATLAB Function code (`q_saturation`, `dispatch`, `bess_limits`), checked by unzipping the models.

### Recreating it

1. Create a model with fixed-step discrete solver, `Ts` = 0.02 s, a model workspace that loads `Ts`, `par`, `N` from `HPP_parameters()`, and `par` declared as a model argument.
2. Add the inports and outports in the order of `modelInterfaces.m` (vector ports of width N for `P_avail_k`, `SOC_k`, `P_set_k`, `Q_set_k`).
3. Add measurement lags (`Tp`, `Tfltr`), the sum of availability and, for BESSC, the `bess_limits` MATLAB Function.
4. Build the frequency path: f error, Dead Zone (`fdbd1/2`), split into under/over-frequency, gains `Dup`/`Ddn`, sum with `P_set`.
5. Build the P loop: error limits, Discrete PID (PI, external limits, clamping), `ref + PI`, switch on `Frqflag` (else `P_set`), dynamic saturation [P_lo, P_hi], lag `Tlag`.
6. Build the Q path: droop error `Vref − V − Kc(Q − Q_set)`, Q error, switch on `RefFlag`, deadband, error limits, freeze below `Vfrz`, PI with the mode-dependent gain expression and limits `Qmax − Q_set`, `Qmin − Q_set`, feed-forward of `Q_set`, lag `Tfv`, `q_saturation`.
7. Add the dispatch MATLAB Function (P by availability, Q by `share`; BESS by SOC).
8. In `controllerParameters`, compute `dVdQ`, and in the voltage cascade `RefFlag`, `Vref`, `Kc` from the slope, transformer reactance, SCR and `KcBusMin`.
9. Check: Q set-point tracking at the busbar within about 5 s, droop 7.2 % at +0.2 Hz with `Frqflag` = 1.

---

## B5. HPPC: central control at the POC

### B5.1 Overview

**What.** `models/ctrl/HPPC.slx` is the HPP controller. Its template is `buildHppcCtrl` in `utilities/build/buildHPP.m` (~line 1956) and its parameters are `p.hppc` (`params/hppcParameters.m`), copied and extended into `p.ctrl.hppc.par` by `controllerParameters.m`. It runs at `Ts` = 0.1 s, in per unit on the POC base `p.grid.Pbase` = 180 MW (= Pn of the plant at the POC). Frequency is in pu of 50 Hz.

It has five function areas, each a section of the block diagram:

1. Measurements, SOC-aware BESS limits, P limits
2. Ramp limit, frequency response (LFSM-O, LFSM-U, FSM), export limit
3. P control at the POC (`Pflag`)
4. Q, V or PF control at the POC (`Qflag`, `RefFlag` 0 / 1 / 2)
5. Dispatch to the plants

Seven MATLAB Function blocks hold the logic that is awkward as blocks: `soc_limits`, `dip_freeze`, `ramp_limiter`, `frequency_response`, `poc_q_limit`, `plant_anti_windup`, `dispatch`. Everything else is standard blocks. The code in `HPPC.slx` was compared with the template: the block list, block parameters and all seven functions are identical.

**Why "central control".** Decided on 2026-11-17 (stage 5, step 1): the HPPC closes the P loop at the POC and gives the frequency response of the whole plant. The plant controllers pass their P set-points through (`Frqflag` = 0) but keep their Q/V loops. The grid code is defined at the POC, and only the HPPC measures there. The BESS headroom can only be used for the plant's under-frequency response if one controller sees all plants. The stage-4 decentral setup is kept for comparison (B5.15).

**Development history** (commits on stage 5):

| Step | Commit | Content |
|---|---|---|
| 1 | 3adff21 (2026-11-18) | closed-loop P at the POC, symmetric 5 % droop ±20 mHz, export limit, freeze in dips, dispatch with BESS balancing |
| 2 | 763e85a | Q control and voltage control (PI with droop `Kc`) at the POC, ±0.33 pu |
| 3 | 7459c87 | Danish grid code: ramp limit, LFSM-O/U, FSM with reserve, SOC-aware limits, voltage control with slope and normalised gains, PF control |
| 4 | 309edbb | voltage control as a cascade over the plants (`Vlocal`), cascade timing, POC Q limit, anti-windup over the plant limits (`Q_sat`) |

### B5.2 Ports

20 inputs, in this order (`modelInterfaces.m`): TSO (`P_ref`, `Q_ref`, `f_meas`), POC (`P_poc`, `Q_poc`, `V_poc`), wind (`P_avail_wpp`, `P_wpp`, `Q_wpp`, `Q_sat_wpp`), PV (`P_avail_spp`, `P_spp`, `Q_spp`, `Q_sat_spp`), BESS (`P_chg_max_bess`, `P_dis_max_bess`, `SOC_bess`, `P_bess`, `Q_bess`, `Q_sat_bess`). Six outputs: `P_set_wpp`, `Q_set_wpp`, `P_set_spp`, `Q_set_spp`, `P_set_bess`, `Q_set_bess`.

Links in `HPP.slx` (`buildHppcSub`, ~line 895):

| Signal group | Link |
|---|---|
| `P_ref`, `Q_ref` | W → pu, delay `comm.tso` 200 ms; the delay starts at the initial set-point, so the HPPC never sees zero at start-up |
| `f_meas` | direct, no memory and no delay (measured locally) |
| POC P, Q | W → pu, memory, delay `comm.meas_hppc` 20 ms |
| `V_poc` | memory, delay 20 ms (initial 1) |
| plant P_avail, P, Q, BESS limits | W → pu, memory, delay 20 ms |
| SOC, `Q_sat_*` | memory, delay 20 ms, no unit conversion |
| outputs to the plants | delay `comm.hppc_to_plant` 50 ms, then pu (plant base) |

`P_wpp`, `Q_wpp`, `P_spp`, `Q_spp`, `P_bess`, `Q_bess` are wired to terminators: they are inputs for later functions (for example dispatch on the measured wind output, design.md section 10).

### B5.3 Signal flow at a glance

```
P_ref ─► ramp limiter (min(P_ref, P_hi − reserve), rampUp/rampDn) ─► P_base
f_meas ─► frequency response (LFSM-O, LFSM-U, FSM) ─► dP_f
P_target = min(P_base + dP_f, Ppoc_max)
P_ord = sat(P_target + PI(lag(P_target,Tref) − P_filt), P_lo, P_hi)        (Pflag, freeze in dip)

Q_refx = PF mode ? tanφ·P_filt : Q_ref
Q error = lag(Q_refx, TrefQ) − Q_filt          or, in V mode,
V error = Vref − V_filt − (Vslope/Qmax)·Q_filt, capped by the POC Q limit
Q_ord = sat(ff + PI(error with anti-windup), QminOrd, QmaxOrd)              (Qflag, freeze in dip)

dispatch(P_ord, Q_ord, availabilities, SOC-aware BESS limits) ─► P_set_*, Q_set_*
```

### B5.4 Measurements, SOC-aware BESS limits and P limits

Filters (pole $1 - T_s/T$ at $T_s$ = 0.1 s):

| Block | Constant | Pole | Initial output |
|---|---|---|---|
| P filt | `Tp` = 0.2 s | 0.5 | 0 |
| Q filt | `Tq` = 0.2 s | 0.5 | 0 |
| V filt | `Tq` = 0.2 s | 0.5 | 1 |

The comments in `hppcParameters.m` ask for "≥ Ts" for this reason.

**SOC-aware BESS limits** (`soc_limits`):

$$k_d = \mathrm{clip}\Big(\frac{SOC - SOC_{min}}{SOC_{band}},0,1\Big),\quad k_c = \mathrm{clip}\Big(\frac{SOC_{max} - SOC}{SOC_{band}},0,1\Big)$$
$$BESS_{dis} = k_d\,P_{dis,max},\qquad BESS_{chg} = k_c\,P_{chg,max}$$

`SOCband` = 0.1, `SOCmin` = 0.1, `SOCmax` = 0.9. Discharge is derated linearly from full at SOC 0.2 to zero at SOC 0.1, and charge from SOC 0.8 to 0.9.

*Why.* Without derating the BESS runs at full power up to the hard limit and then stops abruptly. That is a step of up to 30 MW in the plant output, which the P loop then has to cover with curtailment or cannot cover at all. The derating makes the approach smooth, and the dispatch curtails wind and PV earlier when the BESS is nearly full (design.md step 3).

*Alternative not done.* SOC restoration towards a target SOC (an open point, design.md section 10).

**P limits:**
$$P_{hi} = P_{avail,wpp} + P_{avail,spp} + BESS_{dis},\qquad P_{lo} = -BESS_{chg}$$
$P_{lo}$ means wind and PV fully curtailed with the BESS charging.

**Freeze in a voltage dip** (`dip_freeze`):

```matlab
persistent n
if isempty(n), n = 0; end
if V_poc < par.Vfrz, n = round(par.Thld / Ts);
elseif n > 0, n = n - 1; end
frz = double(V_poc < par.Vfrz || n > 0);
```

It uses the unfiltered (but delayed) `V_poc`. `Vfrz` = 0.9 pu, `Thld` = 1.0 s, i.e. 10 samples after the voltage recovers. Both the P and the Q PI use `frz`.

*Why.* During a dip the assets lose active power (current limits, LVPL, LVACM) and the HPPC sees a large P error. Without the freeze, the P PI wound up during the dip: P_poc overshot by 3–13 MW, and recovery took 2–4 s instead of 0.1–0.5 s (design.md step 1). The hold time `Thld` covers the assets' own P recovery ramp after clearing.

### B5.5 Set-point handling and ramp limit

The TSO set-point arrives 200 ms late (`comm.tso`). The case sets step it with `p.grid.Pstep` (time `t0`, change `dP`).

**Availability-limited target with ramp** (`ramp_limiter`):

```matlab
persistent y
if isempty(y), y = P_ref; end
u = min(P_ref, max(P_hi - reserve, 0));
if par.rampFlag
    y = y + min(max(u - y, -par.rampDn / 60 * Ts), par.rampUp / 60 * Ts);
else
    y = u;
end
P_base = y;
```

$$u = \min\big(P_{ref},\ \max(P_{hi} - \textit{reserve}, 0)\big),\qquad \textit{reserve} = \textit{Frqflag}\cdot\textit{FSMflag}\cdot \textit{fsmRange}$$
$$P_{base,k} = P_{base,k-1} + \mathrm{clip}\big(u - P_{base,k-1},\ -\tfrac{r_{dn}}{60}T_s,\ \tfrac{r_{up}}{60}T_s\big)$$

With `rampUp` = `rampDn` = 0.20 pu/min the step is $0.2/60 \cdot 0.1 = 3.3\cdot10^{-4}$ pu per sample, which is 0.6 MW/s = 36 MW/min on 180 MW. The state starts at the set-point, so there is no ramp at start-up.

*Why the target is limited by availability before the ramp.* The ramp then also applies to changes in availability: at the wind step at 20 s (0.6 → 0.9 pu available), the plant output ramps up at 36 MW/min instead of jumping. The BESS absorbs the renewable surplus meanwhile, and covers a fast drop of availability within its limits (design.md step 3).

*Why the reserve is subtracted.* With FSM active, the plant must be able to give `fsmRange` upwards. Wind and PV run at available power, so the reserve has to come from the BESS. Subtracting it from $P_{hi}$ keeps it unused in normal operation.

*Grid code check.* `controllerParameters` asserts $0.01 \le$ `rampUp`, `rampDn` $\le \min(0.20,\ 60\,\mathrm{MW}/P_n)$ = 0.20 (RfG: 1–20 % Pn/min, at most 60 MW/min).

### B5.6 Frequency response after the ramp

`frequency_response`:

```matlab
df = f_meas - par.Freq_ref;
dP = 0;
if par.FSMflag
    e = sign(df) * max(abs(df) - par.fsmDb, 0);
    dP = min(max(-e / par.sFSM, -par.fsmRange), par.fsmRange);
end
dP = dP - max(f_meas - par.fO, 0) / par.sO + max(par.fU - f_meas, 0) / par.sU;
dP_f = par.Frqflag * dP;
```

$$\Delta P_{FSM} = \mathrm{clip}\Big(-\frac{\mathrm{sign}(\Delta f)\max(|\Delta f| - f_{db},0)}{s_{FSM}},\ \pm \textit{fsmRange}\Big)$$
$$\Delta P_f = \textit{Frqflag}\Big(\textit{FSMflag}\cdot\Delta P_{FSM} - \frac{\max(f - f_O,0)}{s_O} + \frac{\max(f_U - f,0)}{s_U}\Big)$$

All in pu of Pn per pu of frequency, so a droop $s$ = 5 % means a gain of 20.

| Function | Parameter | Value |
|---|---|---|
| LFSM-O (always on) | `fO`, `sO` | 50.2/50 pu, 0.05 |
| LFSM-U (always on) | `fU`, `sU` | 49.8/50 pu, 0.05 |
| FSM (only when delivering FCR) | `FSMflag`, `sFSM`, `fsmDb`, `fsmRange` | 0, 0.05, 0.010/50 pu (10 mHz), 0.10 pu |
| master switch | `Frqflag` | 1 (0 in the decentral cases) |
| reference | `Freq_ref` | 1.0 |

Then:

$$P_{target} = \min(P_{base} + \Delta P_f,\ P_{poc,max}),\qquad P_{poc,max} = 180/180 = 1.0 \text{ pu}$$

**Why after the ramp limiter.** Energinet's priority order puts frequency control before the power constraint and the power gradient constraint (RfG 14(5)(c), TR 3.3.1 §26). If the response went through the ramp limiter, a 21.6 MW LFSM response would take 36 s instead of 0.3 s. The export limit comes last, because it is a physical limit at the POC.

**Why LFSM is separate from FSM.** Step 1 used a symmetric droop with ±20 mHz deadband. That is effectively FSM, which is only required when the plant delivers FCR. Step 3 replaced it with LFSM-O and LFSM-U, always on, and an optional FSM. The droop ranges (2–12 %) are asserted in `controllerParameters`.

**Where the upward response comes from.** Wind and PV run at available power, so LFSM-U and FSM upwards come from the BESS: $P_{hi}$ includes its discharge limit, and the dispatch moves the BESS first. LFSM-U is not limited in the function itself. It is limited later by $P_{hi}$ in the P loop's output saturation and by the export limit.

### B5.7 P control at the POC (`Pflag`)

$$e_P = \mathrm{lag}(P_{target}, T_{ref}) - P_{filt}$$
$$e_P \leftarrow \textit{Pflag}\ ?\ e_P : 0, \qquad e_P \leftarrow \textit{frz}\ ?\ 0 : e_P, \qquad e_P \leftarrow \mathrm{sat}(e_P;\ f_{emin}, f_{emax})$$
$$u_{PI} = \mathrm{PI}(e_P;\ K_{pg}, K_{ig})\ \text{limited to}\ [P_{lo} - P_{target},\ P_{hi} - P_{target}]$$
$$P_{ord} = \mathrm{sat}(P_{target} + u_{PI};\ P_{lo}, P_{hi})$$

| Parameter | Value | Meaning |
|---|---|---|
| `Pflag` | 1 | closed loop; 0 = open loop (P_ord = P_target) |
| `Tp` | 0.2 s | P_poc filter |
| `Tref` | 0.4 s | reference filter on the target in the PI error |
| `Kpg`, `Kig` | 0.1, 0.5 /s | PI |
| `femax`, `femin` | 0.2, −0.2 | error limits |
| `Vfrz`, `Thld` | 0.9 pu, 1.0 s | freeze |

**Why feed-forward plus PI.** The target goes straight to the plants, so a set-point change takes effect at once. The PI corrects only what the plants do not deliver: losses in the collector and transformers, and asset deviations. Open loop, P_poc was 146.52 MW for P_ref 150 MW. With the loop it is 150.00 MW (design.md step 1).

**Why the reference filter `Tref`.** Between a set-point step and the measured response there are delays (50 ms to the plants, 20 ms back), plant and asset lags, and the measurement filter. Without `Tref`, the PI integrates this response time as if it were an error, and overshoots after the plant arrives. `Tref` = 0.4 s models "the target as the plant can follow it". With it, the +20 MW step overshoots 0.8 MW instead of 3 MW (design.md step 1).

**Anti-windup.** The PI block's clamping uses the external limits, which are the plant capability minus the feed-forward.

### B5.8 Dispatch to the plants

`dispatch`:

```matlab
P_ren = P_avail_wpp + P_avail_spp;
P_set_bess = min(max(P_ord - P_ren, -BESS_chg), BESS_dis);
P_r = min(max(P_ord - P_set_bess, 0), P_ren);
w = P_avail_wpp / P_ren          % (rating share if P_ren is 0)
P_set_wpp = P_r * w;   P_set_spp = P_r * (1 - w);
Q_set_wpp = Q_ord * qshare(1);  Q_set_spp = Q_ord * qshare(2);  Q_set_bess = Q_ord * qshare(3);
```

$$P_{set,bess} = \mathrm{clip}(P_{ord} - P_{ren},\ -BESS_{chg},\ BESS_{dis}),\qquad P_r = \mathrm{clip}(P_{ord} - P_{set,bess},\ 0,\ P_{ren})$$

Wind and PV are set to their available power, and the BESS takes the difference. Only when the BESS cannot absorb more (at its charge limit, or derated near SOCmax) are wind and PV curtailed, in proportion to their availability.

*Why.* Renewable energy is free and the BESS is the only flexible asset, so curtailment is the last resort. Using the BESS first also gives the fastest response, since it has no wind or irradiance limit (inferred; the rule itself is recorded in design.md step 1).

**Q split by rating.** `qshare` = `Prated / sum(Prated)` = [150, 50, 30]/230 = [0.652, 0.217, 0.130]. After the conversion to each plant's base, every plant gets the same per-unit offset ($Q_{ord}\cdot180/230$), which matches the equal per-unit droop of B4.6.

*Alternative (open point).* Q dispatch by available Q capability (U-Q/Pmax profile) instead of rating (design.md section 10).

*Known side effect.* At the wind step (20 s), P_poc dips by about 13 MW for 0.3 s. The HPPC moves the BESS at once on the jump in wind *availability*, while the wind *output* follows only within about 0.5 s. Dispatching the BESS on the measured wind output (`P_wpp`, already an input) would avoid it (design.md section 10).

### B5.9 Q control and PF control (`RefFlag` 0 and 2)

The Q reference:

$$Q_{refx} = \textit{isPF}\ ?\ \tan\varphi \cdot P_{filt} : Q_{ref}, \qquad \tan\varphi = \textit{PFsign}\sqrt{1/PF^2 - 1}$$

`tanphi`, `isPF` and `isV` are computed in `controllerParameters`. Block parameters cannot use `sqrt`, `^` or comparisons and still stay tunable in compiled code.

The loop:

$$e_Q = \mathrm{lag}(Q_{refx}, T_{refQ}) - Q_{filt}$$
$$\textit{ff} = \textit{isV}\ ?\ 0 : Q_{refx}$$
$$e \leftarrow \textit{Qflag}\ ?\ e : 0,\quad e \leftarrow \textit{frz}\ ?\ 0 : e,\quad e \leftarrow \mathrm{sat}(e;\ e_{qmin}, e_{qmax}),\quad e \leftarrow \text{plant anti-windup}(e)$$
$$Q_{ord} = \mathrm{sat}\big(\textit{ff} + \mathrm{PI}(e;\ K_{pQ}, K_{iQ}),\ Q_{minOrd}, Q_{maxOrd}\big)$$

The PI output is limited to $[Q_{minOrd} - \textit{ff},\ Q_{maxOrd} - \textit{ff}]$ with clamping. In Q and PF mode $K_{pQ}$ = `Kpq` = 0.1, $K_{iQ}$ = `Kiq` = 0.2 /s, and $Q_{maxOrd}$ = `Qmax` = 0.33.

| Parameter | Value | Meaning |
|---|---|---|
| `Qflag` | 1 | closed loop; 0 = Q_ref split open loop |
| `RefFlag` | 0 | 0 Q, 1 voltage, 2 power factor |
| `Tq` | 0.2 s | Q and V filter |
| `TrefQ` | 0.4 s | reference filter |
| `Kpq`, `Kiq` | 0.1, 0.2 /s | Q / PF PI (about 5.5 s) |
| `PF`, `PFsign` | 1.0, +1 | power factor set-point; +1 export with P, −1 import |
| `eqmax`, `eqmin` | 0.2, −0.2 | error limits (pu Q or pu V) |
| `Qmax`, `Qmin` | 0.33, −0.33 | POC Q limits (≈ 60 Mvar, PF 0.95 at 180 MW) |

*Why `Kiq` 0.2 (was 0.5).* The cascade rule (B5.13). With 0.5 the HPPC Q loop (2.2 s) was less than three times slower than the plant Q loop (1.1 s).

*Overshoot.* The ±30 Mvar Q steps overshoot by 6–10 Mvar. Most of it comes from the plant and asset Q loops (feed-forward plus PI at two levels): the same step with the HPPC loop open overshoots 4.7 Mvar. A longer `TrefQ` (0.8 s) or a smaller `Kiq` hardly changed it (design.md step 2).

*PF control.* The Q reference follows the filtered P, also while P ramps after the wind step.

### B5.10 Voltage control (`RefFlag` 1): slope and cascade

**The slope.** The voltage error contains the slope:

$$e_V = V_{ref} - V_{filt} - \frac{V_{slope}}{Q_{max}}\,Q_{filt}$$

In steady state $e_V$ = 0, so

$$Q = \frac{V_{ref} - V}{V_{slope}}\,Q_{max}.$$

`Vslope` = 0.04 means that V drops by 4 % for a Q change of Qmax = 0.33 pu. Example: V_poc 0.9917 → Q = 0.0083/0.04 × 0.33 = 0.068 pu = 12.3 Mvar (design.md step 3). Then the POC Q limit (B5.11) caps $e_V$. There is no feed-forward in voltage mode.

**Two variants, chosen by `Vlocal`:**

| | `Vlocal` = 1 (default): cascade | `Vlocal` = 0: direct |
|---|---|---|
| who controls the voltage fast | the plant controllers, at the 33 kV busbar, on a droop line (B4.6) | the HPPC, at the POC |
| HPPC role | slow trim of the plants' Q offsets so V and Q at the POC lie on the slope | PI on the POC voltage error |
| HPPC gains | `KpVt` = 0, `KiVt` = 0.3 (normalised) | `KpV` = 0.5, `KiV` = 2.5 (normalised) |
| loop gain `LV` | $(K_{c,POC} + 1/SCR)\cdot \textit{auth}$ | $K_{c,POC} + 1/SCR$ |
| order limits | $\pm Q_{max}/\textit{auth}$ | $\pm Q_{max}$ |
| plant `RefFlag` | 1 (set by `controllerParameters`) | 0 (Q tracking) |

Here $\textit{auth} = K_{c,Bus}/(K_{c,Bus} + x_{Bus})$, the share of an offset change that becomes real Q. The actual PI gains are $K_{pQ} = K_{pVn}/LV$, $K_{iQ} = K_{iVn}/LV$.

**Why the loop gain is $K_{c,POC} + 1/SCR$.** A change $\Delta Q$ at the POC changes $e_V$ by $-(V_{slope}/Q_{max})\Delta Q$ directly, and by $-\Delta V_{POC} \approx -\Delta Q/SCR$ through the grid ($dV/dQ \approx 1/SCR$ on the POC base). Dividing the gains by this sum makes the speed nearly independent of the slope and the grid. Before step 3, fixed gains were a compromise: `KiV` 5 took 6 s at SCR 10, and `KiV` 20 oscillated at SCR 3 (10 Mvar overshoot, 3.6 s) (design.md step 2).

**Why "authority".** In the cascade, the HPPC output is the offset of the plants' droop line. If the HPPC raises the offset by $\Delta Q_{set}$, the plants raise Q, the busbar voltage rises by $x_{Bus}\Delta Q$, and the droop takes back part of it. In steady state:

$$\Delta Q = \frac{K_{c,Bus}}{K_{c,Bus} + x_{Bus}}\,\Delta Q_{set}$$

At SCR 10 this is 0.0348/(0.0348 + 0.1864) = 0.157 (design.md: "0.16 at SCR 10"). So the HPPC needs a much wider order range to reach Qmax: $Q_{maxOrd}$ = 0.33/0.157 = 2.1 pu at SCR 10 (4.3 pu at SCR 3). The real Q is limited by the plants (±0.5 pu on their ratings) and by the POC Q limit.

Numerical values (4 % slope, defaults):

| | SCR 10 | SCR 3 |
|---|---|---|
| $K_{c,POC}$ | 0.1212 | 0.1212 |
| $K_{c,Bus}$ | 0.0348 | 0.0348 |
| auth | 0.157 | 0.077 |
| `LV` (cascade) | 0.0348 | 0.0349 |
| $K_{iQ}$ = 0.3/LV (cascade) | 8.6 /s | 8.6 /s |
| `QmaxOrd` (cascade) | 2.10 | 4.30 |
| `LV` (direct) | 0.221 | 0.454 |
| $K_{pQ}$, $K_{iQ}$ (direct) | 2.26, 11.3 /s | 1.10, 5.5 /s |

(computed from the formulas in `controllerParameters.m`)

**Why a cascade at all.** In practice voltage is controlled in layers: each plant controller regulates its own point fast with a droop, and a master controller slowly corrects the set-points so the requirement holds at the POC. This is the same as primary and secondary voltage control in transmission grids. The requirement itself (slope, t1, t2, Q range) is defined at the POC, so the slow part stays central. It also fits the research setting: vendor plant controllers that close their own loops under a coordinating HPPC (design.md step 4).

**What the direct variant got wrong.** With `Vlocal` = 0 the HPPC voltage loop (about 0.6 s) is *faster* than the plant Q loops (1.1 s). That breaks the cascade rule, and `controllerParameters` warns. It was also sensitive to a wrongly set SCR: with `SCRset` 10 on an SCR 3 grid, 50–90 % overshoot; with the cascade, 19 % (design.md steps 3 and 4).

**`SCRset`.** dV/dQ is a commissioning setting: `SCRset` = NaN uses the actual `p.grid.SCR`. In reality the SCR at the POC is known only approximately and changes with the grid topology. An online estimate of dV/dQ is an open point.

### B5.11 POC Q limit (block "POC Q limit")

```matlab
k = par.Vslope / par.Qmax + par.dVdQ;
V_errL = min(max(V_err, (par.Qmin - Q_filt) * k), (par.Qmax - Q_filt) * k);
```

$$e_V \leftarrow \mathrm{clip}\big(e_V,\ (Q_{min} - Q_{filt})k,\ (Q_{max} - Q_{filt})k\big),\qquad k = V_{slope}/Q_{max} + 1/SCR_{set}$$

**What it does.** Where the slope asks for more Q than `Qmax` (or less than `Qmin`), the voltage error is replaced by the error that the remaining Q margin would cause at the loop gain $k$. At $Q_{filt} = Q_{max}$ the upper bound is 0, so the integrator stops. Above $Q_{max}$ it becomes negative and pulls Q back. The loop settles exactly at the limit.

**Why it is needed.** In the cascade, the HPPC's order is only an offset with a wide range (±2.1 pu), so the order limit no longer limits Q at the POC. The plants' droop alone would give whatever the voltage asks for, up to their own limits. The plant limits cannot be used either, because they act at the asset terminals and the wind losses make them inaccurate (B4.6). So the HPPC limits the cause, the voltage error, with a value that maps to the Q margin.

**Why not saturate Q_ord instead.** Because of the authority factor, the order needed for a given Q depends on the grid and the droop. A fixed order limit would be either too tight or too loose (inferred from the authority calculation above).

**Result.** Grid −10 % / +8 %: Q_poc settles at +59.40 / −59.40 Mvar. The plants' fast droop overshoots to +69 / −72 Mvar first, and the slow trim (about 3 s time constant, but limited by the capped error) brings Q back within 0.3 Mvar of the limit after about 20 s (design.md step 4).

### B5.12 Anti-windup over the plant limits (`QawFlag`)

```matlab
up = Q_sat_wpp > 0.5 && Q_sat_spp > 0.5 && Q_sat_bess > 0.5;
dn = Q_sat_wpp < -0.5 && Q_sat_spp < -0.5 && Q_sat_bess < -0.5;
e_aw = e;
if par.QawFlag && ((e > 0 && up) || (e < 0 && dn)), e_aw = 0; end
```

**What.** Conditional integration: if every plant reports its Q order at the upper limit and the error asks for more, the error is set to 0 (and likewise downwards). It sits after the error limits and before the PI, so it acts in all three modes.

**Why "all plants".** The HPPC splits one order over all plants. As long as one plant can still move, more order still changes Q, so holding would be wrong.

**Why it is needed in addition to the PI's clamping.** The PI clamps only at its own output limits. In the cascade these are ±2.1 pu, far beyond what the plants can deliver, so the PI would wind up long before it reached them.

**Result** (plant limits reduced to 0.2 pu, grid −10 % from 40 to 60 s): with anti-windup Q is back within 3 Mvar 0.9 s after the voltage returns. Without it Q stays at +15.8 Mvar (V_poc 1.013) for 9.5 s (design.md step 4).

| Parameter | Value | Meaning |
|---|---|---|
| `QawFlag` | 1 | anti-windup on |

### B5.13 Cascade timing rule (`cascadeSep`)

**Rule.** Every control level must be at least `cascadeSep` = 3 times slower than the level below it. The speed of a PI loop is estimated as $\tau \approx (1+K_p)/K_i$ with the gains normalised by the loop gain (Q loops: loop gain 1). `controllerParameters` computes this and stores it in `p.ctrl.hppc.cascade`. It warns (`controllerParameters:cascade`) if $\tau_{plant} < 3\tau_{asset}$ or $\tau_{HPPC} < 3\tau_{plant}$.

| Level | Loop | Gains | τ |
|---|---|---|---|
| asset | WT `kpq`/`kiq`, PV and BESS `Kqp`/`Kqi` | 0.1, 10 /s (was 5) | 0.11 s |
| plant, Q control | `Kp`/`Ki` | 0.1, 1.0 | 1.1 s |
| plant, voltage control (cascade) | `KpV`/`KiV` | 0.5, 2.5 | 0.6 s |
| HPPC Q / PF | `Kpq`/`Kiq` | 0.1, 0.2 (was 0.5) | 5.5 s |
| HPPC voltage trim (cascade) | `KpVt`/`KiVt` | 0, 0.3 | 3.3 s |
| HPPC direct voltage (`Vlocal` 0) | `KpV`/`KiV` | 0.5, 2.5 | 0.6 s (warns) |

**Why.** Nested loops with similar speeds interact: the outer loop reacts to a transient that the inner loop is about to correct, and the two overshoot or oscillate. A separation of 3–5 times is a common rule of thumb for cascades (inferred; the value 3 is a parameter). Step 4 raised the asset Q gains from 5 to 10 /s and lowered the HPPC Q gain from 0.5 to 0.2 /s to satisfy the rule (commit 309edbb).

**Trade-off recorded.** Slower plant voltage gains reduce the overshoot (halved gains: 2–12 %, a quarter: 0–1 %), but t90 then reaches 1.23 / 2.75 s and misses t1 = 1 s. The final Q does not depend on the gains. The current gains are the compromise (design.md step 4).

### B5.14 Full HPPC parameter table

From `hppcParameters.m`, plus fields added by `controllerParameters.m`.

| Parameter | Value | Meaning |
|---|---|---|
| `Pflag` | 1 | P loop closed |
| `Tp` | 0.2 s | P_poc filter |
| `Tref` | 0.4 s | P reference filter |
| `Vfrz`, `Thld` | 0.9, 1.0 s | freeze in a dip and hold |
| `Kpg`, `Kig` | 0.1, 0.5 /s | P PI |
| `femax`, `femin` | ±0.2 | P error limits |
| `rampFlag` | 1 | ramp limit on |
| `rampUp`, `rampDn` | 0.20 pu/min | 36 MW/min |
| `Frqflag` | 1 | frequency response in the HPPC |
| `Freq_ref` | 1.0 | |
| `fO`, `sO` | 1.004 (50.2 Hz), 0.05 | LFSM-O |
| `fU`, `sU` | 0.996 (49.8 Hz), 0.05 | LFSM-U |
| `FSMflag`, `sFSM`, `fsmDb`, `fsmRange` | 0, 0.05, 0.0002 (10 mHz), 0.10 | FSM |
| `SOCband` | 0.1 | SOC derating band |
| `Qflag` | 1 | Q loop closed |
| `QawFlag` | 1 | anti-windup over plant limits |
| `RefFlag` | 0 | Q / V / PF |
| `Vref` | 1.0 | voltage set-point (0.95–1.05) |
| `Vslope` | 0.04 | slope (2–7 %) |
| `PF`, `PFsign` | 1.0, +1 | PF set-point |
| `Tq`, `TrefQ` | 0.2, 0.4 s | filters |
| `Kpq`, `Kiq` | 0.1, 0.2 /s | Q / PF PI |
| `Vlocal` | 1 | voltage cascade |
| `KpVt`, `KiVt` | 0, 0.3 /s | normalised trim gains |
| `KcBusMin` | 0.02 | minimum busbar droop [pu on 180 MVA] |
| `cascadeSep` | 3 | cascade separation |
| `KpV`, `KiV` | 0.5, 2.5 /s | normalised direct V gains |
| `SCRset` | NaN | commissioned SCR (NaN = actual) |
| `eqmax`, `eqmin` | ±0.2 | Q/V error limits |
| `Qmax`, `Qmin` | ±0.33 | POC Q limits |
| *derived:* `Ppoc_max` | 1.0 | export limit 180 MW |
| `qshare` | [0.652 0.217 0.130] | Q split by rating |
| `dVdQ` | 1/SCRset | grid dV/dQ at the POC |
| `SOCmin`, `SOCmax` | 0.1, 0.9 | from `p.bess` |
| `LV`, `KpVn`, `KiVn`, `QmaxOrd`, `QminOrd` | see B5.10 | mode-dependent |
| `isV`, `isPF`, `tanphi`, `KpQ`, `KiQ` | | derived for tunable block parameters |

### B5.15 The decentral alternative and the comparison

**Decentral setup** (`freq_step` cases "+0.2 Hz, decentral" and "−0.2 Hz, decentral"): `hppc.Pflag` = 0, `hppc.Frqflag` = 0, and `pc.<plant>.Frqflag` = 1 for all three plants. The HPPC is open loop in P (the ramp limit, export limit and dispatch still act), and each plant controller has its own REPC_A droop (5 %, ±20 mHz) and P loop on its own rating. The Q/V loops are the same as in central control.

**Results.**
- Step 1 (with the old symmetric droop in the HPPC): central ±0.2 Hz gave ∓12.96 MW (5 % × 0.18 Hz × 180 MW), symmetric, within 5 % after 0.3–0.7 s. Decentral gave −13.55 / +2.15 MW in 1.4–2.0 s: under-frequency only from the BESS on its own 30 MW base, and P_poc offset by the uncompensated losses (design.md step 1).
- Current report figure `freq_central_decentral` (+0.2 Hz with FSM, central against decentral): "about the same change at the POC; decentral reacts slightly earlier but overshoots and curtails wind, central uses the BESS and settles without overshoot" (`reportFigures.m`).

**Why central is the default.** It gives the response on the plant's Pn rather than on each plant's rating. It uses the BESS headroom for the whole plant, avoids curtailing wind for over-frequency (the BESS absorbs instead), and holds the POC set-point exactly. Decentral is kept because it is what vendor plant controllers do on their own. How a central HPPC should coordinate plant controllers that keep their own P and frequency loops is the open research question (design.md section 10).

### B5.16 Contradictions and notes

- `controllerParameters.m` comments `Ppoc_max` as "(not used yet)", but `HPPC.slx` uses it in the "export limit" block. The code wins.
- design.md section 7 lists `KcBusMin` and `KpVt`/`KiVt` with the HPPC, but section 8 step 2 still states the step-2 values (`Kiq` 0.5, `KpV` 2, `KiV` 10). These are history; current values are in `hppcParameters.m` (Kiq 0.2, KpV 0.5, KiV 2.5).
- `buildHPP.m` `controllerSpecs` still contains the stage-1 placeholder code for all controllers. It is only used for the model names and ports, while the real templates are `buildHppcCtrl` and `buildPlantCtrlRepc`.
- The `.slx` and the template agree in every block and function.

### Recreating it

1. Create `HPPC.slx` (fixed-step discrete, `Ts` = 0.1 s, `par` model argument) with the 20 inputs and 6 outputs of `modelInterfaces.m`.
2. Measurements: lags `Tp`, `Tq` (V initial 1), `soc_limits`, sum $P_{hi}$, $P_{lo} = -BESS_{chg}$, `dip_freeze` with `Ts` as a constant input.
3. `ramp_limiter` with the FSM reserve constant `Frqflag·FSMflag·fsmRange`; `frequency_response`; sum; MinMax (min) with `Ppoc_max`.
4. P loop: `Tref` lag, error, switch `Pflag`, switch freeze, error limits, PI (external limits $P_{hi} - P_{target}$, $P_{lo} - P_{target}$, clamping), target + PI, dynamic saturation.
5. Q path: gain `Vslope/Qmax`, V error, `poc_q_limit`; PF switch with `tanphi·P_filt`; feed-forward switch (`isV` → 0); `TrefQ`; Q error; switch `isV` to the V error; switch `Qflag`; freeze; error limits; `plant_anti_windup`; PI with `KpQ`/`KiQ` and limits `QmaxOrd − ff`, `QminOrd − ff`; ff + PI; saturation.
6. `dispatch` (BESS balances, curtailment by availability, Q by `qshare`).
7. In `controllerParameters`: `qshare`, `dVdQ`, `SOCmin/max`, ramp and droop asserts, the voltage cascade (`xTr`, `xBus`, `KcBus`, auth, `LV`, `QmaxOrd`), the plant droop fields, `isV`, `isPF`, `tanphi`, `KpQ`, `KiQ`, and the cascade check.
8. Verify with `hppc_p`, `freq_step`, `hppc_qv`, `wt_frt`, `bess_soc` against the results in B7.

---

## B6. Communication delays, controller rates, measurement memories and algebraic loops

This chapter covers the timing of the control hierarchy: the communication delay of each signal path, the controller sample rates, the one-step measurement memories and the algebraic-loop reason for them, the initial conditions of the delays, the fixed and variable delay implementations (the physical side; the run/compile side is chapter A5), and the measured effect of the delays.

Sources: `params/commParameters.m`, `params/deriveParameters.m`, `params/controllerParameters.m`, `params/simulationParameters.m`, `utilities/build/buildHPP.m` (`makeLink` ~line 1313, `buildDelayVariant` ~line 1370, `buildHppcSub`, `buildPlantSub`, `buildNetwork`, `buildAsset`, `buildController`), `scripts/delayMode.m`, `scripts/+caseSets/comm_delay.m`, `docs/design.md` sections 6, 8 (stage 1) and 11, commits `41a0b4f`, `ae5559d`, `fb180ee`, `ac18a90`.

### B6.1 Delay values per path

#### What

`commParameters` (all in seconds, *typical* values):

| Field | Path | Default | Applied in link block | Receiving rate |
|---|---|---|---|---|
| `comm.tso` | TSO set-points (P_ref, Q_ref) → HPPC | 0.200 | HPPC subsystem | 100 ms |
| `comm.meas_hppc` | POC measurements and plant measurements (P, Q, P_avail, P_chg_max, P_dis_max, SOC, Q_sat) → HPPC | 0.020 | HPPC subsystem | 100 ms |
| `comm.hppc_to_plant` | HPPC set-points → plant controllers | 0.050 | plant subsystem | 20 ms |
| `comm.meas_plant` | plant measurements (P, Q at the busbar feeder, V_bus) and asset signals (P_avail_k, SOC_k) → plant controller | 0.020 | plant subsystem | 20 ms |
| `comm.plant_to_asset` | plant controller set-points → asset controllers | 0.020 | plant subsystem | 1 ms |
| — | asset measurements → asset controller | 0 (local) | inside the asset model | 1 ms |
| — | grid frequency → HPPC | 0 (no link) | | 100 ms |
| — | grid frequency → plant controllers | 0 (memory only) | | 20 ms |
| `comm.maxDelay` | largest delay any case may use (buffer size of variable delays) | 0.5 | | |
| `comm.delays` | "fixed", "variable" or "auto" | "auto" | | |

`deriveParameters` turns every delay into whole simulation steps and checks the range:

```matlab
nMax = round(p.comm.maxDelay / p.sim.Ts);
for f = setdiff(fieldnames(p.comm), ["maxDelay", "steps", "delays"])
    n = round(p.comm.(f) / p.sim.Ts);
    assert(n >= 0 && n <= nMax, ...);
    p.comm.steps.(f) = n;
end
```

With 1 ms: tso 200 steps, meas_hppc 20, hppc_to_plant 50, meas_plant 20, plant_to_asset 20.

Each delay is applied once, in the subsystem of the receiving controller, as part of the link block of that signal path (B1.3 lists every link). The sending side converts only units.

Note: `docs/design.md` section 6 labels the 20 ms HPPC path "POC measurement → HPPC"; in the code `comm.meas_hppc` is used for both the POC and the plant measurements to the HPPC (`commParameters`: "POC and plant measurements -> HPPC"; `buildHppcSub` applies `meas` to all of them).

#### Why these values (reasoning not recorded in the repository; inferred)

The design document marks them "typical", to be replaced by project data, and makes them parameters so case sets can sweep them. Plausible reasoning:
- **TSO 200 ms**: set-points come from a remote control centre over a wide-area SCADA link (e.g. IEC 60870-5-104), with gateway processing at both ends; hundreds of milliseconds is typical.
- **Measurements 20 ms**: local substation meters and IEDs on a fast fibre network (e.g. IEC 61850), about one cycle of processing.
- **HPPC → plant 50 ms**: a set-point to a vendor plant controller passes a protocol gateway (e.g. Modbus TCP or OPC) into the vendor's system, slower than a direct measurement.
- **Plant → asset 20 ms**: the plant controller's broadcast over the plant's fieldbus or SCADA network to the turbines/inverters.
- **Local asset signals 0**: the converter control measures its own terminal directly.

#### Consequences

- A delay must be a multiple of `sim.Ts` (1 ms resolution); values are rounded.
- A delay above `comm.maxDelay` (0.5 s) fails the assertion; raise `maxDelay` for longer delays (compile-time value: larger buffer, rebuild of the Rapid target).
- Delays are tunable in variable mode only (B6.5).

### B6.2 Discrete whole-step delays instead of transport delays

#### What

Each delay is a Simulink **Delay** block (discrete, sample time `p.sim.Ts`) whose length is `p.comm.steps.<path>` steps, inside a Variant Subsystem with a Fixed and a Variable choice (B6.5).

#### Why

- **Speed.** Stage 1 (commit `41a0b4f`) used continuous Transport Delay blocks. Commit `ae5559d` replaced them with discrete whole-step delays as part of the run-time work (with `ode1` and logging every 10 ms): about 90 s instead of 105 s per run at the time, steady state unchanged, dip values within 0.7 MW. A Transport Delay keeps a time-stamped buffer and interpolates at every minor and major step; a discrete Delay is a ring buffer of samples (inferred explanation).
- **Exactness.** All delays of interest are multiples of 1 ms, so the discrete delay reproduces them exactly.
- **Code generation and PSCAD.** In PSCAD and in a real controller, communication is sampled; a whole-step delay is closer to that than a continuous one (inferred).

#### Consequences

The delay resolution is the simulation step. With a larger `sim.Ts`, the delays are rounded more coarsely.

### B6.3 Controller sample rates

#### What

| Controller | `Ts` | Set in |
|---|---|---|
| HPPC | 0.1 s | `controllerParameters`, `ctrl.hppc.Ts = 0.1` |
| WPPC, SPPC, BESSC | 0.02 s | `plantCtrl`, `c.Ts = 0.02` |
| WT, PV, BESS asset controllers | `p.sim.Ts` (1 ms) | `assetCtrl(p.sim.Ts, ...)` |

Each controller model reads its `Ts` from its model workspace (`Ts = pDefault.ctrl.<c>.Ts`, from `HPP_parameters()`), uses solver `FixedStepDiscrete` with `FixedStep = Ts`, and its inports have `SampleTime = Ts` (`buildController` / `ctrlHeader` in `buildHPP.m`; confirmed in the model workspaces of `models/ctrl/HPPC.slx`, `WPPC.slx`, `WT_ctrl.slx`). `controllerParameters` asserts that each `Ts` is an integer multiple of `sim.Ts`.

The link blocks run at 1 ms; the receiving controller samples the delayed signal at its own rate ("The receiving controller samples it at its own rate", `docs/design.md` section 6). A controller output is held between its samples. The top model is single-tasking, so these rate transitions need no Rate Transition blocks (inferred, B1.5).

#### Why

- **Realism.** A plant-level controller (HPPC) typically runs at 100 ms; vendor plant controllers faster (tens of ms); converter control effectively continuous relative to the plant (here at the simulation step). These are typical industrial values (inferred; not referenced in the repository).
- **Grid code time scale.** The requirements at the POC are in seconds (voltage control t90 ≤ 1 s, LFSM start ≤ 2 s); 100 ms gives 10 samples per second.
- **Cascade separation.** Each level must be slower than the one below. `controllerParameters` estimates each loop's response time as (1 + Kp)/Ki and warns if a level is not at least `p.hppc.cascadeSep` (3) times slower than the one below (`p.ctrl.hppc.cascade`); chapter B5. The sample rates follow the same ordering.
- **PSCAD analogy.** "The controller sample time must be an integer multiple of sim.Ts. This mirrors PSCAD, where the controller DLL runs at a multiple of the EMT time step" (`simulationParameters`).

#### Consequences

- `Ts` is not part of `par`; it is a fixed setting of each controller (and of its DLL). Changing it means changing `controllerParameters` and recompiling; the HPPC gains in discrete form depend on `Ts`.
- Because the HPPC samples every 100 ms, a one-step (1 ms) delay before it can postpone its reaction by a whole HPPC period when an event falls exactly on a sample instant (B6.4).

### B6.4 One-step measurement memories and algebraic loops

#### What

Every measurement path into a controller contains a **Memory** block (a one-step delay of 1 ms, initial output 0 by default) in its link, before the communication delay (gain → memory → delay): POC P, Q, V and all plant measurements into the HPPC; plant P, Q, V_bus, P_avail_k, SOC_k into the plant controllers; and the frequency into the plant controllers (memory only). Set-point paths (TSO → HPPC, HPPC → plant, plant → asset) have no memory. Inside the asset model, P and Q pass through Memory blocks (`P meas`, `Q meas`) and the terminal voltage is already one step old from the network (B2.3).

The top model sets `AlgebraicLoopMsg = error`.

#### Why

`docs/design.md` section 6: "Every measurement path into a controller also has a one-step memory (1 ms): without it, the loops through the controllers' direct feedthrough are algebraic loops when a delay is 0."

An *algebraic loop* is a closed signal path where every block has direct feedthrough (its output at time t depends on its input at time t), so Simulink must solve an implicit equation at every step. The controllers are MATLAB Function blocks or block diagrams whose outputs depend directly on their inputs (a PI's proportional part, a feed-forward of the set-point), and the network is direct feedthrough. A discrete Delay with length 0 (as in the "no delays" cases) is also direct feedthrough. With all delays at 0, a loop like HPPC → plant controller → asset controller → converter → network → POC measurement → HPPC could become algebraic. A Memory block has no direct feedthrough, so it breaks every such loop independently of the delay values and of what the controllers contain.

Making algebraic loops an error (rather than letting Simulink iterate) keeps the fixed-step simulation fast and deterministic, and is required for code generation (inferred).

Note (inferred, from reading the code): today the converter current lags (`contLag`: an Integrator) also have no direct feedthrough, so the control path through the converter is already broken by the lag; the memories still guarantee loop freedom for any controller or converter model, including the stage-1 placeholder generator, which had no lag. They also represent the fact that a sampled measurement is never available in the same instant.

#### Measured effect

Stage 1 (`docs/design.md` section 8, commit `41a0b4f`): with delays set to 0, results matched the earlier single-controller model "except that the HPPC reacts one sample (0.1 s) later at start-up and at the 20 s wind step (one-step measurement memories)". Mechanism (inferred): an event at exactly t = 20.000 s reaches the HPPC input at 20.001 s because of the memory, just after the HPPC's sample at 20.000 s, so the HPPC sees it at its next sample, 20.1 s.

#### Consequences

- When adding a measurement path, use a link with `memory` (the `meas` step list in `buildHppcSub`, or the pattern in `buildPlantSub`); a missing memory can create an algebraic loop when the delays are 0, which stops the build with an error.
- The memory adds 1 ms to every measurement path on top of the communication delay.

### B6.5 Initial conditions of the delays

#### What

`makeLink` accepts `delay:p.comm.<name>@<initial output>`:

| Link | Initial output of the delay |
|---|---|
| TSO P_ref → HPPC | `p.grid.Pref / p.ctrl.hppc.Pbase` (the initial set-point in pu) |
| TSO Q_ref → HPPC | `p.grid.Qref / p.ctrl.hppc.Pbase` |
| POC V → HPPC | 1 |
| Busbar V → plant controller | 1 |
| all others | 0 |

Memory blocks keep the default initial output 0; the network's voltage memories start at 1∠0 (flat start).

#### Why

- **TSO delay.** "The TSO delays start at the initial set-point, so the HPPC does not see a zero reference at start-up" (`docs/design.md` section 6). With a 200 ms delay starting at 0, the HPPC would receive P_ref = 0 for the first 200 ms; its ramp limiter (whose state starts at the set-point) and PI would move towards 0 and then back.
- **Voltages at 1** (inferred): a voltage of 0 would look like a deep dip to the controllers, triggering the voltage-dip freeze (`Vfrz`, PI freeze below 0.9 pu) and voltage-control action at start-up.

#### A possible start-up glitch (observation from reading the code, not verified in simulation)

In the voltage links the Memory comes before the Delay (memory → delay@1). The Delay outputs its initial value 1 for its first `steps` samples, and then the Memory's own initial output 0 (from t = 0) comes out for one step. For the busbar voltage into the plant controllers with `meas_plant` = 20 ms, that zero arrives exactly at t = 20 ms, which is a sample instant of the 20 ms plant controllers, so they would read V_meas = 0 once at start-up. Giving the Memory blocks in voltage links an initial condition of 1 (or putting the delay first) would remove this. It falls in the start-up transient that case sets avoid, and is listed here only for completeness.

### B6.6 Fixed and variable delays (physical side)

#### What

`buildDelayVariant` makes each link delay a Variant Subsystem with two choices, selected by `HPP_delays` (activation at update diagram, only the active choice is compiled):

| Choice | Variant control | Implementation |
|---|---|---|
| `Fixed` | `HPP_delays == 1` | Delay block, `DelayLength = p.comm.steps.<path>`, initial condition, sample time `p.sim.Ts` |
| `Variable` | `HPP_delays == 2` | Delay block with `DelayLengthSource = Input port`, `DelayLengthUpperLimit = round(p.comm.maxDelay/p.sim.Ts)` (500), length from a Constant `p.comm.steps.<path>` (tunable), same initial condition and sample time |

`delayMode(p, cases, override)` resolves `p.comm.delays`: "auto" becomes "variable" when the case table has a `comm.*` column (other than `comm.maxDelay` / `comm.delays`), otherwise "fixed"; returns the code 1 or 2.

#### Why

- **Fixed**: the delay length is a block parameter, fixed when the model compiles; fastest (Rapid Accelerator about 1.5–1.7 s per 60 s case). But a case with other delays needs a recompile / Rapid rebuild.
- **Variable**: the length is a tunable signal, so a delay sweep runs on one compiled model (Fast Restart or Rapid Accelerator), up to the buffer size `maxDelay`. Cost: about 0.6–1 s more per 60 s run in Rapid Accelerator (2.2–2.6 s instead of 1.5–1.7 s).
- Results are identical between the two (commit `ac18a90`: "results equal").

#### Alternatives / tried and dropped

Variable-length delays were first tried and dropped in normal mode (commit `fb180ee`): each made every run 0.5–0.75 s per simulated second slower. They came back (2026-11-19, `ac18a90`) as a choice next to the fixed delays, for delay sweeps, once Rapid Accelerator made their cost small. The run-side handling (compile signatures, `runSweep` refusing Fast Restart delay sweeps with fixed delays, Rapid fingerprint) is in chapter A5.

### B6.7 Measured effect of the delays: the `comm_delay` case set

#### What

`scripts/+caseSets/comm_delay.m`: a +30 Mvar Q_ref step at 40 s (`grid.Qstep.dQ = 30e6`, Q control at the POC, the default) with six delay settings:

| Case | Changes |
|---|---|
| HPPC-plant 50 ms (default) | none |
| HPPC-plant 0 ms | `comm.hppc_to_plant` 0 |
| HPPC-plant 100 ms | `comm.hppc_to_plant` 0.100 |
| HPPC-plant 200 ms | `comm.hppc_to_plant` 0.200 |
| all delays x2 | tso 0.400, meas_hppc 0.040, hppc_to_plant 0.100, meas_plant 0.040, plant_to_asset 0.040 |
| no delays | all five 0 |

The set changes `comm.*` values, so "auto" compiles variable delays and all six cases run on one compiled model: 15 s in Rapid Accelerator against 43 s in normal mode with Fast Restart, identical results (`docs/design.md` section 13).

#### Results

The time for Q_poc to reach 50 % of the step was given to me as 0.03 s without delays and 0.57 s with all delays doubled. I could not find these numbers in `docs/design.md`, the README or the git log, and the saved results (`results/comm_delay/sweep.mat`) hold MATLAB objects that cannot be read without MATLAB. They should be checked against the results before the handbook is final.

They are consistent with the dead time along the forward path (inferred arithmetic): a Q_ref step passes `comm.tso`, waits for the next HPPC sample (up to 100 ms), passes `comm.hppc_to_plant`, the plant sample (up to 20 ms) and `comm.plant_to_asset`, and the set-point is fed forward at each level (chapters B4, B5), so the response starts after roughly the sum of these. With the defaults that is 200 + 50 + 20 = 270 ms plus sampling; doubled, 400 + 100 + 40 = 540 ms plus sampling, close to the 0.57 s 50 % time. Without delays, the feed-forward reaches the converters within one HPPC and plant sample, and the 50 % point is set by the converter and asset Q dynamics. The TSO delay therefore dominates the response time to a TSO set-point, and the HPPC–plant delay matters most for the closed loops at the POC (measurement, HPPC, plant), which see `meas_hppc` + `hppc_to_plant` + sampling in every loop pass.

#### Consequences

- For grid code compliance checks with tight times (Q / voltage t90 ≤ 1 s), the delays use a significant part of the allowed time; the default HPPC and plant gains were tuned with the default delays in place.
- Higher delays reduce the phase margin of the HPPC's closed loops; the `comm_delay` set is the place to test robustness (inferred).

### Recreating it

1. Write `commParameters` with one delay per path (tso, meas_hppc, hppc_to_plant, meas_plant, plant_to_asset), `maxDelay` and the `delays` mode.
2. In `deriveParameters`, convert each delay to whole steps (`round(delay/sim.Ts)`), assert 0 ≤ n ≤ maxDelay/Ts, store in `p.comm.steps`.
3. Write `makeLink`: a masked subsystem with steps in series — `gain:<expr>`, `memory`, `delay:p.comm.<name>@<init>` — and an icon listing them.
4. Write `buildDelayVariant`: Variant Subsystem with a Fixed Delay (length parameter) and a Variable Delay (length input from a tunable Constant, upper limit maxDelay/Ts), both at `p.sim.Ts` with the given initial condition, controlled by `HPP_delays`, activation at update diagram.
5. Measurement paths into controllers: gain → memory → delay; set-point paths: delay → gain (sender side: unit gain only). TSO delays start at the initial set-point, voltage delays at 1.
6. Controllers: HPPC 0.1 s, plant 20 ms, assets at the simulation step; inports at `Ts`; assert each `Ts` is a multiple of `sim.Ts`; single-tasking top model.
7. Set `AlgebraicLoopMsg = error` and check that the model compiles with all delays at 0.
8. Write `delayMode` (auto → variable when a case set has `comm.*` columns) and a delay sweep case set (`comm_delay`).

## B7. Grid code (Energinet DK1): requirements, verification and open points

### B7.1 Sources and scope

**What.** The HPPC functions follow Energinet's requirements for DK1 (decided after review on 2026-11-16, design.md section 9; implemented in stage 5, step 3, commit 7459c87):

- the Danish RfG requirements (*Krav jf. RfG, bilag 1*) for **power park modules type D**. The copy used is rev. 2 of 22-07-2022, marked historic, so it must be checked against the current revision (design.md step 3; open point);
- **Technical Regulation 3.3.1** for energy storage facilities (rev. 6), for the BESS.

Pn = 180 MW is the POC base. Compliance is checked **in simulation only**. The type-D category follows from the 132 kV connection (inferred: RfG type D applies to connection points at 110 kV and above).

**Why one set of rules at the POC.** The plant is one hybrid plant at one POC. The requirements apply there, which is why they are implemented in the HPPC, the only controller that measures at the POC.

### B7.2 Requirements and how each is met

| Requirement (DK1) | Source (as recorded) | Implementation | Verified result |
|---|---|---|---|
| Ramp rate constraint up and down, 1–20 % Pn/min, at most 60 MW/min (PPM) or 1 MW/s (storage); maximum by default; not for frequency response | RfG 15(6)(e); TR 3.3.1 §32 | `ramp_limiter`, `rampUp`/`rampDn` 0.20 pu/min = 36 MW/min, also on availability changes; asserted 1–20 %, ≤ 60 MW/min | `hppc_p`: −30 / +20 / +40 MW reached after 48 / 31 / 47 s (1 % band); largest 60 s change 36.46 MW (limit 36), at 10 %/min 18.04 MW (limit 18) |
| Priority: protection, synthetic inertia, frequency control, power constraint, power gradient constraint | RfG 14(5)(c); TR 3.3.1 §26 | frequency response after the ramp limiter, before the export limit | LFSM response in 0.3 s, not ramp-limited |
| LFSM-O: threshold 50.2–50.5 Hz (DK1 50.2), droop 2–12 % (DK1 5 %), start ≤ 2 s, full ≤ 30 s, prevails over other set-points | RfG 13(2); TR 3.3.1 §23 | `fO` 50.2 Hz, `sO` 5 %, always on | +0.2 Hz: none; +0.5 Hz: −21.60 MW (12 % Pn); start 0.1 s, 90 % after 0.3 s |
| LFSM-U: threshold 49.5–49.8 Hz (DK1 49.8), droop 2–12 % (DK1 5 %), up to maximum capacity | RfG 15(2)(c); TR 3.3.1 §52 | `fU` 49.8 Hz, `sU` 5 %, always on; headroom from the BESS | −0.5 Hz: +21.60 MW |
| FSM: droop 2–12 %, deadband 0–500 mHz, insensitivity ≤ 10 mHz, range ≥ 1.5–10 % Pn, full activation ≤ 30 s | RfG 15(2)(d); TR 3.3.1 §53 | `FSMflag` (off unless delivering FCR), `sFSM` 5 %, `fsmDb` 10 mHz, `fsmRange` 10 %; BESS keeps the range as upward reserve | ±0.2 Hz: ∓13.68 MW (7.6 % Pn); +0.5 Hz FSM + LFSM-O: −39.6 MW (10 % + 12 %; BESS at its charge limit, wind curtailed) |
| Voltage control: set-point 0.95–1.05 pu, slope 2–7 %, 90 % of the Q change within t1 = 1 s, settled within t2 = 5 s (tolerance 5 % of Qmax) | RfG 21(3)(d)(ii–iv) | `RefFlag` 1, `Vref`, `Vslope` 4 %, cascade (`Vlocal` 1) | 2 % grid drop: t90 0.40–0.59 s, settled within 0.52 s, overshoot 5–22 % for slopes 2–7 % and SCR 3–10 |
| Q, voltage and PF control modes; PF: start ≤ 2 s, complete ≤ 30 s | RfG 21(3)(d)(i, vi) | `RefFlag` 0 / 1 / 2, `PF`, `PFsign` | PF 0.95: 0.953 export (47.7 Mvar), 0.950 import (−49.3 Mvar) |
| Q range at the POC | not cited in the repository | `Qmax`/`Qmin` ±0.33 pu (≈ ±59.4 Mvar, PF 0.95 at 180 MW), held by the POC Q limit | grid −10 % / +8 %: Q_poc ±59.40 Mvar |
| P set-point accuracy 2 % Pn over 1 min | RfG 15(2)(a) | closed P loop at the POC | P_poc 150.00 MW for 150 MW; ramp excess 0.5 MW < 3.6 MW |
| Fault ride-through | not cited in the repository | asset level: reactive current injection, P recovery ramps (chapter B3); HPPC P and Q PIs frozen in dips | `wt_frt`: recovery within 0.1–0.5 s, no overshoot |

Notes on the table:
- The ±0.33 pu Q range is the model's choice, matching PF 0.95 at Pn (tan(acos 0.95) = 0.329). The P- and V-dependent U-Q/Pmax profile of the RfG is not modelled (open point). The specific RfG article for the Q range is not recorded in the repository (inferred: RfG Art. 21(3)(b)).
- The FRT requirement values (voltage-time profile) are not recorded in the repository. The `wt_frt` cases test dips to 0.5 and 0.2 pu residual grid source voltage for 150 ms. They show that the plant rides through and recovers, not compliance with a specific curve.
- The 1 % band and the 60 s window for the ramp are the evaluation methods recorded in design.md step 3.

**Why the ramp limit also applies to availability.** The requirement is on the plant output gradient. Without it, the wind step at 20 s (+45 MW of availability) would be a gradient violation. The BESS absorbs the surplus meanwhile.

**Why the small ramp excess is accepted.** After the wind step the PI takes up the losses, which adds about 0.5 MW to the ramp within a 60 s window. That is within the 2 % Pn (3.6 MW) accuracy (design.md step 3). Removing it is an open point.

### B7.3 Verification per case set

All results are from the default layout and the nodal network, as recorded in design.md (section 8) and in the report figure statements in `scripts/reportFigures.m`. The figure statements are checked against the results when the cases are rerun (design.md section 12). No new runs were made for this handbook.

#### `hppc_p`: P control and ramp limit (120 s runs, P_ref step at 40 s)

Cases: `Pref 150 MW`, `step -30 MW`, `step +20 MW`, `step +40 MW (limit)`, `step -30 MW, 10 %/min`, `step -30 MW, no ramp`. Before 20 s the wind plant has 0.6 pu available, so 150 MW cannot be met (P_poc 137.8 MW with the BESS at its discharge limit, step 1).

- Nominal: P_poc 150.00 MW (146.52 MW open loop); the BESS covers the losses (−1.51 instead of −5.01 MW).
- Steps reached after 48 / 31 / 47 s; +40 MW stops at the 180.00 MW export limit. 10 %/min: 18 MW/min. Without the ramp limit: about 1 s.
- Step 1 results without the ramp limit (still valid for the dynamics of the loop): −30 MW undershoot 1.6 MW, within 1 % after 1.0 s; +20 MW overshoot 0.8 MW, 0.5 s.
- Report figures: `ramp_steps` ("P_ref steps follow the ramp rate constraint of 20 % Pn/min (36 MW/min); +40 MW stops at the 180 MW export limit"), `ramp_rates`, `ramp_dispatch` ("the BESS takes the change first; wind is curtailed only when the BESS reaches its charge limit"; −30 MW: BESS at −30.3 MW, wind 132.1 → 131.0 MW).

#### `freq_step`: frequency response (step at 50 s, 70 s runs)

Cases: `no event`, `+0.2 Hz (LFSM-O limit)`, `+0.5 Hz LFSM-O`, `-0.5 Hz LFSM-U`, `+0.2 Hz FSM`, `-0.2 Hz FSM`, `+0.5 Hz FSM + LFSM-O`, `+0.2 Hz, decentral`, `-0.2 Hz, decentral`.

- +0.2 Hz (at the threshold): no response. ±0.5 Hz: ∓21.60 MW. FSM ±0.2 Hz: ∓13.68 MW. FSM + LFSM-O +0.5 Hz: −39.6 MW. Start 0.1 s, 90 % after 0.3 s (requirement ≤ 2 s, ≤ 30 s).
- Hand check: (0.5 − 0.2) Hz / 50 Hz / 0.05 = 0.12 pu = 21.6 MW; FSM (0.2 − 0.01)/50/0.05 = 0.076 pu = 13.68 MW.
- Report figures: `freq_lfsm`, `freq_fsm`, `freq_central_decentral` (B5.15).

#### `hppc_qv`: Q, voltage and PF control (90 s runs, events at 40 s)

Cases: Q 0, Q steps ±30 Mvar, voltage control after a 2 % grid drop (4 % SCR 10, 4 % SCR 3, 2 % SCR 10, 7 % SCR 3, SCR 3 set for SCR 10), grid −10 % / +8 % (Q limit), plants limited to 0.2 pu with and without anti-windup, Q control in the 2 % drop, `Vref` 1.01, PF 0.95 export / import.

- Q control: Q_poc 0.00 Mvar for Q_ref 0 (−11.39 Mvar open loop); V_poc 1.0048 instead of 0.9985. Steps ±30 Mvar: figure `q_steps` "90 % after 0.35 s, settled within 0.7 s (overshoot 6–10 Mvar, mostly from the plant and asset Q loops)"; design.md step 4: overshoot 6–10 Mvar, settled 0.65 s.
- Voltage control (2 % drop): t90 0.40–0.59 s, settled within 0.52 s, overshoot 5–22 % (figure `v_control`). Q control in the same event keeps Q_poc at 0, and V_poc drops to 0.985 (step 2).
- Wrong SCR (set for 10 on an SCR 3 grid): t90 0.12 s, 19 % overshoot (was 50–90 % with direct POC control); figure `v_scr_mismatch` "the fast busbar voltage control makes it robust".
- Q limit: ±59.40 Mvar after about 20 s; plants overshoot to +69 / −72 Mvar first (figure `v_qlimit`).
- Anti-windup: Q back within 3 Mvar 0.9 s after the voltage returns, against 9.5 s at +15.8 Mvar without (figure `v_antiwindup`).
- PF 0.95: 0.953 export (47.7 Mvar), 0.950 import (−49.3 Mvar) (figure `pf_control`).
- `Vref` 1.01: Q_poc 9.4 Mvar (recorded in step 2, before the slope; not rechecked in the records after step 3).

#### `wt_frt`: fault ride-through (150 ms dips at 45 s)

Cases: `no dip`, `dip 0.5`, `dip 0.8`, `dip 0.5 SCR3`.

- P_poc relative to the no-dip case back within 1.5 MW 0.13 / 0.51 / 0.29 s after clearing, no overshoot (step 3); figure `frt_p` "P recovers within 0.5 s after clearing, without overshoot (HPPC P loop frozen during the dip)".
- Voltage and Q (stage 4, before the HPPC loops): dip 0.5 V_poc ≈ 0.56 pu, Q wind ≈ 56, PV ≈ 17, BESS ≈ 10 Mvar; figure `frt_vq` "reactive current injection supports the POC voltage".
- Known artefact: a single-step spike at fault clearing (Q_poc ≈ 125 Mvar for one step) from the network with one-step-delayed current sources.

#### `bess_soc`: SOC limits (120 s runs)

Cases: `SOC0 0.5`, `SOC0 0.15`, `at SOCmin`, `SOC0 0.85`, `at SOCmax`, `SOC0 0.5, +20 MW`, `SOC0 0.15, +20 MW`.

- SOC0 0.15: BESS discharge derated to 14.6 MW (29.7 MW at SOC 0.5). At SOCmin: 0 MW. SOC0 0.15 with +20 MW: BESS 13.5 MW, P_poc 165.0 instead of 170 MW. Near SOCmax: charging −0.8 MW, wind and PV curtailed, P_poc 150 MW (step 3).
- Figures `soc_derating`, `soc_step` ("the derated BESS limits the plant to 165 MW").

#### `grid_strength`: SCR 10 / 5 / 3

- P_poc 150.00 MW in all cases (step 1). The voltage-control results at SCR 3 are in `hppc_qv`. There is no report figure for this set.

### B7.4 Known limitations and open points

From design.md section 10 and the README status. Not modelled:

- the island-detection delay of 500 ms (distribution-connected storage only), synthetic inertia, FRT current requirements at the POC (asset level only), the U-Q/Pmax profile and Q capability per P, the LFSM-O behaviour at the minimum regulating level (design.md step 3);
- V_ref and PF from the TSO (they are parameters, `p.hppc.Vref`, `PF`);
- Q dispatch by available Q capability instead of rating;
- SOC restoration towards a target SOC;
- tap changer and reactive compensation (STATCOM, shunt reactors);
- steady-state initialisation: the controllers start from zero measurements.

Open issues:

- the ramp-limit excess after availability steps (0.5 MW);
- the P_poc dip of about 13 MW for 0.3 s at the wind step (dispatch on availability instead of measured output);
- voltage gains depend on the commissioned SCR (`SCRset`); an online dV/dQ estimate would remove this;
- the requirements must be checked against the current RfG revision (the copy used is marked historic);
- the WPPC uses REPC_A; the IEC 61400-27-1 WPP control model is a later refinement;
- block details and parameter values of the IEC and WECC models are typical, not from a specific product;
- the spike at fault clearing;
- the research question: how central HPPC loops interact with plant controllers that keep their own P and frequency loops (decentral).

### B7.5 Contradictions noticed

- README calls the project "stage 5 of 5 started", and design.md's status line says stage 5 is in progress. Both agree. The README result "voltage control 90 % within 0.4–0.6 s" agrees with design.md step 4. The earlier step-3 numbers (t90 0.49–0.59 s) were superseded by the cascade.
- The `q_steps` figure statement ("90 % after 0.35 s, settled within 0.7 s") and design.md step 2 ("within 5 % after 0.8–0.9 s") differ. Step 2 used `Kiq` 0.5 before the cascade retuning; step 4 records "settled 0.65 s". The figure statement is the newer one.
- The case set files `hppc_qv.m` and `bess_soc.m` have uncommitted changes in the working tree (same stop time for all cases: 90 s and 120 s). They do not change the cases themselves.

### Recreating it

1. Collect the DK1 requirements (current revisions of the RfG requirements for type D and TR 3.3.1) and list each with its range and default.
2. Map each requirement to an HPPC function and parameter (table in B7.2), with the asserts on the ranges in `controllerParameters`.
3. Write one case set per requirement group (`hppc_p`, `freq_step`, `hppc_qv`, `wt_frt`, `bess_soc`, `grid_strength`), events after the plant has settled following the wind step.
4. Define the evaluation (1 % band, 60 s window for the ramp, t90 and 5 %-of-Qmax settling for voltage control) and run the sets.
5. Write one report figure per requirement in `reportFigures.m`, with a statement that is checked against the results.
6. Record not-modelled requirements and open points.

## Appendix X. Discrepancies found while writing this handbook

Found by reading the code against the docs (19 Nov 2026). None of these is fixed yet. "Verified" means checked in the code by the lead author; the rest are as reported in the chapters.

### Out-of-date docs and comments

| Where | What it says | What is true |
|---|---|---|
| `README.md`, model structure | every controller is "one MATLAB Function block `logic`" | the controllers are block diagrams with a few MATLAB Function blocks |
| `README.md`, case sets | seven sets, 46 cases; no `comm_delay` | 11 sets, 73 cases, including `comm_delay` |
| `docs/design.md` §3 | a change of N needs `buildHPP(All=true)` | `buildHPP()` is enough |
| `docs/design.md` §4 | `omega_gen` and `V_ref` are ports | neither is a port |
| `docs/design.md` §6 | the 20 ms path is "POC measurement → HPPC" | `comm.meas_hppc` covers the POC and the plant measurements |
| `docs/design.md`, stage 5 step 2 | Q steps settle in 0.8–0.9 s | 0.7 s after the later retuning (`reportFigures.m`) |
| `docs/design.md` / B3 | WT reactive current limit `iqH1` in dips | `iqH1` limits the reactive current at all times |
| `params/controllerParameters.m` | `Ppoc_max` "not used yet" | the HPPC export limit uses it |
| `params/HPP_parameters.m` help | example `p.ctrl.Kp = 0.8` | that field no longer exists |
| `params/simulationParameters.m` | refers to `p.ctrl.Ts` | each controller has its own `Ts` |
| `utilities/build/buildHPP.m` comments | "one MATLAB Function block logic", "placeholder logic", "unless Force=true", old `makeLink` delay text, "constants for now" in `buildTSO` | stale; `controllerSpecs` / `buildController` are no longer called |
| Simscape run time | ~40 s per run (guide), 107–127 s (design.md), "about 5×" (`networkCode.m`) | depends on case and load; one consistent figure should be measured |

### Behaviour worth checking or fixing

| Item | Detail |
|---|---|
| Plant measurement filters do nothing (**verified**) | `lagBlk` implements $y_k = p\,y_{k-1} + (1-p)\,u_k$ with $p = 1 - T_s/T$. The plant filters `Tfltr` = `Tp` = 0.02 s equal the plant sample time 0.02 s, so $p = 0$ and the output equals the input. Either intended (no filtering) or the constants should be larger. |
| Plant Q feed-forward switch | wired to a constant, so it cannot be switched by a parameter |
| `checkModelBuild` compares only N | a different feeder split with the same N would leave the Simscape variant on the old layout |
| Controller `Ts` from model defaults | a case set that changes `ctrl.*.Ts` would have no effect *(inferred)* |
| Possible start-up glitch | in the voltage links the Memory sits before the Delay and starts at 0, so the plant controllers may read V_meas = 0 once at t = 20 ms *(not verified)* |
| `caseLabels` | would print PF values with "MW"; no current set is affected |
| Asset solver setting | the asset models are set to ode4; the top model's ode1 governs as referenced models run with the top solver *(inferred, not measured)* |
| `main.m` line 26 (uncommitted) | reads `if setName == "all"don`: a stray edit that is a parse error |

### Values without a recorded source

The FRT requirement and the ±0.33 pu Q range at the POC are used in the code without a cited source in the repository; the chapters mark them *(inferred)*. The `comm_delay` timings (Q_poc 50 % after 0.03 s without delays, 0.25–0.57 s with them) come from a test run on 19 Nov 2026 whose log is not in the repository (verified against that log).
