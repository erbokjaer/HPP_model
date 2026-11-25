# Layout rules for the HPP block diagrams

Derived from the hand layouts of `WT_asset`, `BESS_ctrl`, the PLL and the
RoCoF block (Nov 2026). They hold for every diagram the builder makes.

## How the layouts are made

- **Templates from hand layouts.** `utilities/build/captureLayout.m` stores a
  hand layout (block positions, areas, every line route, Goto / From) as
  JSON in `models/layout/`. `utilities/build/applyLayout.m` puts it back on a
  freshly built model: blocks are matched by name (a name map for models
  of the same structure, e.g. WT_ctrl from the BESS_ctrl template),
  missing ones are placed by hand (`Place`), and chosen signals are drawn
  as labels (`Tags`, `Keep`). The signal connections always come from the
  builder, and `applyLayout` checks that they are the same before and
  after (`utilities/build/signalEdges.m`).

  | Template | Used for |
  |---|---|
  | `asset_ctrl.json` (BESS_ctrl) | BESS_ctrl, PV_ctrl, WT_ctrl (name map) |
  | `asset.json` (WT_asset) | WT_asset, PV_asset, BESS_asset (name map) |
  | `pll.json` | the MAF-PLL |
  | `rocof.json` | FreqMeter, block "f average, RoCoF" |

- **Coded layouts in the same style**, where no hand layout exists:
  `layoutRepc` (WPPC, SPPC, BESSC), `layoutHppc` (HPPC), `layoutConverter`
  (Generator system, Converter), `pairBlock` (Current injection, PQ
  measurement), `pllSimple` (simple PLL), all in `utilities/build/buildHPP.m`.
- **Check.** `utilities/build/countCrossings.m` counts line crossings, lines
  through blocks and From blocks of one system level.

A hand change to a model is kept by capturing it again as its template
(or, for a coded layout, by changing the coordinates in the builder).

## A. Signal flow and areas

| # | Rule |
|---|---|
| A1 | Signals flow left to right: an *Inputs* area on the left, an *Outputs* area on the right. |
| A2 | Every area is a group of one function (measurement, P path, Q path, PLL, ...). Areas that feed each other are columns in flow order. |
| A3 | Areas that do not feed each other (parallel paths) share a column and are stacked. |
| A4 | The vertical order of stacked areas follows where their signals come from and go to (P path above Q path because ip_req is above iq_req on the current limit block; PLL at the bottom, next to its outputs). |
| A5 | An area whose outputs are used by several others (voltage filter: V_floor, dip) sits next to its users; its outputs are where Goto labels are acceptable. |
| A6 | A block that collects the paths (current limit logic, dispatch) is its own column between the paths and the outputs. |

## B. Areas as a grid

| # | Rule |
|---|---|
| B1 | Areas in one column share the left and right edge. |
| B2 | Areas side by side start at the same top (and end at the same bottom where they are about equally tall). |
| B3 | A fixed gap (about 20 px) between stacked areas. |
| B4 | No near misses: edges that are meant to line up are equal, not 2 px apart. |

## C. Blocks inside an area

| # | Rule |
|---|---|
| C1 | A chain of blocks is a row: the blocks share their centre line, so the line through them is straight. |
| C2 | Columns are local to the area: a long chain in one area does not widen another. |
| C3 | A constant or parameter sits right next to the block it feeds, level with its port. |
| C4 | A block with several inputs is stretched so its ports meet the lines from its sources (switch 80 to 110 px, current limit block 142 to 436 px, PI blocks 150 px for their limits); its sources are not moved for it. |
| C5 | Round sums (`|++`, `|+-`) take the second input from below. |
| C6 | Compact: rows inside an area are packed with small gaps; no empty columns. |
| C7 | Inputs of blocks whose inputs may be swapped are in the order of their sources: sums (the signs move with the inputs), element-wise products, Min/Max, logic gates, two-input comparisons (operator mirrored). The block computes the same thing. |

## D. Lines

| # | Rule |
|---|---|
| D1 | No crossings. Exception: a few crossings are fine where lines cross on a diagonal (current to power in the generator). |
| D2 | A line is straight, or has one corner (a drop into a block from below or above). No S-bends, no jogs of a few pixels. |
| D3 | A line never runs through a block. |
| D4 | A line does not make a detour (no run around the top of the diagram). |
| D5 | A few feedback lines (P, Q into the measurement) run around the bottom; more become Goto / From. |
| D6 | A long line that is the only one in its channel is fine (SOC below the voltage filter). |

## E. Goto / From and labels

| # | Rule |
|---|---|
| E1 | Direct lines are the default. A Goto / From is used only where a line would cross others, run through a block, detour, or run back, or for a signal used in several places (P_ref, P_filt, dip). |
| E2 | No unnecessary labels: a Goto without a From is removed (local tags only), except a label kept on purpose (`[f_pll]`). |
| E3 | Tag names are the signal names. |

## F. Ports

| # | Rule |
|---|---|
| F1 | Inports in the order of the areas they feed, each level with its first user. |
| F2 | Outports level with their sources (their numbers do not change). |
| F3 | The port order of a subsystem follows its sources and users. |
| F4 | The port order of a referenced model (`WT_ctrl` in `WT_asset`) follows its sources and users; this changes the model's interface in `modelInterfaces.m` (not done yet). |
