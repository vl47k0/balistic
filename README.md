# balistic

Portable tennis **ballistics** core — the shared C math engine extracted from
rakija / pepper / pene. Pure **C11, `math.h` only**: no GTK, no glib, no
json-glib, no UI. This is the one copy of the flight + strike + serve physics
that every front-end links against.

## Why

`physics`/`strike`/`toss` had been copied verbatim across three apps and were
starting to drift (pepper added an air-density model + a spin-saturation
ceiling that rakija and pene never got). balistic is the single source of truth;
consumers link the static lib and delete their local copies.

## What's in it

| header | what |
|---|---|
| `<ballistic/physics.h>` | RK4 flight integrator (quadratic drag + Magnus + altitude wind/air-density model), ground bounce (`apply_impact`), court geometry, ball/verdict model |
| `<ballistic/strike.h>` | sliding-friction racket impact (`compute_strike`) + string-bed model |
| `<ballistic/toss.h>` | serve as two time-parametrised tracks (toss + swing) that meet: contact solver, inverse toss solver, `serve_simulate` |
| `<ballistic/ballistic.h>` | umbrella include for all three |

JSON/UI adapters (rakija's `swing_api`, pepper's `serve_api`) are **not** here —
they stay in the consuming app, which is what keeps this core toolkit-free.

## Build

```bash
make            # -> build/libballistic.a
make test       # math-core assertions, no GUI toolkit (currently 23, seeded from pepper)
make debug      # -O0 -g3 for backtraces
make install    # headers -> $PREFIX/include/ballistic, lib -> $PREFIX/lib (PREFIX=/usr/local)
```

## Using it from a consumer

Point at this tree (siblings in the luigi umbrella don't need `make install`):

```make
CFLAGS += -I../balistic/include
LDLIBS += ../balistic/build/libballistic.a -lm
```

and `#include <ballistic/ballistic.h>` (or the individual headers).

### Two runtime config seams

Set these once at startup (before simulating / spawning worker threads) when the
defaults don't fit. They're **runtime**, so the **one prebuilt `libballistic.a`
serves every consumer** — no per-config build.

- **Sim-box / court offset.** The court is centred in a sim box; changing it
  shifts the whole world frame (`NET_X`, `COURT_X_LO`, … are extern globals
  derived from it). Default is the serve box `36 × 25 × 19`; switch to a full
  rally box with:
  ```c
  ballistic_set_sim_box(50.0, 25.0, 25.0);     /* rakija */
  ```
  (`BALLISTIC_SIM_X_MAX` etc. still set the *default* the globals init to and are
  `-D`-overridable, but no consumer needs that.)
- **Spin-saturation ceiling.** `StrikeParams.spin_cap` soft-caps racket-imparted
  spin (tanh, ~3500→5500 rpm). `strike_params_defaults()` sets it from a runtime
  default (**on**); for the legacy uncapped result:
  ```c
  ballistic_set_spin_cap_default(false);       /* rakija */
  ```
  (Per-call `sp.spin_cap` still overrides.)

## Status

Every consumer links the **one prebuilt `libballistic.a`**:
- **balistic** stands alone, `make test` green (29/29, seeded from pepper + the
  air/humidity model).
- **pepper** links the lib (default config), deleted its copies.
- **rakija** links the same lib and calls `ballistic_set_sim_box(50,25,25)` +
  `ballistic_set_spin_cap_default(false)` at startup for its 50 m rally box +
  uncapped spin (was a from-source `-D` build). Its 50-assertion suite is
  byte-identical across both the extraction and the switch to the prebuilt lib
  (`body_rig` pose/IK stays rakija-local and links alongside).
- **pene** (sibling, Objective-C) runs balistic's **current** serve model end to
  end (it deleted its own older `toss`), so it links the whole archive — physics
  + strike + toss — defaults, no setup call. Its Cocoa serve UI was ported from
  pepper's `controls.c` (conditions, flat/slice/kick presets, the inverse toss
  solver, auto-timing). The pure-C core test passes against the lib; the Cocoa
  GUI build needs macOS (no AppKit on the Linux dev box).
- **moon** (sibling) is an nginx module that links the lib and serves the engine
  over HTTP (JSON in → rendering JSON out); defaults, no setup call.

Two correctness fixes landed here while migrating rakija, both pepper-neutral:
`compute_strike` now copies `air_density` into its out-`SwingParams` (it was
omitted from the env-field copy alongside wind/cor/cof), and
`swing_params_set_mode_defaults` initialises `air_density` to its `0` sentinel
so callers that never set it get deterministic air.

Follow-ups: fold each app's pure-physics asserts into `tests/test.c` as
convenient; build pene's Cocoa GUI on a Mac to confirm the AppKit layer (the
engine side is verified here).
