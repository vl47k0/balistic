# balistic

Portable tennis-physics core — the shared C math engine for rakija / pepper /
pene / moon. Pure **C11, `math.h` only**: no GTK, no glib, no json-glib, no UI.
Two halves: the **ball** (flight + strike + serve toss) and the **body**
(skeleton + forward/inverse kinematics). One copy every front-end links against.

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
| `<ballistic/toss.h>` | a swing meeting a ball track — `serve_simulate` covers **both** serves (the toss) and groundstrokes (an incoming ball), via the shared arm-sweep contact solver + the inverse toss solver |
| `<ballistic/pose.h>` | **biomechanics** (the body half): skeleton, forward + inverse kinematics (`pose_fk` / `pose_from_joints`), 3-joint racket IK, swing sampling, named-shot presets, and `pose_drive_swing` (a pose pair → swing pace, so the body drives the shot) |
| `<ballistic/ballistic.h>` | umbrella include for all four |

JSON/UI adapters (rakija's `swing_api` + `body_rig_json`, pepper's `serve_api`)
are **not** here — they stay in the consuming app, which keeps this toolkit-free.

## Build

```bash
make            # -> build/libballistic.a
make test       # math-core assertions, no GUI toolkit (48: ball + body + serves/groundstrokes)
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
- **balistic** stands alone, `make test` green (48/48 — the serve/strike/flight
  suite seeded from pepper + the air/humidity model + a groundstroke (incoming
  ball) shot + the `pose` FK/IK group).
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
