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

### Two portability seams

- **Sim-box / court offset.** The court is centred in a sim box; changing the
  box shifts the whole world frame (`NET_X`, `COURT_X_LO`, …). Defaults are
  pepper's serve-focused `36 × 25 × 19`. rakija's full rally box overrides at
  compile time:
  ```
  -DBALLISTIC_SIM_X_MAX=50 -DBALLISTIC_SIM_Z_MAX=25
  ```
- **Spin-saturation ceiling.** `StrikeParams.spin_cap` soft-caps racket-imparted
  spin (tanh, ~3500→5500 rpm). `strike_params_defaults()` sets it from
  `BALLISTIC_SPIN_CAP_DEFAULT` (**on** by default — pepper's behaviour).
  Consumers wanting the legacy uncapped sliding-friction result (rakija parity)
  build with `-DBALLISTIC_SPIN_CAP_DEFAULT=false`, or set `sp.spin_cap = false`
  per call.

> **These seams are compile-time constants baked into the library's own object
> code.** A consumer running the *default* config (pepper: box 36, cap on) links
> the prebuilt `libballistic.a`. A consumer with a *different* config (rakija)
> must **compile `src/physics.c` + `src/strike.c` with its own `-D` flags** — the
> flags on the consumer's own TUs don't reach code already compiled into the
> default `.a`. See `../rakija/Makefile` (`bal_physics.o` / `bal_strike.o`) for
> the pattern. rakija pulls only physics + strike; `toss` is the serve core.

## Status

The extraction is complete — all three consumers migrated:
- **balistic** stands alone, `make test` green (29/29, seeded from pepper + the
  air/humidity model).
- **pepper** links the prebuilt lib (default config), deleted its copies.
- **rakija** compiles `physics`+`strike` from source at its own config
  (`-DBALLISTIC_SIM_X_MAX=50 -DBALLISTIC_SIM_Z_MAX=25
  -DBALLISTIC_SPIN_CAP_DEFAULT=false`); its 50-assertion suite is byte-identical
  to pre-migration (`body_rig` pose/IK stays rakija-local and links alongside).
- **pene** (moon umbrella, Objective-C) matches the default box, so it links the
  prebuilt `libballistic.a` like pepper and sets `sp.spin_cap=false` at runtime.
  It keeps its own older `toss` model; the archive only pulls `physics.o`+
  `strike.o` (no `serve_simulate` clash). Cross-repo via
  `BALISTIC ?= ../../../luigi/projects/balistic`. The pure-C core test passes
  14/14 against the lib; the Cocoa GUI build needs macOS (no AppKit on the Linux
  dev box) — to be confirmed there.

Two correctness fixes landed here while migrating rakija, both pepper-neutral:
`compute_strike` now copies `air_density` into its out-`SwingParams` (it was
omitted from the env-field copy alongside wind/cor/cof), and
`swing_params_set_mode_defaults` initialises `air_density` to its `0` sentinel
so callers that never set it get deterministic air.

Follow-ups: fold each app's pure-physics asserts into `tests/test.c` as
convenient; pene still runs the *pre-rewrite* serve model, so adopting balistic's
current `toss` (arm-sweep release + inverse solver) would be a separate
Cocoa-UI port, not a mechanical migration.
