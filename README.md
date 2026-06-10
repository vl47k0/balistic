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
  spin (tanh, ~3500→5500 rpm). `strike_params_defaults()` sets it **on**
  (pepper's behaviour). Consumers wanting the legacy uncapped sliding-friction
  result (rakija parity) set `sp.spin_cap = false` after calling defaults.

## Status

Step 1 of the extraction: library stands alone and `make test` is green (23/23,
seeded from pepper's suite — the newest baseline). Consumers not yet migrated.

Next: ① migrate **pepper** to link the lib (delete its copies), ② **rakija**
(`spin_cap=false`, `-DBALLISTIC_SIM_X_MAX=50`, gate on its 40-assertion suite),
③ **pene** (moon repo, Objective-C — calls the C lib directly), folding each
app's pure-physics asserts into `tests/test.c` as they move.
