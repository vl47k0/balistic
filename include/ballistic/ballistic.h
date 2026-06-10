#ifndef BALLISTIC_H
#define BALLISTIC_H

/* balistic — portable tennis ballistics core (pure C11, math.h only).
 *
 * One umbrella include for the whole library. No GTK, no glib, no JSON — this
 * is the math core shared across rakija, pepper, pene and any future consumer.
 * JSON/UI adapters (swing_api, serve_api, …) live in the consuming app, not
 * here.
 *
 *   physics.h  flight integrator (RK4 + drag + Magnus + wind/air model),
 *              ground bounce (apply_impact), court + ball + verdict model.
 *   strike.h   sliding-friction racket impact (compute_strike) + string model.
 *   toss.h     serve as two time-parametrised tracks (toss + swing) that meet:
 *              contact solver, inverse toss solver, serve_simulate.
 *   pose.h     biomechanics — skeleton, forward + inverse kinematics, swing
 *              sampling (the body half; the others are the ball half).
 *
 * Consumers may include this umbrella, or the individual headers as
 * <ballistic/physics.h> etc. The sim box + spin-cap default are runtime settings
 * (ballistic_set_sim_box / ballistic_set_spin_cap_default). */

#include "physics.h"
#include "strike.h"
#include "toss.h"
#include "pose.h"

#endif
