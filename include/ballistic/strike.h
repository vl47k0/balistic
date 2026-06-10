#ifndef BALLISTIC_STRIKE_H
#define BALLISTIC_STRIKE_H

#include <stdbool.h>
#include "physics.h"

typedef enum {
    STRING_POLY = 0,     /* polyester / co-poly — RPM Blast, Luxilon, etc. */
    STRING_NYLON,        /* nylon / multifilament — softer, less spin */
    STRING_GUT,          /* natural gut — most COR, least friction */
} StringMaterial;

typedef enum {
    STRING_ROUND = 0,
    STRING_SHAPED,       /* textured / shaped surface */
    STRING_TWISTED,      /* octagonal / heavily textured — RPM-Blast-like */
} StringShape;

typedef enum {
    STRING_OPEN_16x19 = 0, /* open pattern — more spin, less control */
    STRING_DENSE_18x20,    /* dense pattern — more control, less spin */
} StringPattern;

const char *string_material_str(StringMaterial m);
const char *string_shape_str(StringShape s);
const char *string_pattern_str(StringPattern p);

/* One direction of the string bed: mains (vertical) or crosses (horizontal).
 * Mains run from the top of the racket to the throat; they're the strings
 * that snap back laterally during the strike and generate most of the spin.
 * Crosses run side-to-side and mostly set the bed's overall stiffness/COR. */
typedef struct {
    StringMaterial material;
    StringShape    shape;
    double         tension_lbs;
} StringSet;

typedef struct {
    StringSet     mains;         /* vertical — dominates spin generation */
    StringSet     crosses;       /* horizontal — dominates string-bed COR */
    StringPattern pattern;       /* overall layout (open vs dense) */

    /* Cached / override coefficients (auto-filled from mains+crosses+pattern,
     * but the UI lets the user nudge them after). */
    double mu_strings;           /* string-ball friction (Coulomb) */
    double cor_strings;          /* string-bed COR contribution (~0.80–0.95) */
    double spin_efficiency;      /* snap-back multiplier on angular impulse,
                                  * ~0.85 (nylon round, high tension) to
                                  * ~1.30 (poly twisted, low tension) */
} Strings;

typedef struct {
    double length_m;             /* total racket length (~0.685 m = 27") */
    double weight_kg;            /* total mass (~0.300 kg) */
    double balance_mm;           /* from butt cap (~320 mm) */
    double stiffness_RA;         /* 50 (soft) – 75 (stiff) */
    double arm_length_m;         /* shoulder-to-grip distance (~0.60 m);
                                  * acts as the lever arm for the arm's
                                  * angular velocity in the kinetic chain. */
} Racket;

/* Kinetic-chain inputs. We model the swing kinematically (each segment's
 * end-state velocity at contact, not muscle forces) which is what
 * biomechanics labs actually measure with high-speed video. The composition
 *
 *   v_racket = v_leg
 *            + ω_hip      × r_hip_to_racket        (mostly forward)
 *            + ω_shoulder × r_shoulder_to_racket   (mostly forward + slight up)
 *            + ω_arm      × arm_length             (along the arm direction)
 *
 * yields the racket head velocity at contact. Each segment's lever arm and
 * direction are fixed-by-convention constants for a typical right-handed
 * forehand; only the rates are exposed to the user. The "arm direction"
 * is the swing_path_elev/az pair on StrikeParams — that's the conscious
 * low-to-high (topspin) vs high-to-low (slice) choice. */
typedef struct {
    /* 3D body translational velocity at contact (centre-of-mass motion).
     *   forward  +x: leaning/running into the shot (the dominant "weight
     *                transfer" effect — a positive value is what coaches
     *                mean by "lean into it").
     *   vertical +y: jumping (kick-serve push-off, open-stance lift).
     *   lateral  +z: sliding/chasing toward the player's left. Nonzero
     *                lateral velocity transfers into the racket head and
     *                tends to spray a defensive shot off its intended line. */
    double body_v_forward;       /* m/s */
    double body_v_vertical;      /* m/s */
    double body_v_lateral;       /* m/s */

    double hip_rate_dps;         /* deg/s — hip rotation around vertical axis */
    double shoulder_rate_dps;    /* deg/s — shoulder rotation around spine */
    double arm_rate_dps;         /* deg/s — arm angular velocity around shoulder */

    /* Geometric lean — body posture at contact (orientation). Distinct from
     * body_v_*, which represents translational momentum (the "weight
     * transfer" effect of leaning). Lean rotates the rotational kinetic-chain
     * contributions in world frame: forward lean tilts the swing plane down,
     * making a topspin path flatter (or even hitting-down for big lean).
     * The body's CoM translation is unaffected — it's already in world frame. */
    double lean_forward_deg;     /* + forward, - back ("falling away") */
    double lean_lateral_deg;     /* + toward player's left (+z) */
} KineticChain;

/* Reasonable rates for a given shot type. */
void kinetic_chain_defaults(KineticChain *kc, ShotMode mode);

/* Compose the 3D racket head velocity vector at contact. arm_dir_* give the
 * orientation of the arm's contribution (low-to-high for topspin etc.). */
void kinetic_chain_racket_velocity(const KineticChain *kc,
                                   double arm_length_m,
                                   double arm_dir_elev_deg,
                                   double arm_dir_az_deg,
                                   double out_v[3]);

/* Auto-fill mu_strings + cor_strings from material/shape/tension. */
void strings_recompute(Strings *s);

/* Effective mass at the impact point (sweet spot heuristic). */
double racket_effective_mass(const Racket *r);

/* Apparent COR contribution from the frame. */
double racket_frame_factor(const Racket *r);

typedef struct {
    /* Incoming ball at racket contact (post-bounce / mid-flight off opponent).
     * speed=0 means ball-at-rest (e.g. serve toss apex). */
    double in_speed;
    double in_elevation_deg;     /* + = ball rising toward player */
    double in_azimuth_deg;       /* + = ball coming from player's left */
    double in_spin_rpm;          /* + = topspin (on the incoming ball) */
    double in_sidespin_rpm;

    /* Contact point (= start_x/y/z for the outgoing trajectory) */
    double contact_x, contact_y, contact_z;

    /* Racket motion at contact — the arm's swing direction. The conscious
     * brushing motion: low-to-high for topspin, high-to-low for slice.
     * Hip and shoulder contributions are added via the kinetic chain. */
    double swing_path_elev_deg;  /* + = low-to-high (topspin path) */
    double swing_path_az_deg;    /* horizontal swing direction */
    double face_angle_deg;       /* + = open (slice), - = closed (topspin)
                                    — legacy; only consulted when the
                                    geometric ball_contact_* inputs are
                                    NaN. The new path is to set the two
                                    fields below. */

    /* Geometric contact-point on the ball, ball-fixed frame.
     * (lon, lat) — lon yaw around the ball's vertical, lat
     * elevation. (0, 0) is directly in front of the ball along its
     * direction of travel; +lat = above equator; +lon = ball's right.
     * Used to construct the racket face normal at impact directly:
     *   n_surf = -(ball outward normal at contact).
     * Pass NaN in both fields to fall back to face_angle_deg +
     * swing_path_az_deg construction (the pre-geometry behaviour). */
    double ball_contact_lon_deg;
    double ball_contact_lat_deg;

    /* Direct racket-head velocity magnitude at impact, m/s. When
     * non-NaN, compute_strike builds v_r from
     *   v_r = racket_velocity_override_mps
     *         * (cos(elev)·cos(az), sin(elev), cos(elev)·sin(az))
     * with elev/az taken from swing_path_elev_deg / swing_path_az_deg,
     * and skips kinetic_chain_racket_velocity entirely. The "swing
     * plus movement" pre-summed by the UI's 3D swing-arrow widget.
     * Pass NaN to fall back to the kinetic-chain construction. */
    double racket_velocity_override_mps;

    /* Body mechanics that drive the racket head velocity. */
    KineticChain kc;

    /* Equipment */
    Racket   racket;
    Strings  strings;
    BallType ball_type;

    /* Soft-saturate racket-imparted spin via a tanh ceiling (SPIN_KNEE/SPIN_MAX
     * in strike.c) so a glancing brush can't manufacture unbounded RPM.
     * strike_params_defaults() sets this ON (pepper's behaviour). Consumers that
     * want the legacy uncapped sliding-friction result (rakija parity) set it to
     * false after calling defaults, or zero-init the struct. */
    bool spin_cap;
} StrikeParams;

/* For inspection in the UI. */
typedef struct {
    /* Resolved racket head velocity (kinetic chain → racket) */
    double racket_speed;
    double racket_path_elev_deg;
    double racket_path_az_deg;
    /* Ball exit state */
    double exit_speed;
    double exit_angle_deg;
    double exit_azimuth_deg;
    double exit_spin_rpm;
    double exit_sidespin_rpm;
    /* Impact diagnostics */
    double e_a_effective;
    double mu_used;
    double m_eff_kg;
    /* String-bed dynamics. pocket_depth_m is the depth the ball
     * sinks into the bed during compression (harmonic-oscillator
     * estimate from ball mass + bed stiffness + normal impact
     * velocity). dwell_ms is the full contact time (the half-period
     * of that oscillator) — measurements give 3-5 ms for a typical
     * pro setup; the model lands at ~3.4 ms for the default kit.
     *
     * mains_engaged is a derived count: contact-patch diameter on
     * the ball (a function of pocket depth + ball radius) divided
     * by the pattern's mains spacing. Fractional values are kept so
     * the UI can render "≈4.7" instead of just collapsing to an
     * int. Typical range is 3-5 across realistic impact speeds. */
    double pocket_depth_m;
    double dwell_ms;
    double mains_engaged;
} StrikeOutput;

/* Reasonable starting defaults. */
void strike_params_defaults(StrikeParams *sp, ShotMode mode);

/* Run the strike model + populate the SwingParams that feed simulate_swing. */
StrikeOutput compute_strike(const StrikeParams *sp,
                            SwingParams *out_swing,
                            const SwingParams *conditions /* wind, ground cor/cof, mode */);

#endif
