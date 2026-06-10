#ifndef BALLISTIC_PHYSICS_H
#define BALLISTIC_PHYSICS_H

#include <stdbool.h>
#include <stddef.h>

/* Integrator step (s). Trajectory samples are spaced this far apart in time
 * (except at bounce points where we interpolate to y=0; that small jitter
 * is sub-millisecond and ignorable for display purposes). */
#define RAKIJA_DT     0.002

/* ITF singles court (m), from ref/tnsstroke.m.
 *
 * Single unified world frame:
 *   Origin (0, 0, 0) at the corner of the simulation floor.
 *   x ∈ [0, SIM_X_MAX]   along the court length (player → opponent)
 *   y ∈ [0, SIM_Y_MAX]   up
 *   z ∈ [0, SIM_Z_MAX]   across the court width
 *
 * The court is centred in the sim footprint:
 *   Player's baseline at x = COURT_X_LO
 *   Opponent's baseline at x = COURT_X_HI
 *   Net at x = NET_X (= COURT_X_MID, sim midline)
 *   Doubles sidelines at z = COURT_Z_DBL_LO / COURT_Z_DBL_HI
 *   Singles sidelines at z = COURT_Z_W_LO  / COURT_Z_W_HI
 *
 * pepper trims rakija's 50 × 25 box down to a serve-focused footprint:
 * 6 m run-back behind each baseline and ~4 m beyond each sideline is
 * plenty for a serve, and the tighter box makes the court fill the
 * view. Court dimensions themselves are unchanged — only the margins.
 * 36 × 25 × 19 m (the Y extent is full-height for the physics; the
 * 3D view crops it to VIEW_Y_MAX). */
#define COURT_LEN          23.7744
#define COURT_HALF_W       4.1148
#define COURT_DBL_HW       5.4864
#define NET_HEIGHT         0.914
#define SERVICE_LINE_DEPTH 6.4008

/* Sim-box extent — a RUNTIME value now, not a compile-time constant. The court
 * is centred in this footprint, so the box sets the world frame (NET_X,
 * COURT_X_LO, … are derived from it). One prebuilt libballistic.a therefore
 * serves every consumer; call ballistic_set_sim_box() once at startup if you
 * need a non-default frame:
 *   pepper / pene / moon : 36 × 25 × 19 (the default — no call needed)
 *   rakija (full rally box): ballistic_set_sim_box(50, 25, 25)
 * The macros below are only the DEFAULT the globals initialise to (still
 * overridable at build time via -DBALLISTIC_SIM_X_MAX=… if ever wanted). Court
 * *dimensions* (COURT_LEN, NET_HEIGHT, …) are universal; only the margins
 * around the court change with the box. */
#ifndef BALLISTIC_SIM_X_MAX
#define BALLISTIC_SIM_X_MAX  36.0
#endif
#ifndef BALLISTIC_SIM_Y_MAX
#define BALLISTIC_SIM_Y_MAX  25.0
#endif
#ifndef BALLISTIC_SIM_Z_MAX
#define BALLISTIC_SIM_Z_MAX  19.0
#endif

/* The sim box + the court-frame coordinates derived from it. Read-only globals
 * (defined in physics.c, initialised to the defaults above); change them via
 * ballistic_set_sim_box(). Used as plain values throughout — no use site cares
 * that they became variables. */
extern double SIM_X_MAX, SIM_Y_MAX, SIM_Z_MAX;
extern double COURT_X_LO, COURT_X_HI, NET_X, SERVICE_X1, SERVICE_X2;
extern double COURT_Z_MID, COURT_Z_DBL_LO, COURT_Z_DBL_HI, COURT_Z_W_LO, COURT_Z_W_HI;

/* Set the sim-box extent (m) and recompute the derived court constants above.
 * Call once at startup, before simulating, if the default 36×25×19 frame isn't
 * what you want. Not thread-safe — set it before spawning worker threads. */
void ballistic_set_sim_box(double x_max, double y_max, double z_max);

typedef enum {
    MODE_GROUNDSTROKE = 0,
    MODE_SERVE_DEUCE,
    MODE_SERVE_AD,
} ShotMode;

typedef enum {
    VERDICT_IN = 0,
    VERDICT_NET,
    VERDICT_SHORT,
    VERDICT_LONG,
    VERDICT_WIDE,
    VERDICT_NO_BOUNCE,
} Verdict;

typedef enum {
    BALL_ITF_TYPE1 = 0,   /* fast: hard / "speed" ball, used on slow surfaces */
    BALL_ITF_TYPE2,       /* medium / "regular" — default */
    BALL_ITF_TYPE3,       /* slow: ~6% larger diameter (high-altitude) */
    BALL_PRESSURELESS,    /* durable, slightly heavier, lower COR */
    BALL_GREEN,           /* ITF Stage 1 "green dot" junior ball: ~25% lower
                           * compression (lower COR) + draggier felt → flies and
                           * bounces notably slower/shorter. */
} BallType;

typedef struct {
    double mass_kg;
    double radius_m;
    double area_m2;      /* cross-sectional */
    double cd;           /* drag coefficient */
    double cor_with_strings; /* baseline ball contribution to apparent COR */
    double friction_mult;    /* multiplier on string-bed friction */
} BallProps;

BallProps ball_props_for_type(BallType t);
const char *verdict_str(Verdict v);
const char *ball_type_str(BallType t);

typedef struct {
    ShotMode mode;
    double start_x, start_y, start_z;
    double power;
    double angle_deg;
    double azimuth_deg;
    double spin_rpm;          /* + = topspin */
    double sidespin_rpm;
    double windspeed;
    double winddirection;     /* radians */
    double cor;               /* ground COR */
    double cof;               /* ground COF */
    double air_density;       /* kg/m^3; 0 → sea-level default. Scales drag +
                               * Magnus, so altitude (thin mountain air) makes
                               * the ball fly faster/further and curve less. */
    BallType ball_type;
} SwingParams;

void swing_params_set_mode_defaults(SwingParams *p, ShotMode mode);

/* Sea-level reference density (kg/m^3) pepper calibrates against — defined to
 * be the density at sea level at the reference temperature below. */
#define AIR_RHO_SEA   1.177
#define AIR_TEMP_REF_C 20.0   /* the reference "mild day" the defaults assume */

/* Air density (kg/m^3) at a geometric altitude (m) and air temperature (°C):
 * ρ = AIR_RHO_SEA · (P(h)/P0) · (T_ref/T), with the ISA pressure ratio
 * P(h)/P0 = (1 − 2.25577e-5·h)^5.25588. Anchored so (0 m, AIR_TEMP_REF_C) =
 * AIR_RHO_SEA. Thinner air ↑altitude or ↑temperature → less drag + Magnus. */
double air_density(double alt_m, double temp_c);

/* Saturation vapor pressure of water (Pa) at temp_c (Tetens, over water). */
double saturation_vapor_pressure_pa(double temp_c);

/* Air density (kg/m^3) including humidity. rel_humidity_pct in [0, 100];
 * 0 = dry air, identical to air_density(alt_m, temp_c). Humid air is *less*
 * dense — light water vapor (M≈18) displaces heavier dry air (M≈29):
 *   ρ = ρ_dry · (1 − 0.378·Pv/P),  Pv = RH·Psat(T),  P = ISA pressure at alt.
 * So muggy heat thins the air a touch further → slightly less drag + Magnus. */
double air_density_humid(double alt_m, double temp_c, double rel_humidity_pct);

/* Back-compat: density at altitude at the reference temperature. */
double air_density_at_altitude_m(double alt_m);

/* Inverse of air_density(): the altitude (m, clamped 0..4000) that yields the
 * given density at the given temperature. Used to round-trip a loaded serve's
 * air_density back into the altitude control. */
double altitude_from_density(double density, double temp_c);

/* Inverse of air_density_humid(): the altitude that yields the given density at
 * the given temperature + humidity. Divides out the humidity factor (at sea-
 * level pressure — a <0.01% approximation) then inverts the dry model. */
double altitude_from_density_humid(double density, double temp_c, double rel_humidity_pct);

/* Ball "liveliness" vs temperature: a warm ball is springier (higher COR off
 * the strings AND the court), a cold ball is dead. Returns a multiplier on the
 * restitution, 1.0 at AIR_TEMP_REF_C. Clamped to a sane band. */
double ball_cor_temp_factor(double temp_c);

typedef struct {
    double *x, *y, *z;
    size_t n;
    double max_height;
    double landing_dist;
    double landing_x;
    double landing_z;
    double net_clearance;
    double bounce_height_m;   /* peak height of the arc AFTER the first bounce
                               * (0 if it never bounces) — the kick-serve "how
                               * high does it jump" metric. */
    double bounce_apex_x, bounce_apex_z;  /* where that peak occurs (court) */
    int    n_bounces;
    bool   hit_net;
    Verdict verdict;
    double target_x_min, target_x_max;
    double target_z_min, target_z_max;
} Trajectory;

void       trajectory_free(Trajectory *tr);
Trajectory simulate_swing(const SwingParams *p);

/* Generalized impact (the same math powers ground bounces and racket strikes).
 * v_in/w_in are the ball's pre-impact velocity and angular velocity.
 * v_surf and n_surf are the surface's velocity and outward unit normal.
 * cor and cof are the impact's restitution and friction.
 * ball_r is the ball radius (used by the spin-coupling terms).
 * After the call v_in/w_in hold the post-impact values. */
void apply_impact(double v_in[3], double w_in[3],
                  const double v_surf[3], const double n_surf[3],
                  double cor, double cof, double ball_r);

#endif
