#ifndef BALLISTIC_TOSS_H
#define BALLISTIC_TOSS_H

/* pepper's core: a serve as two time-parametrized trajectories that
 * meet in space and time.
 *
 *   1. The TOSS — the ball is released near the server and follows a
 *      projectile arc (gravity + light quadratic drag, no spin).
 *   2. The SWING — the racket head sweeps a circular arc of radius
 *      `swing_radius_m` about a shoulder pivot. The arc lies in a
 *      plane whose orientation is set by (plane_elev, plane_az); the
 *      head passes through "full extension" (the arc apex) at
 *      `apex_time_s` after the toss release, moving at
 *      `swing_speed_mps`.
 *
 * The CONTACT solver walks both tracks and finds the first instant the
 * swept head comes within a ball-radius of the falling ball. At that
 * instant the *contact angle is an output*, not an input: it falls out
 * of where on the arc (which φ) the racket happened to meet the ball,
 * which in turn depends on the toss and the swing timing. Mistime the
 * swing and you meet the ball early (racket still rising → ball sails
 * long) or late (racket coming down → ball into the net), or miss it
 * entirely.
 *
 * Once contact geometry is known it is mapped onto rakija's StrikeParams
 * and handed to compute_strike() → simulate_swing() for the exit ball +
 * flight verdict. The face is modelled square to the swing path (a flat
 * hit); the swing direction at contact therefore drives the exit
 * direction. Brushing the face for slice/kick is a future knob. */

#include <stdbool.h>
#include <stddef.h>

#include "physics.h"
#include "strike.h"
#include "pose.h"

typedef struct {
    /* Server stance — on the player's own side, near the baseline.
     * The toss is released above-and-slightly-in-front of this point. */
    double player_x;
    double player_z;
    double release_height_m;       /* height the ball leaves the hand */

    /* Toss. The tossing arm is a straight rod of length arm_length_m pivoting at
     * the LEFT shoulder, sweeping UP through the toss plane; the ball leaves the
     * hand at release_angle_deg along that sweep and flies as a projectile with
     * the hand's tangential velocity (toss_speed_mps). The release angle is what
     * places the toss forward-in-court vs behind-the-baseline — NOT a lean. */
    double toss_speed_mps;         /* hand speed at release (tangential) */
    double release_angle_deg;      /* where on the arm sweep the ball is let go,
                                    * measured from forward-horizontal up and over:
                                    * 90 = hand straight out front (ball goes
                                    * forward into the court); larger = LATER
                                    * release, hand higher/over → ball goes BACK
                                    * behind the baseline. ~180 = straight back. */
    double toss_fwd_deg;           /* LEGACY (old lean model) — unused */
    double toss_side_deg;          /* small out-of-plane (+z) velocity add */
    double toss_plane_deg;         /* angle of the toss plane to the NET:
                                    * 0 = parallel to the net, 90 = perpendicular
                                    * (straight down the court). Sets the vertical
                                    * plane the tossing arm sweeps in. */
    /* Toss spin (rpm). A clean controlled toss is ~0, but a high, angled toss
     * imparts spin that curves it via Magnus (same axis convention as flight:
     * +topspin / +sidespin). The serve's ball track carries these; for a
     * groundstroke the incoming ball's spin is strike.in_spin_rpm instead. */
    double toss_spin_rpm;
    double toss_sidespin_rpm;

    /* Swing-plane arc. plane_elev/plane_az are the swing-through direction
     * at full extension (the way the head travels through the ball) — for
     * a cleanly-timed hit this is the exit direction. The reach (apex)
     * direction is derived as world-up perpendicular to it. */
    double arm_length_m;           /* shoulder → hand reach. Swing radius (RS →
                                    * racket string) = arm_length_m + racket_len_m;
                                    * toss release sits arm_length_m in front of
                                    * the left shoulder. */
    double shoulder_height_m;      /* player's shoulder height — the toss-release
                                    * and swing-pivot height. ~1.5 m adult; set
                                    * lower for a junior (≈0.82·body height). */
    double racket_len_m;           /* racket length added to the swing radius.
                                    * 0.6858 m = 27" adult; ~0.635 m = 25" junior. */
    double swing_radius_m;         /* legacy: superseded by arm_length_m + racket */
    double reach_tilt_deg;         /* forward tilt of the reach (full-extension) off
                                    * vertical, toward the box: high (~28°) puts the
                                    * contact OUT in front (flat 1st serve); low (~5°)
                                    * puts it OVERHEAD so a near-vertical toss falls
                                    * onto an up-brushing racket (kick 2nd serve). */
    double plane_elev_deg;         /* swing-through elevation (− = downward) */
    double contact_angle_deg;      /* face tilt (brush) relative to the swing-through,
                                    * in the vertical plane: + opens the face UP
                                    * (brush up the back of the ball → topspin/kick),
                                    * − closes it (brush down → slice/backspin).
                                    * This is the "angle of contact" that drives spin
                                    * independent of the gross swing direction. */
    double side_contact_angle_deg; /* lateral face tilt (brush) in azimuth: turns
                                    * the face off the swing path sideways → SIDESPIN
                                    * (slice/kick curve). + brushes toward +z. */
    double plane_az_deg;           /* fine azimuth offset on top of swing_plane_deg */
    double swing_plane_deg;        /* angle of the swing plane to the NET:
                                    * 0 = parallel to the net, 90 = perpendicular
                                    * (straight down the court). Leans toward the
                                    * target service box. */
    double contact_height_m;       /* height of the arc apex (full extension) */
    double swing_speed_mps;        /* racket-head tangential speed at full extension */
    double swing_start_speed_mps;  /* head speed entering the tracked loop at SS
                                    * (the backswing/leg-drive momentum); the head
                                    * accelerates from this to swing_speed_mps by
                                    * full extension. */
    double apex_time_s;            /* time after release when head hits apex */

    /* Equipment + conditions handed straight to the strike + flight
     * models. The incoming-ball / contact-point / racket-velocity
     * fields of `strike` are overwritten by the contact solver; the
     * racket, strings and ball_type are used as-is. */
    StrikeParams strike;
    ShotMode mode;                 /* MODE_SERVE_DEUCE or MODE_SERVE_AD */
    bool catch_descending;         /* strike the ball AFTER the toss apex (falling,
                                    * hit from underneath) — the kick/2nd-serve
                                    * contact. Off = struck at/before the apex
                                    * (flatter 1st serve). */
    bool right_handed;             /* tosses with the off hand → the toss is
                                    * released on the opposite side to the
                                    * racket and lofts across to the contact */
    double shoulder_to_net_deg;    /* angle of the shoulder line to the net:
                                    * 0 = parallel (square stance), 90 = side-on */
    double cor_ground, cof_ground;
    double windspeed, winddirection;
    double air_density;            /* kg/m^3; 0 → sea level. Set from altitude +
                                    * temperature in the UI. Less drag + Magnus. */
    double air_temp_c;             /* court temperature (°C); warms the ball →
                                    * livelier off the strings AND the court. */
    double air_humidity_pct;       /* relative humidity (0..100); 0 = dry. Folded
                                    * into air_density (muggy air is a touch
                                    * thinner → marginally less drag + Magnus). */
} ServeParams;

typedef struct {
    /* Toss path (release → ground or end of search), world coords. */
    double *toss_x, *toss_y, *toss_z;
    size_t  toss_n;

    /* Racket-head path sampled across the whole swing window, world
     * coords — drawn regardless of whether contact was made. */
    double *rkt_x, *rkt_y, *rkt_z;
    size_t  rkt_n;

    /* Contact event. */
    bool   contact;
    double t_contact;              /* s after release */
    double contact_pos[3];         /* world */
    double phi_contact_deg;        /* arc angle from apex; 0 = full extension */
    double miss_distance_m;        /* closest head↔ball approach (any case) */
    double swing_az_deg;           /* actual swing-through azimuth (auto-aim + offset) */
    double toss_az_deg;            /* actual toss horizontal bearing (for the views) */
    double swing_pivot[3];         /* RS — the racket-arc centre (for the swing plane) */
    double swing_u[3], swing_v[3]; /* swing-plane basis: u = arm (reach), v = swing-through */
    double ls_pos[3];              /* LS — the tossing shoulder / toss release pivot */
    double toss_apex[3];           /* highest point of the toss path (world) */
    double toss_te[3];             /* toss endpoint: where the toss would land on the
                                    * ground if it were never struck (world; y≈0) */
    double ibs[3];                 /* Incoming Ball Start — the ball-track anchor:
                                    * the release point for a serve, the near-contact
                                    * anchor for a groundstroke (world) */
    double ibs_vel[3];             /* the ball's velocity at IBS; −ibs_vel points back
                                    * to where the ball came from (the "feed" side) */

    /* Contact-frame vectors (world coords) — for drawing the racket face
     * disc + its normal + the velocity arrows so the contact angle reads
     * directly off the 3D view. Only set when contact == true. */
    double face_normal[3];         /* unit; = swing tangent (flat hit) */
    double racket_vel[3];          /* racket-head velocity at contact */
    double incoming_vel[3];        /* toss-ball velocity at contact */

    /* Impact dynamics (only when contact == true). From the ball's velocity
     * change over the contact dwell: impulse J = m_ball·(v_out − v_in), average
     * force F = J/dwell. The racket (and the arm behind it) feel the Newton
     * reaction −F at the contact point; arm_torque is that reaction's moment
     * about the hand (lever = the racket length from grip to contact). */
    struct {
        double ball_impulse[3];    /* N·s on the ball */
        double ball_force[3];      /* N, average over the dwell, on the ball */
        double racket_force[3];    /* N on the racket = −ball_force */
        double arm_torque[3];      /* N·m about the hand (r_grip→contact × racket_force) */
        double ball_force_mag;     /* |ball_force| (N) */
        double arm_torque_mag;     /* |arm_torque| (N·m) */
        double racket_eff_kg;      /* effective racket mass at the impact point */
    } impact;

    /* Resolved strike + flight (only meaningful when contact == true). */
    StrikeOutput strike_out;
    Trajectory   flight;
    Trajectory   flight_nospin;    /* same launch, zero spin — the no-Magnus
                                    * reference; the gap to `flight` is the
                                    * Magnus bend (topspin dip / sidespin curve). */
} ServeResult;

/* Fill p with a sensible right-handed serve into the given service box
 * (clean, lands IN with the default equipment). */
void serve_params_defaults(ServeParams *p, ShotMode mode);

/* Run the toss + swing + contact + strike + flight pipeline. The caller
 * owns the returned arrays + Trajectory; release with serve_result_free. */
ServeResult serve_simulate(const ServeParams *p);

/* The auto-timed swing-start moment: the apex_time_s that lands the racket
 * head on the toss (apex catch, or the descending crossing when
 * catch_descending is set). Depends only on the toss + swing geometry, NOT
 * on apex_time_s. The UI seeds apex_time_s = serve_seed_apex_time(p) + offset
 * so timing tracks the toss; an offset of 0 is a dead-on serve. */
double serve_seed_apex_time(const ServeParams *p);

/* Inverse toss solver: adjust p->release_angle_deg + p->toss_speed_mps so the
 * toss endpoint (where the un-struck ball lands) sits `fwd_of_baseline_m` past
 * the server's baseline (COURT_X_LO) and the toss peaks at `apex_h_m`. Lateral
 * placement is left to p->toss_plane_deg / p->toss_side_deg (the toss bearing
 * is preserved so the toss stays in its contact-compatible plane). Returns true
 * if it converged within tolerance; clamps + returns false if the target is out
 * of the arm/speed envelope. Does not change the swing or contact timing. */
bool serve_solve_toss(ServeParams *p, double fwd_of_baseline_m, double apex_h_m);

/* Lateral toss placement: set p->toss_side_deg so the toss endpoint lands
 * `side_of_ls_m` to the player's left (+) / right (−) of LS, holding everything
 * else. Large offsets may walk the toss out of the swing plane (→ a miss); the
 * caller surfaces the resulting contact verdict. Returns the achieved offset. */
double serve_solve_toss_side(ServeParams *p, double side_of_ls_m);

/* Groundstroke feed solver: constrain the incoming ball to a legal feed that
 * bounced at world x = bounce_x on the player's side and passes through the
 * swing's contact at the configured pace (strike.in_speed). Sets
 * strike.in_elevation_deg accordingly (a deep bounce → descending contact, a
 * short bounce → rising). Returns false if no feed at that pace reaches the
 * contact. So the incoming geometry stops being free — it follows from the bounce. */
bool serve_solve_feed(ServeParams *p, double bounce_x);

void serve_result_free(ServeResult *r);

/* Build a stick-figure pose for the player at the moment of contact, from a
 * simulated shot. Anchors the body's shoulder line on the swing geometry
 * (ls_pos / swing_pivot), matches the figure's arm + racket to the shot's, poses
 * the legs/spine, raises the off-hand, and IKs the racket arm so the tip lands on
 * contact_pos (pinned exactly — the racket meets the ball there). Fills `out`
 * with world-frame joint positions for drawing. Works for serves and
 * groundstrokes (the contact just sits lower/forward). Right-handed for now (a
 * left-hander draws as a right-hander — pose IK is the right arm). Pure C, used
 * by every front-end (pene, pepper) so the body derivation lives once. */
void serve_skeleton(const ServeParams *p, const ServeResult *r, PoseJoints *out);

#endif
