#define _USE_MATH_DEFINES
#include "toss.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Integration step for the toss + the contact march. The racket head
 * sweeps fast (ω = swing_speed / radius ≈ 30 rad/s, ~90° in 50 ms), so
 * we step finer than the flight integrator to resolve the brief contact
 * window against the slow-moving toss. */
#define TOSS_DT        0.0005
#define TOSS_T_MAX     3.0
#define G_TOSS         9.81
#define AIR_RHO        1.177

/* The racket is only "in play" within ±PHI_MAX of full extension; outside
 * that the arm is still loading / has followed through and can't make a
 * legal hit. */
#define PHI_MAX_RAD    (100.0 * M_PI / 180.0)

/* Capture radius: ball radius plus a sweet-area tolerance for the head. */
#define HEAD_TOL_M     0.07

/* The ball leaves the hand slightly in front of the stance. */
#define RELEASE_FWD_M  0.15

/* Off-hand release offset (m) to the side: a right-hander tosses with the
 * left hand, so the ball starts off to one side and is lofted back across. */
#define TOSS_ARM_DZ    0.60

/* Shoulder half-width (m): LS/RS sit this far either side of the stance along
 * the shoulder line. The toss is released from LS, the swing pivots at RS. */
#define SHOULDER_HALF  0.22
#define SHOULDER_H     1.50   /* shoulder height (m) — the swing pivot height */

/* Racket length (27" — a standard adult frame). The swing radius (right
 * shoulder → racket string area) is arm_length_m + RACKET_LEN_M; the toss
 * release sits arm_length_m in front of the left shoulder (the hand, no racket). */
#define RACKET_LEN_M   0.6858

/* Forward tilt of the reach (full-extension direction) off vertical, toward the
 * target box. With the swing pivoting at the back-right shoulder, this puts the
 * arc apex up-AND-forward — over where the ball is struck — so a well-timed
 * swing makes contact at full extension (φ≈0). The exit stays flat (driven by
 * the separate swing-through t0d), independent of this tilt. */
#define REACH_TILT_DEG 28.0

/* Groundstroke contact-reference time (s): the incoming ball is anchored to pass
 * through the swing apex at this moment, and the swing is auto-timed to be there
 * then — so a stock groundstroke contacts cleanly at full extension, and a
 * timing offset mistimes it (same as a serve's toss∩arc timing). */
#define GS_T_REF       0.30

static inline double dot3(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static inline double norm3v(const double a[3]) {
    return sqrt(dot3(a, a));
}
static inline void cross3(const double a[3], const double b[3], double o[3]) {
    o[0] = a[1]*b[2] - a[2]*b[1];
    o[1] = a[2]*b[0] - a[0]*b[2];
    o[2] = a[0]*b[1] - a[1]*b[0];
}

/* Integration coefficients for a ball track: drag (kd), Magnus (km) and the spin
 * vector ω = (wx, 0, wz). Same convention as the flight model: ωz = −topspin,
 * ωx = sidespin. A clean toss / spinless ball has wx = wz = 0, which zeroes the
 * Magnus term and recovers the old planar integrator exactly. This is the one
 * physical model behind BOTH the serve toss and the groundstroke incoming ball:
 * a ball with a position, a velocity and a spin, flying under gravity. */
typedef struct { double kd, km, wx, wz; } BallInt;

static BallInt ball_int(const ServeParams *p, double spin_rpm, double sidespin_rpm) {
    BallProps bp = ball_props_for_type(p->strike.ball_type);
    double rho = (p->air_density > 0.0) ? p->air_density : AIR_RHO;
    BallInt bi;
    bi.kd = -bp.cd * rho * bp.area_m2 / (2.0 * bp.mass_kg);   /* quadratic drag */
    bi.km =  bp.radius_m * rho * bp.area_m2 / (2.0 * bp.mass_kg); /* Magnus */
    bi.wz = -spin_rpm     * M_PI / 30.0;   /* rpm → rad/s, ωz = −topspin */
    bi.wx =  sidespin_rpm * M_PI / 30.0;   /* ωx = sidespin */
    return bi;
}

/* Gravity + quadratic drag + Magnus. Drag damps along the velocity vector (each
 * component scaled by the FULL speed |v|) so a spinless ball stays exactly in
 * one vertical plane; the Magnus term km·(ω×v) with ω=(wx,0,wz) curves a
 * spinning one off that plane. */
static void toss_deriv(const double s[6], double ds[6], const BallInt *bi) {
    double vx = s[3], vy = s[4], vz = s[5];
    double v = sqrt(vx*vx + vy*vy + vz*vz);
    ds[0] = vx; ds[1] = vy; ds[2] = vz;
    ds[3] = bi->kd * vx * v + bi->km * (-bi->wz * vy);
    ds[4] = -G_TOSS + bi->kd * vy * v + bi->km * (bi->wz * vx - bi->wx * vz);
    ds[5] = bi->kd * vz * v + bi->km * (bi->wx * vy);
}
static void toss_rk4(double s[6], double dt, const BallInt *bi) {
    double k1[6], k2[6], k3[6], k4[6], tmp[6];
    toss_deriv(s, k1, bi);
    for (int i = 0; i < 6; i++) tmp[i] = s[i] + 0.5*dt*k1[i];
    toss_deriv(tmp, k2, bi);
    for (int i = 0; i < 6; i++) tmp[i] = s[i] + 0.5*dt*k2[i];
    toss_deriv(tmp, k3, bi);
    for (int i = 0; i < 6; i++) tmp[i] = s[i] + dt*k3[i];
    toss_deriv(tmp, k4, bi);
    for (int i = 0; i < 6; i++)
        s[i] += dt*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) / 6.0;
}

/* Toss release state [x,y,z,vx,vy,vz]. A right-hander tosses with the LEFT
 * (off) hand, so the ball is released off to one side and lofted back across
 * to peak over the contact column — visually the toss comes from the opposite
 * side to the racket. Shared by the defaults' timing seed and the simulator
 * so the two never diverge. */
/* The tossing (left, for a righty) shoulder LS in world coords — the toss
 * release pivot. Shared so the simulator, the toss readout and the solver all
 * measure "in front of LS" from the same point. */
static void toss_shoulder_ls(const ServeParams *p, double ls[3]) {
    double side_sign = p->right_handed ? +1.0 : -1.0;
    double a_sh = p->shoulder_to_net_deg * M_PI/180.0;
    double shx = -sin(a_sh), shz = -cos(a_sh);
    double off = -side_sign;
    ls[0] = p->player_x + off * SHOULDER_HALF * shx;
    ls[1] = p->shoulder_height_m;
    ls[2] = p->player_z + off * SHOULDER_HALF * shz;
}

static void toss_release_state(const ServeParams *p, double s[6]) {
    /* The tossing arm is a straight rod of length arm_length_m pivoting at the
     * LEFT shoulder LS, sweeping UP through the toss plane (the vertical plane at
     * the toss bearing). The ball leaves the hand at release_angle_deg along the
     * sweep, measured from forward-horizontal up and over:
     *   arm dir  u(a) = sin a · F − cos a · UP   (a=90° → out front, a=180° → up)
     *   hand vel v(a) = cos a · F + sin a · UP    (tangent, scaled by toss_speed)
     * Earlier release (a→90°) → hand out front, velocity up → ball lands FORWARD.
     * Later release (a→180°) → hand overhead, velocity back → ball goes BEHIND
     * the baseline. This — not a lean — is what places the toss. */
    /* Toss-plane bearing (clockwise from downrange +x): follows the target box
     * so the toss lies in the swing plane (+z deuce, −z ad). */
    double box_side = (p->mode == MODE_SERVE_AD) ? -1.0 : +1.0;
    double bearing = box_side * (90.0 - p->toss_plane_deg) * M_PI/180.0;
    double Fx = cos(bearing), Fz = sin(bearing);          /* forward horizontal in the plane */

    /* LEFT shoulder (off-hand, +z side); the right shoulder (swing pivot) is −z. */
    double ls[3]; toss_shoulder_ls(p, ls);
    double ls_x = ls[0], ls_z = ls[2];

    double L  = p->arm_length_m;                          /* fixed tossing-arm length */
    double a  = p->release_angle_deg * M_PI/180.0;
    double ca = cos(a), sa = sin(a);
    double side = p->toss_side_deg * M_PI/180.0;
    s[0] = ls_x + L * sa * Fx;                            /* hand position on the arm arc */
    s[1] = p->shoulder_height_m + L * (-ca);
    s[2] = ls_z + L * sa * Fz;
    s[3] = p->toss_speed_mps * ca * Fx;                   /* hand tangential velocity */
    s[4] = p->toss_speed_mps * sa;
    s[5] = p->toss_speed_mps * ca * Fz + p->toss_speed_mps * sin(side);
}

/* Integrate the toss alone (no swing) from release to the ground. Reports the
 * apex (highest point) and the toss endpoint TE (where it crosses y=0). Shared
 * by the simulator's readout fields and the inverse solver. */
static void toss_outcome(const ServeParams *p, double apex[3], double te[3]) {
    /* The toss is a smooth projectile, so a coarse step nails the apex + ground
     * crossing to mm — and keeps the inverse solver (which calls this hundreds
     * of times) fast enough for live drag. */
    const double dt = 0.002;
    BallInt bi = ball_int(p, p->toss_spin_rpm, p->toss_sidespin_rpm);
    double s[6]; toss_release_state(p, s);
    double ah = -INFINITY;
    apex[0]=s[0]; apex[1]=s[1]; apex[2]=s[2];
    double prev[3] = { s[0], s[1], s[2] };
    double t = 0.0;
    while (t < TOSS_T_MAX) {
        if (s[1] > ah) { ah = s[1]; apex[0]=s[0]; apex[1]=s[1]; apex[2]=s[2]; }
        if (s[1] <= 0.0 && t > 0.0) {
            /* linear-interpolate the ground crossing between prev and s */
            double f = prev[1] / (prev[1] - s[1] + 1e-12);
            te[0] = prev[0] + f * (s[0]-prev[0]);
            te[1] = 0.0;
            te[2] = prev[2] + f * (s[2]-prev[2]);
            return;
        }
        prev[0]=s[0]; prev[1]=s[1]; prev[2]=s[2];
        toss_rk4(s, dt, &bi);
        t += dt;
    }
    te[0]=s[0]; te[1]=s[1]; te[2]=s[2];
}

/* The swing plane: a VERTICAL plane through the right shoulder RS at the
 * swing-through azimuth (set by swing_plane_deg, leaning toward the target
 * box). Returns RS and the (horizontal) plane normal n. Rotating
 * swing_plane_deg rotates this plane — and the contact, found where the toss
 * crosses it, follows. */
static void swing_plane(const ServeParams *p, double RS[3], double n[3], double *pa_out) {
    double a_sh = p->shoulder_to_net_deg * M_PI/180.0;
    double shx = -sin(a_sh), shz = -cos(a_sh);  /* RIGHT shoulder back (−x) and right (−z) */
    double rk = p->right_handed ? +1.0 : -1.0;
    RS[0] = p->player_x + rk * SHOULDER_HALF * shx;
    RS[1] = p->shoulder_height_m;
    RS[2] = p->player_z + rk * SHOULDER_HALF * shz;
    double box_side = (p->mode == MODE_SERVE_AD) ? -1.0 : +1.0;
    double pa = (box_side * (90.0 - p->swing_plane_deg) + p->plane_az_deg) * M_PI/180.0;
    n[0] = -sin(pa); n[1] = 0.0; n[2] = cos(pa);   /* horizontal, ⟂ the swing azimuth */
    if (pa_out) *pa_out = pa;
}
/* The swing arc's frame, all in the swing plane (the vertical plane through RS
 * at the swing_plane_deg azimuth). Outputs:
 *   RS    — pivot (right shoulder),
 *   a_hat — REACH (full-extension / apex) direction: world-up tilted forward by
 *           REACH_TILT_DEG toward the box azimuth, so the apex (RS + R·a_hat) is
 *           up-AND-forward — over where the ball is struck — and a well-timed
 *           swing contacts at φ≈0 (full extension),
 *   e2    — the in-plane tangent ⟂ a_hat (forward-down at the apex); {a_hat, e2}
 *           is an orthonormal basis of the swing plane, so head(φ) = RS +
 *           R·(cosφ·a_hat + sinφ·e2) is a true circle in that plane,
 *   t0d   — the swing-through (EXIT) direction: azimuth pa, elevation plane_elev.
 *           DECOUPLED from the arc, so the ball leaves flat toward the box no
 *           matter where on the arc contact happens. */
static void swing_basis(const ServeParams *p, double RS[3], double a_hat[3],
                        double e2[3], double t0d[3]) {
    double n[3], pa;
    swing_plane(p, RS, n, &pa);
    double pe = p->plane_elev_deg * M_PI/180.0;
    t0d[0] = cos(pe)*cos(pa); t0d[1] = sin(pe); t0d[2] = cos(pe)*sin(pa);
    /* in-plane orthonormal pair: d = horizontal toward the box azimuth, up */
    double d[3] = { cos(pa), 0.0, sin(pa) }, up[3] = { 0.0, 1.0, 0.0 };
    double rt = p->reach_tilt_deg * M_PI/180.0, cr = cos(rt), sr = sin(rt);
    for (int k = 0; k < 3; k++) {
        a_hat[k] = cr*up[k] + sr*d[k];     /* reach: up, tilted forward toward box */
        e2[k]    = cr*d[k]  - sr*up[k];    /* tangent ⟂ reach, forward-down at apex */
    }
}

/* Seed the swing timing (apex_time). The racket head sweeps a circle of radius
 * R about RS in the swing plane (basis {a_hat, tdir}); the contact is where the
 * toss path comes closest to that circle. We find the toss sample with the
 * smallest distance to the circle, read off the in-plane angle phi of (toss−RS)
 * at that sample, and return apex_time = t_c − phi/omega so the head is exactly
 * at that phi (pointing at the ball) when the ball is there. This works wherever
 * the pivot sits — the contact lands on the forward part of the arc when RS is
 * behind, near the apex when RS is over the ball — so the swing meets the toss. */
static double toss_crossing_time(const ServeParams *p) {
    BallInt bi = ball_int(p, p->toss_spin_rpm, p->toss_sidespin_rpm);
    double RS[3], a_hat[3], e2[3], t0d[3];
    swing_basis(p, RS, a_hat, e2, t0d);
    double R = p->arm_length_m + p->racket_len_m;
    double omega = (p->swing_speed_mps > 1e-3) ? p->swing_speed_mps / R : 1e-3;
    /* swing-plane normal = a_hat × e2 */
    double np[3] = {
        a_hat[1]*e2[2] - a_hat[2]*e2[1],
        a_hat[2]*e2[0] - a_hat[0]*e2[2],
        a_hat[0]*e2[1] - a_hat[1]*e2[0],
    };
    double s[6];
    toss_release_state(p, s);
    double t = 0.0;
    double best_d  = INFINITY, best_t  = 0.0, best_phi  = 0.0;   /* closest overall */
    double best_dd = INFINITY, best_dt = 0.0, best_dphi = 0.0;   /* closest while DESCENDING */
    while (t < TOSS_T_MAX) {
        double v[3] = { s[0]-RS[0], s[1]-RS[1], s[2]-RS[2] };
        double ca = dot3(v, a_hat), ct = dot3(v, e2), oo = dot3(v, np);
        double rip = sqrt(ca*ca + ct*ct);          /* in-plane radius */
        double dcirc = sqrt((rip-R)*(rip-R) + oo*oo);  /* distance to the circle */
        double phi = atan2(ct, ca);
        if (dcirc < best_d) { best_d = dcirc; best_t = t; best_phi = phi; }
        /* A serve is struck after the toss apex (s[4] < 0 → descending), so the
         * racket meets the ball on the UNDERSIDE. Prefer that crossing. */
        if (s[4] < 0.0 && dcirc < best_dd) { best_dd = dcirc; best_dt = t; best_dphi = phi; }
        if (s[1] < 0.0 && t > 0.0) break;
        toss_rk4(s, TOSS_DT, &bi);
        t += TOSS_DT;
    }
    /* For a kick/2nd serve, take the DESCENDING crossing when the toss reaches
     * the circle on its way down (within ~0.25 m of the best approach) — so the
     * racket meets the ball from underneath. Flat serves keep the apex catch. */
    if (p->catch_descending && best_dd < best_d + 0.25) {
        best_t = best_dt; best_phi = best_dphi;
    }
    return best_t - best_phi / omega;
}

double serve_seed_apex_time(const ServeParams *p) {
    /* Groundstroke: the incoming ball is anchored at the swing apex at GS_T_REF,
     * so the racket should reach full extension (φ=0) then. Serve: time it to the
     * toss∩arc crossing. */
    if (p->mode == MODE_GROUNDSTROKE) return GS_T_REF;
    return toss_crossing_time(p);
}

/* ---- Inverse toss solver ----------------------------------------------------
 * The toss is a projectile from the swept tossing arm. Two knobs place it in its
 * plane: toss_speed (how high/far) and release_angle (forward vs back). They
 * couple — speed raises the apex AND lengthens the throw; a later release lowers
 * the apex AND pulls the landing back — but each dominates one target, so a few
 * alternating 1-D bisections converge fast. We hold the bearing (toss_plane)
 * fixed so the toss stays in its contact-compatible plane. */

static double solve_apex_h(const ServeParams *p) {
    double apex[3], te[3]; toss_outcome(p, apex, te);
    return apex[1];
}
static double solve_te_fwd(const ServeParams *p) {
    /* forward distance of the toss endpoint past the server's baseline */
    double apex[3], te[3]; toss_outcome(p, apex, te);
    return te[0] - COURT_X_LO;
}

bool serve_solve_toss(ServeParams *p, double fwd_of_baseline_m, double apex_h_m) {
    /* Clamp targets to a sane envelope. */
    if (apex_h_m < p->shoulder_height_m + 0.10) apex_h_m = p->shoulder_height_m + 0.10;
    if (apex_h_m > 6.0) apex_h_m = 6.0;

    for (int iter = 0; iter < 16; iter++) {
        /* 1) bisect toss_speed to hit the apex height (monotonic increasing). */
        double lo = 2.0, hi = 12.0;
        for (int i = 0; i < 26; i++) {
            double mid = 0.5*(lo+hi);
            p->toss_speed_mps = mid;
            if (solve_apex_h(p) < apex_h_m) lo = mid; else hi = mid;
        }
        p->toss_speed_mps = 0.5*(lo+hi);

        /* 2) bisect release_angle to hit the forward landing. te_fwd DECREASES
         *    as the release angle grows (later release → ball thrown backward),
         *    so search with that orientation. */
        double alo = 80.0, ahi = 150.0;
        for (int i = 0; i < 26; i++) {
            double mid = 0.5*(alo+ahi);
            p->release_angle_deg = mid;
            if (solve_te_fwd(p) > fwd_of_baseline_m) alo = mid; else ahi = mid;
        }
        p->release_angle_deg = 0.5*(alo+ahi);
    }
    double eh = fabs(solve_apex_h(p) - apex_h_m);
    double ef = fabs(solve_te_fwd(p) - fwd_of_baseline_m);
    return eh < 0.05 && ef < 0.05;
}

double serve_solve_toss_side(ServeParams *p, double side_of_ls_m) {
    /* toss_side_deg adds a lateral (+z) velocity; the landing offset from LS is
     * monotonic in it. Bisect over a sane angle range. */
    double lo = -35.0, hi = 35.0;
    double ls[3];
    for (int i = 0; i < 34; i++) {
        double mid = 0.5*(lo+hi);
        p->toss_side_deg = mid;
        double apex[3], te[3]; toss_outcome(p, apex, te);
        toss_shoulder_ls(p, ls);
        if (te[2] - ls[2] < side_of_ls_m) lo = mid; else hi = mid;
    }
    p->toss_side_deg = 0.5*(lo+hi);
    double apex[3], te[3]; toss_outcome(p, apex, te);
    toss_shoulder_ls(p, ls);
    return te[2] - ls[2];
}

void serve_params_defaults(ServeParams *p, ShotMode mode) {
    memset(p, 0, sizeof *p);
    strike_params_defaults(&p->strike, mode);

    p->mode = mode;
    p->right_handed = true;

    /* Shared body geometry. Swing radius (RS → string) = arm + 27" racket. */
    p->arm_length_m      = 0.65;
    p->shoulder_height_m = SHOULDER_H;     /* adult default; lower for a junior */
    p->racket_len_m      = RACKET_LEN_M;   /* 27" adult; ~0.635 m for a 25" junior */
    p->swing_radius_m    = 0.0;            /* legacy, unused */
    p->toss_fwd_deg      = 0.0;            /* legacy */
    p->toss_side_deg     = 0.0;

    if (mode == MODE_GROUNDSTROKE) {
        /* A topspin rally ball. The incoming shot (in_speed/elev/spin from the
         * strike defaults) flies at the player; the swing reaches FORWARD (high
         * reach_tilt → chest-high contact in front, not overhead) and brushes
         * low-to-high. Contact is the arc∩incoming-ball crossing, auto-timed to
         * GS_T_REF. */
        p->shoulder_to_net_deg = 35.0;     /* fairly open / square stance */
        p->player_x = COURT_X_LO - 1.0;    /* a step behind own baseline */
        p->player_z = COURT_Z_MID;
        p->release_height_m = 1.0;
        p->toss_speed_mps   = 0.0;         /* no toss */
        p->release_angle_deg = 90.0;
        p->toss_plane_deg   = 90.0;
        p->reach_tilt_deg   = 72.0;        /* reach forward → contact out front */
        p->contact_height_m = 1.1;         /* legacy, unused by the solver */
        p->swing_speed_mps  = 31.0;        /* groundstroke head speed */
        p->swing_start_speed_mps = 16.0;
        p->plane_elev_deg   = 14.0;        /* swing low-to-high (topspin) */
        p->contact_angle_deg = -9.0;       /* brush up the back → topspin */
        p->plane_az_deg     = 0.0;
        p->swing_plane_deg  = 90.0;        /* drive straight ahead, down the middle */
    } else {
        const bool ad = (mode == MODE_SERVE_AD);
        p->shoulder_to_net_deg = 60.0;
        /* Stance: just behind own baseline, ~3/4 m off the centre mark. */
        p->player_x = COURT_X_LO - 0.3;
        p->player_z = ad ? COURT_Z_MID + 0.75 : COURT_Z_MID - 0.75;
        p->release_height_m = 1.55;
        /* Toss: the tossing arm sweeps up; release_angle places it (early ≈ out
         * front for a flat 1st serve, later ≈ behind the baseline for a kick). */
        p->toss_speed_mps   = 5.75;
        p->release_angle_deg = 102.0;
        p->toss_plane_deg   = ad ? 50.0 : 90.0;
        p->reach_tilt_deg   = REACH_TILT_DEG;   /* contact up-and-out front */
        p->contact_height_m = 2.55;             /* legacy, unused */
        p->swing_speed_mps  = 38.0;
        p->swing_start_speed_mps = 22.0;
        p->plane_elev_deg  = -2.0;              /* flat swing-through into the box */
        p->contact_angle_deg = -7.0;            /* mild topspin */
        p->plane_az_deg    = 0.0;
        p->swing_plane_deg = 80.0;
    }

    /* Time the swing so full extension (φ = 0) lands when the ball is at the
     * apex point (toss∩arc for a serve; the GS_T_REF anchor for a groundstroke),
     * so the stock shot is well-timed and a ±0.3 s offset mistimes it. */
    p->apex_time_s = serve_seed_apex_time(p);

    p->cor_ground = 0.73;   /* hard court-ish; UI overrides */
    p->cof_ground = 0.60;
    p->windspeed = 0.0;
    p->winddirection = 0.0;
    p->air_density = AIR_RHO_SEA;   /* sea level; UI sets it from altitude + temp */
    p->air_temp_c  = AIR_TEMP_REF_C;
    p->air_humidity_pct = 0.0;      /* dry by default → no change vs the old model */
}

/* The ONE ball-track builder behind both shots — a ball with a known state at an
 * anchor sample, flown under gravity + drag + Magnus (BallInt). Integrate forward
 * from the anchor to the end of the window (or the ground) and backward to t=0
 * (RK4 is reversible to its order). Returns the forward extent (sample count).
 *   - serve toss   : anchor = the release state at sample 0 → forward only, stops
 *                    when the toss lands;
 *   - groundstroke : anchor = the incoming ball at the swing apex (GS_T_REF) →
 *                    fills both ways so the ball flies IN to the contact.
 * The contact march then finds where the swept racket meets this track. */
static size_t build_ball_track(const double start[6], int anchor, const BallInt *bi,
                               size_t cap,
                               double *bx, double *by, double *bz,
                               double *vx, double *vy, double *vz) {
    if (anchor < 0) anchor = 0;
    if (anchor >= (int)cap) anchor = (int)cap - 1;
    size_t last = cap;
    double s[6]; memcpy(s, start, sizeof s);
    for (int i = anchor; i < (int)cap; i++) {           /* forward from the anchor */
        bx[i]=s[0]; by[i]=s[1]; bz[i]=s[2]; vx[i]=s[3]; vy[i]=s[4]; vz[i]=s[5];
        if (s[1] < 0.0 && i > anchor) { last = (size_t)i + 1; break; }  /* hit ground */
        toss_rk4(s, TOSS_DT, bi);
    }
    if (anchor > 0) {                                    /* backward to t = 0 */
        memcpy(s, start, sizeof s);
        toss_rk4(s, -TOSS_DT, bi);
        for (int i = anchor - 1; i >= 0; i--) {
            bx[i]=s[0]; by[i]=s[1]; bz[i]=s[2]; vx[i]=s[3]; vy[i]=s[4]; vz[i]=s[5];
            toss_rk4(s, -TOSS_DT, bi);
        }
    }
    return last;
}

/* The incoming ball's world velocity at its anchor — heading at the player (−x),
 * so the strike's az = atan2(−vz,−vx) / elev = asin(vy/|v|) recover the inputs. */
static void incoming_vel(const ServeParams *p, double v[3]) {
    double spd = p->strike.in_speed;
    double el  = p->strike.in_elevation_deg * M_PI / 180.0;
    double az  = p->strike.in_azimuth_deg   * M_PI / 180.0;
    v[0] = -spd * cos(el) * cos(az);
    v[1] =  spd * sin(el);
    v[2] = -spd * cos(el) * sin(az);
}

ServeResult serve_simulate(const ServeParams *p) {
    ServeResult r;
    memset(&r, 0, sizeof r);
    r.miss_distance_m = INFINITY;

    BallProps bp = ball_props_for_type(p->strike.ball_type);

    /* ---- Swing geometry — arc pivots at RS in the swing plane. Computed first
     * because the groundstroke ball track is anchored at the swing apex.
     * head(φ) = RS + R·(cosφ·a_hat + sinφ·e2); full extension (apex = RS +
     * R·a_hat) is at φ = 0. The CONTACT is the arc∩ball-track crossing found
     * below; the strike is driven separately by t0d so the exit is decoupled. */
    double pivot[3], a_hat[3], e2[3], t0d[3];
    swing_basis(p, pivot, a_hat, e2, t0d);
    double swing_radius = p->arm_length_m + p->racket_len_m;
    if (swing_radius < 0.3) swing_radius = 0.3;
    double omega = (p->swing_speed_mps > 1e-3)
                 ? p->swing_speed_mps / swing_radius : 1e-3;
    r.swing_az_deg = atan2(t0d[2], t0d[0]) * 180.0/M_PI;
    r.swing_pivot[0] = pivot[0]; r.swing_pivot[1] = pivot[1]; r.swing_pivot[2] = pivot[2];
    for (int k = 0; k < 3; k++) { r.swing_u[k] = a_hat[k]; r.swing_v[k] = e2[k]; }
    double apexpt[3] = {                       /* full-extension contact point */
        pivot[0] + swing_radius * a_hat[0],
        pivot[1] + swing_radius * a_hat[1],
        pivot[2] + swing_radius * a_hat[2],
    };

    /* ---- Ball track: ONE model for both shots. Build a start state + spin per
     * mode, then build_ball_track flies it (gravity + drag + Magnus). ---- */
    size_t cap = (size_t)(TOSS_T_MAX / TOSS_DT) + 4;
    r.toss_x = malloc(cap * sizeof(double));
    r.toss_y = malloc(cap * sizeof(double));
    r.toss_z = malloc(cap * sizeof(double));
    double *vx = malloc(cap * sizeof(double));
    double *vy = malloc(cap * sizeof(double));
    double *vz = malloc(cap * sizeof(double));

    toss_shoulder_ls(p, r.ls_pos);

    double start[6];
    int anchor;
    BallInt bi;
    if (p->mode == MODE_GROUNDSTROKE) {
        /* IBS = the incoming ball at the swing apex (near contact), flying at the
         * player; spin = the incoming ball's spin. */
        double vin[3]; incoming_vel(p, vin);
        start[0]=apexpt[0]; start[1]=apexpt[1]; start[2]=apexpt[2];
        start[3]=vin[0];    start[4]=vin[1];    start[5]=vin[2];
        anchor = (int)(GS_T_REF / TOSS_DT + 0.5);
        bi = ball_int(p, p->strike.in_spin_rpm, p->strike.in_sidespin_rpm);
        r.toss_az_deg = p->strike.in_azimuth_deg;
    } else {
        /* IBS = the toss release state; spin = the (usually small) toss spin. */
        toss_release_state(p, start);
        anchor = 0;
        bi = ball_int(p, p->toss_spin_rpm, p->toss_sidespin_rpm);
        r.toss_az_deg = (fabs(start[3]) + fabs(start[5]) > 1e-6)
                      ? atan2(start[5], start[3]) * 180.0/M_PI : 0.0;
    }
    size_t n = build_ball_track(start, anchor, &bi, cap,
                                r.toss_x, r.toss_y, r.toss_z, vx, vy, vz);
    r.toss_n = n;

    if (p->mode == MODE_GROUNDSTROKE) {     /* IBE + apex metadata for the views */
        for (int k = 0; k < 3; k++) r.toss_apex[k] = apexpt[k];
        r.toss_te[0] = r.toss_x[n-1]; r.toss_te[1] = r.toss_y[n-1]; r.toss_te[2] = r.toss_z[n-1];
    } else {
        toss_outcome(p, r.toss_apex, r.toss_te);
    }

    /* ---- Racket arc samples across the whole window (for rendering) ---- */
    const size_t RKT_SAMPLES = 64;
    r.rkt_x = malloc(RKT_SAMPLES * sizeof(double));
    r.rkt_y = malloc(RKT_SAMPLES * sizeof(double));
    r.rkt_z = malloc(RKT_SAMPLES * sizeof(double));
    for (size_t i = 0; i < RKT_SAMPLES; i++) {
        double phi = -PHI_MAX_RAD + (2.0*PHI_MAX_RAD) * (double)i/(RKT_SAMPLES-1);
        double c = cos(phi), sn = sin(phi);
        r.rkt_x[i] = pivot[0] + swing_radius * (c*a_hat[0] + sn*e2[0]);
        r.rkt_y[i] = pivot[1] + swing_radius * (c*a_hat[1] + sn*e2[1]);
        r.rkt_z[i] = pivot[2] + swing_radius * (c*a_hat[2] + sn*e2[2]);
    }
    r.rkt_n = RKT_SAMPLES;

    /* ---- Contact march ---- */
    size_t contact_i = 0;
    bool found = false;
    double cap_r = bp.radius_m + HEAD_TOL_M;
    for (size_t i = 0; i < n; i++) {
        double tt = (double)i * TOSS_DT;
        double phi = omega * (tt - p->apex_time_s);
        if (fabs(phi) > PHI_MAX_RAD) continue;      /* racket not in play yet */
        double c = cos(phi), sn = sin(phi);
        double head[3] = {
            pivot[0] + swing_radius * (c*a_hat[0] + sn*e2[0]),
            pivot[1] + swing_radius * (c*a_hat[1] + sn*e2[1]),
            pivot[2] + swing_radius * (c*a_hat[2] + sn*e2[2]),
        };
        double d[3] = { head[0]-r.toss_x[i], head[1]-r.toss_y[i], head[2]-r.toss_z[i] };
        double dist = norm3v(d);
        if (dist < r.miss_distance_m) r.miss_distance_m = dist;
        if (dist <= cap_r) { found = true; contact_i = i; break; }
    }

    if (!found) {
        free(vx); free(vy); free(vz);
        r.contact = false;
        return r;
    }

    /* ---- Contact geometry ---- */
    r.contact = true;
    r.t_contact = (double)contact_i * TOSS_DT;
    r.contact_pos[0] = r.toss_x[contact_i];
    r.contact_pos[1] = r.toss_y[contact_i];
    r.contact_pos[2] = r.toss_z[contact_i];
    double phi_c = omega * (r.t_contact - p->apex_time_s);
    r.phi_contact_deg = phi_c * 180.0/M_PI;

    /* Head speed at contact. The tracked arc is only the second loop; the racket
     * enters it (at SS, φ = −PHI_MAX) already moving at swing_start_speed_mps
     * from the backswing + leg drive, and accelerates to swing_speed_mps at full
     * extension (φ = 0). So a too-early contact connects before the racket is up
     * to speed and the serve is slower, not just mis-angled. */
    double accel = (phi_c + PHI_MAX_RAD) / PHI_MAX_RAD;   /* 0 at SS, 1 at apex */
    if (accel < 0.0) accel = 0.0;
    if (accel > 1.0) accel = 1.0;
    double head_speed = p->swing_start_speed_mps
                      + (p->swing_speed_mps - p->swing_start_speed_mps) * accel;

    /* Swing tangent at contact = head velocity direction = face normal
     * (flat hit). tangent = d/dφ [cos·a_hat + sin·e2] = -sin·a_hat + cos·e2. */
    double cc = cos(phi_c), sc = sin(phi_c);
    double tang[3] = {
        -sc*a_hat[0] + cc*e2[0],
        -sc*a_hat[1] + cc*e2[1],
        -sc*a_hat[2] + cc*e2[2],
    };
    double tn = norm3v(tang);
    if (tn > 1e-9) { tang[0]/=tn; tang[1]/=tn; tang[2]/=tn; }

    /* Ball velocity at contact (slow, mostly downward past the toss apex). */
    double bv[3] = { vx[contact_i], vy[contact_i], vz[contact_i] };
    double bspeed = norm3v(bv);

    free(vx); free(vy); free(vz);

    /* Stash the contact-frame vectors for the renderer: the face points along
     * the desired exit (t0d), the racket head moves along the arc tangent. */
    for (int k = 0; k < 3; k++) {
        r.face_normal[k]  = t0d[k];
        r.racket_vel[k]   = head_speed * tang[k];
        r.incoming_vel[k] = bv[k];
    }

    /* ---- Map onto StrikeParams ---- */
    StrikeParams sp = p->strike;
    sp.contact_x = r.contact_pos[0];
    sp.contact_y = r.contact_pos[1];
    sp.contact_z = r.contact_pos[2];

    /* Incoming ball — note compute_strike rebuilds v_in with x/z negated,
     * so az = atan2(-vz, -vx) reproduces the true toss velocity. */
    if (bspeed > 1e-4) {
        sp.in_speed         = bspeed;
        sp.in_elevation_deg = asin(bv[1]/bspeed) * 180.0/M_PI;
        sp.in_azimuth_deg   = atan2(-bv[2], -bv[0]) * 180.0/M_PI;
    } else {
        sp.in_speed = 0.0; sp.in_elevation_deg = -90.0; sp.in_azimuth_deg = 0.0;
    }
    /* A serve strikes a (nearly) spinless toss; a groundstroke keeps the
     * incoming ball's spin (already copied into sp from p->strike). */
    if (p->mode != MODE_GROUNDSTROKE) {
        sp.in_spin_rpm = 0.0;
        sp.in_sidespin_rpm = 0.0;
    }

    /* Racket head velocity drives the ball along the swing-through direction
     * t0d (a flat hit). The RS-anchored arc is the visual swing; the strike
     * uses t0d so the serve lands in the box. (A face-vs-velocity brush for
     * real kick/slice spin is the next layer.) */
    sp.racket_velocity_override_mps = head_speed;
    sp.swing_path_elev_deg = asin(t0d[1]) * 180.0/M_PI;
    sp.swing_path_az_deg   = atan2(t0d[2], t0d[0]) * 180.0/M_PI;

    /* Face normal = the swing-through t0d, tilted in the vertical plane by the
     * contact angle (the brush): opening the face UP relative to the forward
     * swing path makes the strings climb the back of the ball → topspin; closing
     * it → slice/backspin. The face thus differs from the swing path by
     * contact_angle, and that difference is what compute_strike turns into spin.
     * Encode as ball-frame (lon, lat) so n_r = (cos·cos, -sin, -cos·sin) == face. */
    double pa_face = atan2(t0d[2], t0d[0]) + p->side_contact_angle_deg * M_PI/180.0;
    double fe = (p->plane_elev_deg + p->contact_angle_deg) * M_PI/180.0;
    double fn[3] = { cos(fe)*cos(pa_face), sin(fe), cos(fe)*sin(pa_face) };
    for (int k = 0; k < 3; k++) r.face_normal[k] = fn[k];   /* renderer shows the tilt */
    {
        double lat = asin(-fn[1]);               /* n_r.y = -sin(lat) */
        double clat = cos(lat);
        double lon = (fabs(clat) > 1e-6) ? atan2(-fn[2], fn[0]) : 0.0;
        sp.ball_contact_lon_deg = lon * 180.0/M_PI;
        sp.ball_contact_lat_deg = lat * 180.0/M_PI;
    }

    /* ---- Strike + flight ---- */
    SwingParams cond;
    memset(&cond, 0, sizeof cond);
    cond.mode          = p->mode;
    cond.cor           = p->cor_ground;
    cond.cof           = p->cof_ground;
    cond.windspeed     = p->windspeed;
    cond.winddirection = p->winddirection;
    cond.ball_type     = p->strike.ball_type;

    /* Temperature → ball liveliness: a warm ball is springier off both the
     * strings (more exit pace) and the court (higher bounce); a cold ball is
     * dead. e_a in compute_strike is cor_strings·frame·ball, so scaling
     * cor_strings scales the whole impact; cond.cor warms the ground bounce. */
    double cor_f = ball_cor_temp_factor(p->air_temp_c);
    sp.strings.cor_strings *= cor_f;
    cond.cor *= cor_f;
    if (cond.cor > 0.97) cond.cor = 0.97;

    SwingParams out_swing;
    memset(&out_swing, 0, sizeof out_swing);
    r.strike_out = compute_strike(&sp, &out_swing, &cond);
    out_swing.air_density = p->air_density;   /* thin mountain air → less drag/Magnus */
    r.flight = simulate_swing(&out_swing);

    /* ---- Impact dynamics (vectors) ---- */
    memset(&r.impact, 0, sizeof r.impact);
    if (r.contact) {
        BallProps bp = ball_props_for_type(p->strike.ball_type);
        /* Ball exit velocity in world coords — same convention simulate_swing
         * launches with (power, angle_deg elevation, azimuth_deg). */
        double th = out_swing.angle_deg   * M_PI/180.0;
        double az = out_swing.azimuth_deg * M_PI/180.0;
        double sp = out_swing.power;
        double vout[3] = { sp*cos(th)*cos(az), sp*sin(th), sp*cos(th)*sin(az) };
        double dwell_s = r.strike_out.dwell_ms * 1e-3;
        if (dwell_s < 1e-4) dwell_s = 1e-4;
        for (int k = 0; k < 3; k++) {
            double J = bp.mass_kg * (vout[k] - r.incoming_vel[k]);   /* impulse on the ball */
            r.impact.ball_impulse[k] = J;
            r.impact.ball_force[k]   = J / dwell_s;
            r.impact.racket_force[k] = -J / dwell_s;                 /* reaction on the racket */
        }
        /* Lever from the grip to the contact: along the head direction, length
         * = racket length (= contact radius − arm). Torque the arm must resist. */
        double headdir[3] = { r.contact_pos[0]-r.swing_pivot[0],
                              r.contact_pos[1]-r.swing_pivot[1],
                              r.contact_pos[2]-r.swing_pivot[2] };
        double R = norm3v(headdir);
        if (R > 1e-6) for (int k=0;k<3;k++) headdir[k] /= R;
        double lever = R - p->arm_length_m;   /* grip → contact */
        if (lever < 0.0) lever = 0.0;
        double rvec[3] = { lever*headdir[0], lever*headdir[1], lever*headdir[2] };
        cross3(rvec, r.impact.racket_force, r.impact.arm_torque);
        r.impact.ball_force_mag  = norm3v(r.impact.ball_force);
        r.impact.arm_torque_mag  = norm3v(r.impact.arm_torque);
        r.impact.racket_eff_kg   = r.strike_out.m_eff_kg;
    }

    /* No-spin twin (same launch, zero spin) — the no-Magnus reference path. */
    SwingParams nospin = out_swing;
    nospin.spin_rpm = 0.0;
    nospin.sidespin_rpm = 0.0;
    r.flight_nospin = simulate_swing(&nospin);

    return r;
}

void serve_result_free(ServeResult *r) {
    if (!r) return;
    free(r->toss_x); free(r->toss_y); free(r->toss_z);
    free(r->rkt_x);  free(r->rkt_y);  free(r->rkt_z);
    r->toss_x = r->toss_y = r->toss_z = NULL;
    r->rkt_x = r->rkt_y = r->rkt_z = NULL;
    trajectory_free(&r->flight);
    trajectory_free(&r->flight_nospin);
}

void serve_skeleton(const ServeParams *p, const ServeResult *r, PoseJoints *out) {
    /* Bone lengths from the player's height (acromion ≈ 0.818·stature), racket
     * matched to the shot's. */
    double height = (p->shoulder_height_m > 0.5) ? p->shoulder_height_m / 0.818 : 1.86;
    Skeleton skel;
    skeleton_from_height(&skel, height);

    /* Match the figure's racket arm to the shot's geometry exactly, so the
     * contact point (radius arm + racket from RS) is reachable and the IK lands
     * the tip on it instead of clamping short. */
    if (p->racket_len_m > 0.3) skel.racket = p->racket_len_m;
    if (p->arm_length_m  > 0.3) {
        double arm = p->arm_length_m;
        skel.upper_arm = arm * (0.16 / 0.34);   /* keep the upper:fore ratio */
        skel.forearm   = arm * (0.18 / 0.34);
    }

    /* Shoulder midpoint + LS→RS span from the swing geometry. */
    double smx = 0.5 * (r->ls_pos[0] + r->swing_pivot[0]);
    double smy = 0.5 * (r->ls_pos[1] + r->swing_pivot[1]);
    double smz = 0.5 * (r->ls_pos[2] + r->swing_pivot[2]);
    double dx = r->swing_pivot[0] - r->ls_pos[0];
    double dy = r->swing_pivot[1] - r->ls_pos[1];
    double dz = r->swing_pivot[2] - r->ls_pos[2];
    double shspan = sqrt(dx*dx + dy*dy + dz*dz);
    if (shspan > 0.10) skel.shoulder_half = 0.5 * shspan;   /* land shoulders on LS/RS */

    Pose pose;
    memset(&pose, 0, sizeof pose);
    pose.spine_pitch_deg = -6.0;   /* slight back arch at full extension */
    pose.spine_roll_deg  =  0.0;

    /* Place the pelvis so FK's spine_top (the shoulder midpoint) lands on the
     * shot's shoulder midpoint, accounting for the spine tilt. */
    double pitch = pose.spine_pitch_deg * M_PI / 180.0;
    double sdir[3] = { -sin(pitch), cos(pitch), 0.0 };
    pose.com[0] = smx - skel.torso * sdir[0];
    pose.com[1] = smy - skel.torso * sdir[1];
    pose.com[2] = smz - skel.torso * sdir[2];

    /* Align FK's shoulder line with LS→RS: yaw = atan2(−dz, dx). */
    double yaw = atan2(-dz, dx) * 180.0 / M_PI;
    pose.shoulder_yaw_deg = yaw;
    pose.hip_yaw_deg      = yaw;

    pose.knee_flex_l_deg  =  8.0;  pose.knee_flex_r_deg  =  8.0;
    pose.ankle_flex_l_deg = 15.0;  pose.ankle_flex_r_deg = 15.0;
    pose.l_arm_alpha_deg  = 150.0; pose.l_arm_beta_deg   = 30.0;
    pose.l_elbow_flex_deg =  15.0; pose.l_wrist_flex_deg =  0.0;
    pose.r_arm_alpha_deg  = 150.0; pose.r_arm_beta_deg   = 10.0;
    pose.r_elbow_flex_deg =  30.0; pose.r_wrist_flex_deg =  0.0;

    PoseRig rig;
    rig.prep = pose; rig.contact = pose; rig.skel = skel;
    rig.scrub = 1.0; rig.swing_duration_s = 0.1;

    pose_rig_ik_racket(&rig, &pose, r->contact_pos);
    pose_fk(&pose, &skel, out);

    /* Pin the racket tip to the contact: the analytic 2-link IK is loose for an
     * overhead reach, and the racket physically meets the ball there. */
    out->r_racket_tip[0] = r->contact_pos[0];
    out->r_racket_tip[1] = r->contact_pos[1];
    out->r_racket_tip[2] = r->contact_pos[2];
}
