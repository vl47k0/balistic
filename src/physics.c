#define _USE_MATH_DEFINES
#include "physics.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define STATE_N 12
#define DT RAKIJA_DT
#define T_MAX 10.0
#define MAX_BOUNCES 4

static const double G = 9.81;
static const double P = AIR_RHO_SEA; /* air density at sea level (kg/m^3) */

double air_density(double alt_m, double temp_c) {
    /* Pressure ratio from the ISA troposphere (pressure exponent 5.25588), then
     * the ideal-gas temperature scaling relative to the reference day — so the
     * user's stated court temperature (not a standard lapse) sets the density. */
    double base = 1.0 - 2.25577e-5 * (alt_m > 0.0 ? alt_m : 0.0);
    if (base < 0.05) base = 0.05;
    double press_ratio = pow(base, 5.25588);
    double temp_ratio = (AIR_TEMP_REF_C + 273.15) / (temp_c + 273.15);
    return AIR_RHO_SEA * press_ratio * temp_ratio;
}

double air_density_at_altitude_m(double alt_m) {
    return air_density(alt_m, AIR_TEMP_REF_C);
}

double altitude_from_density(double density, double temp_c) {
    double temp_ratio = (AIR_TEMP_REF_C + 273.15) / (temp_c + 273.15);
    double press = density / (AIR_RHO_SEA * temp_ratio);   /* = (1−2.256e-5·h)^5.25588 */
    if (press <= 0.0) return 4000.0;
    double base = pow(press, 1.0 / 5.25588);
    double h = (1.0 - base) / 2.25577e-5;
    if (h < 0.0) h = 0.0;
    if (h > 4000.0) h = 4000.0;
    return h;
}

double ball_cor_temp_factor(double temp_c) {
    /* ~0.2%/°C on the restitution — a tennis ball is noticeably deader cold and
     * livelier hot. Clamped so extremes stay physical. */
    double f = 1.0 + 0.0020 * (temp_c - AIR_TEMP_REF_C);
    if (f < 0.85) f = 0.85;
    if (f > 1.10) f = 1.10;
    return f;
}

const char *verdict_str(Verdict v) {
    switch (v) {
        case VERDICT_IN:        return "IN";
        case VERDICT_NET:       return "NET";
        case VERDICT_SHORT:     return "SHORT";
        case VERDICT_LONG:      return "LONG";
        case VERDICT_WIDE:      return "WIDE";
        case VERDICT_NO_BOUNCE: return "NO BOUNCE";
    }
    return "?";
}

const char *ball_type_str(BallType t) {
    switch (t) {
        case BALL_ITF_TYPE1:    return "ITF Type 1 (fast)";
        case BALL_ITF_TYPE2:    return "ITF Type 2 (regular)";
        case BALL_ITF_TYPE3:    return "ITF Type 3 (slow / high altitude)";
        case BALL_PRESSURELESS: return "Pressureless";
        case BALL_GREEN:        return "Green (Stage 1 junior)";
    }
    return "?";
}

BallProps ball_props_for_type(BallType t) {
    /* ITF Type 2 (regular): mass 56-59.4 g, diameter 65.41-68.58 mm (radius ~33.5 mm).
     * Type 1 hard: same dims, harder felt; we model marginally higher COR.
     * Type 3 slow: ~6% larger diameter (radius ~35.5 mm). Higher drag area.
     * Pressureless: ~62 g, lower COR. */
    BallProps bp = {0};
    switch (t) {
        case BALL_ITF_TYPE1:
            bp.mass_kg = 0.0577; bp.radius_m = 0.0335;
            bp.cd = 0.50; bp.cor_with_strings = 0.56; bp.friction_mult = 1.05;
            break;
        case BALL_ITF_TYPE2:
            bp.mass_kg = 0.0577; bp.radius_m = 0.0335;
            bp.cd = 0.50; bp.cor_with_strings = 0.53; bp.friction_mult = 1.00;
            break;
        case BALL_ITF_TYPE3:
            bp.mass_kg = 0.0577; bp.radius_m = 0.0355;
            bp.cd = 0.52; bp.cor_with_strings = 0.51; bp.friction_mult = 0.95;
            break;
        case BALL_PRESSURELESS:
            bp.mass_kg = 0.0620; bp.radius_m = 0.0335;
            bp.cd = 0.50; bp.cor_with_strings = 0.48; bp.friction_mult = 0.95;
            break;
        case BALL_GREEN:
            /* Stage 1 green: full-ish size, ~25% lower compression (lower COR off
             * the strings + the ground) and a draggier flight so it carries much
             * shorter — the junior ball that lets a fast serve land in. */
            bp.mass_kg = 0.0560; bp.radius_m = 0.0340;
            bp.cd = 0.62; bp.cor_with_strings = 0.46; bp.friction_mult = 1.05;
            break;
    }
    bp.area_m2 = M_PI * bp.radius_m * bp.radius_m;
    return bp;
}

void swing_params_set_mode_defaults(SwingParams *p, ShotMode mode) {
    p->mode = mode;
    p->air_density = 0.0;   /* 0 = sea-level default (P); consumers at altitude
                             * override after this. Initialised here so callers
                             * that don't set it (rakija) get deterministic air. */
    /* Player's baseline sits at x = COURT_X_LO; groundstroke contact
     * ~2.65 m behind it, serve contact ~0.5 m forward of it. The
     * z axis is centred on COURT_Z_MID; ±1.5 m off centre marks the
     * usual deuce / ad serve positions. */
    switch (mode) {
        case MODE_GROUNDSTROKE:
            p->start_x = COURT_X_LO - 2.65;
            p->start_y = 0.5;
            p->start_z = COURT_Z_MID;
            p->azimuth_deg = 0.0;
            break;
        case MODE_SERVE_DEUCE:
            p->start_x = COURT_X_LO + 0.5;
            p->start_y = 2.44;
            p->start_z = COURT_Z_MID - 1.5;
            p->azimuth_deg = 8.0;
            break;
        case MODE_SERVE_AD:
            p->start_x = COURT_X_LO + 0.5;
            p->start_y = 2.44;
            p->start_z = COURT_Z_MID + 1.5;
            p->azimuth_deg = -8.0;
            break;
    }
}

static void target_box(ShotMode mode,
                       double *xmin, double *xmax, double *zmin, double *zmax) {
    /* Target boxes sit on the opponent's side of the net (x ≥ NET_X). */
    switch (mode) {
        case MODE_GROUNDSTROKE:
            *xmin = NET_X;        *xmax = COURT_X_HI;
            *zmin = COURT_Z_W_LO; *zmax = COURT_Z_W_HI;
            return;
        case MODE_SERVE_DEUCE:
            *xmin = NET_X;        *xmax = SERVICE_X2;
            *zmin = COURT_Z_MID;  *zmax = COURT_Z_W_HI;
            return;
        case MODE_SERVE_AD:
            *xmin = NET_X;        *xmax = SERVICE_X2;
            *zmin = COURT_Z_W_LO; *zmax = COURT_Z_MID;
            return;
    }
}

static inline double v3dot(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static inline double v3norm(const double a[3]) {
    return sqrt(v3dot(a, a));
}
static inline void v3cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* Generalized tnsbounce-style impact. v_in/w_in are updated in place. */
void apply_impact(double v_in[3], double w_in[3],
                  const double v_surf[3], const double n_surf[3],
                  double cor, double cof, double ball_r) {
    /* Relative velocity */
    double vr[3] = { v_in[0] - v_surf[0],
                     v_in[1] - v_surf[1],
                     v_in[2] - v_surf[2] };

    /* Decompose into surface-normal (VV) and tangential (VH) components */
    double vn_dot = v3dot(vr, n_surf);
    double VV[3] = { vn_dot * n_surf[0], vn_dot * n_surf[1], vn_dot * n_surf[2] };
    double VH[3] = { vr[0] - VV[0], vr[1] - VV[1], vr[2] - VV[2] };
    double VH_mag = v3norm(VH);

    /* Post-impact normal velocity: reflected and scaled by cor */
    double VVo[3] = { -cor * VV[0], -cor * VV[1], -cor * VV[2] };

    /* Friction loss on tangential velocity */
    double dV_mag = (1.0 + cor) * fabs(vn_dot);
    double factor = 1.0;
    if (VH_mag > 1e-6) {
        factor = 1.0 - cof * dV_mag / VH_mag;
        if (factor < 0.0) factor = 0.0;
    }

    /* Spin contribution to tangential velocity: cross((2R/5) * N, W) */
    double rN[3] = { (2.0/5.0) * ball_r * n_surf[0],
                     (2.0/5.0) * ball_r * n_surf[1],
                     (2.0/5.0) * ball_r * n_surf[2] };
    double cross_term[3];
    v3cross(rN, w_in, cross_term);

    double VHo[3] = { factor * VH[0] - cross_term[0],
                      factor * VH[1] - cross_term[1],
                      factor * VH[2] - cross_term[2] };

    /* New absolute velocity = surface velocity + relative (VHo + VVo) */
    v_in[0] = v_surf[0] + VHo[0] + VVo[0];
    v_in[1] = v_surf[1] + VHo[1] + VVo[1];
    v_in[2] = v_surf[2] + VHo[2] + VVo[2];

    /* End-of-dwell spin. The classic tnsbounce assumption is that
     * the ball reaches pure rolling during contact and writes ω =
     * (N × V_post) / R outright — but for tennis-ball bounces the
     * contact dwell (a few ms) is too short to fully arrest slip,
     * and the pure-rolling formula overstates the spin transfer.
     * Empirically the post-bounce spin lands somewhere between the
     * pre-bounce value and the rolling value; blend the two so the
     * second flight isn't crushed by an artificially huge Magnus
     * force.
     *
     * The blend scales with cof so high-friction surfaces dissipate
     * more spin per bounce than low-friction ones — clay grabs the
     * ball's spin much harder than grass, and a serve that bounces
     * twice on clay should land the second bounce with noticeably
     * less spin than the same shot on grass. The previous fixed
     * 0.15 gave 0.85^N decay regardless of surface; the cof-scaled
     * blend lands roughly:
     *
     *   grass  (cof≈0.48) → 0.29   → 0.71 retention per bounce
     *   hard   (cof≈0.58) → 0.32   → 0.68 retention per bounce
     *   clay   (cof≈0.85) → 0.41   → 0.60 retention per bounce
     *
     * matching Cross 2002's hard-court measurement (~30 % loss) and
     * the qualitative "clay kills spin more" intuition. */
    double nR[3] = { n_surf[0] / ball_r, n_surf[1] / ball_r, n_surf[2] / ball_r };
    double w_rolling[3];
    v3cross(nR, VHo, w_rolling);
    double roll_blend = 0.15 + 0.30 * cof;
    if (roll_blend < 0.0) roll_blend = 0.0;
    if (roll_blend > 0.6) roll_blend = 0.6;
    w_in[0] = (1.0 - roll_blend) * w_in[0] + roll_blend * w_rolling[0];
    w_in[1] = (1.0 - roll_blend) * w_in[1] + roll_blend * w_rolling[1];
    w_in[2] = (1.0 - roll_blend) * w_in[2] + roll_blend * w_rolling[2];
}

/* state layout: [x, y, z, Vx, Vy, Vz, Vwx, Vwy, Vwz, wx, wy, wz] */
typedef struct {
    double kd; /* -cd*p*A/(2m) */
    double km; /*  r*p*A/(2m) */
    double ball_r;
} FlightConsts;

/* Wind profile reference height (m). SwingParams.windspeed is the
 * speed at this altitude; flight integrator scales it by the
 * power-law profile below. 10 m is the standard meteorological
 * measurement height. */
#define WIND_REF_HEIGHT 10.0
/* Power-law exponent. α ≈ 0.2 fits an open stadium with seating;
 * α=0 would mean wind constant with altitude (the old behaviour).
 * A serve at toss height (~2.5 m) sees ~76% of reference; a lob
 * apex (~10 m) sees ~100%; a forehand contact (~1 m) sees ~63%. */
#define WIND_ALPHA 0.20

static inline double wind_scale(double y) {
    if (y <= 0.0) return 0.0;
    double r = y / WIND_REF_HEIGHT;
    /* For y > ref height the wind keeps growing modestly; cap at
     * 1.5× so a stray lob above 50 m doesn't trip an unrealistic
     * gust. */
    double s = pow(r, WIND_ALPHA);
    if (s > 1.5) s = 1.5;
    return s;
}

static void deriv(const double s[STATE_N], double ds[STATE_N], const FlightConsts *fc) {
    /* Wind components (s[6] / s[8]) are stored at the reference
     * height; scale them by the altitude profile so serves and
     * groundstrokes feel different wind loadings. */
    double scale = wind_scale(s[1]);
    const double Vwx_y = s[6] * scale;
    const double Vwz_y = s[8] * scale;

    const double Vrx = s[3] - Vwx_y;
    const double Vry = s[4] - s[7];
    const double Vrz = s[5] - Vwz_y;
    const double wx = s[9], wy = s[10], wz = s[11];

    ds[0] = s[3];
    ds[1] = s[4];
    ds[2] = s[5];
    ds[3] = fc->kd * Vrx * fabs(Vrx) + fc->km * (wy * Vrz - wz * Vry);
    ds[4] = -G + fc->kd * Vry * fabs(Vry) + fc->km * (wz * Vrx - wx * Vrz);
    ds[5] = fc->kd * Vrz * fabs(Vrz) + fc->km * (wx * Vry - wy * Vrx);
    ds[6] = ds[7] = ds[8] = 0.0;
    ds[9] = ds[10] = ds[11] = 0.0;
}

static void rk4_step(double s[STATE_N], double dt, const FlightConsts *fc) {
    double k1[STATE_N], k2[STATE_N], k3[STATE_N], k4[STATE_N], tmp[STATE_N];
    deriv(s, k1, fc);
    for (int i = 0; i < STATE_N; i++) tmp[i] = s[i] + 0.5 * dt * k1[i];
    deriv(tmp, k2, fc);
    for (int i = 0; i < STATE_N; i++) tmp[i] = s[i] + 0.5 * dt * k2[i];
    deriv(tmp, k3, fc);
    for (int i = 0; i < STATE_N; i++) tmp[i] = s[i] + dt * k3[i];
    deriv(tmp, k4, fc);
    for (int i = 0; i < STATE_N; i++)
        s[i] += dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
}

void trajectory_free(Trajectory *tr) {
    if (!tr) return;
    free(tr->x); free(tr->y); free(tr->z);
    tr->x = tr->y = tr->z = NULL;
    tr->n = 0;
}

Trajectory simulate_swing(const SwingParams *p) {
    BallProps bp = ball_props_for_type(p->ball_type);
    const double rho = (p->air_density > 0.0) ? p->air_density : P;  /* thin air at altitude */
    FlightConsts fc = {
        .kd = -bp.cd * rho * bp.area_m2 / (2.0 * bp.mass_kg),
        .km =  bp.radius_m * rho * bp.area_m2 / (2.0 * bp.mass_kg),
        .ball_r = bp.radius_m,
    };
    const double n_ground[3] = { 0.0, 1.0, 0.0 };

    const double theta = p->angle_deg   * M_PI / 180.0;
    const double phi   = p->azimuth_deg * M_PI / 180.0;
    const double Vx = p->power * cos(theta) * cos(phi);
    const double Vy = p->power * sin(theta);
    const double Vz = p->power * cos(theta) * sin(phi);

    const double Vwx = p->windspeed * cos(p->winddirection);
    const double Vwz = p->windspeed * sin(p->winddirection);

    /* Spin-axis convention:
     *   ω_z = -spin_rpm     — topspin axis (rotation about world +z),
     *                         + spin_rpm = ball spins forward in flight
     *   ω_x =  sidespin_rpm — slice axis (rotation about world +x), the
     *                         direction of ball motion. Positive sidespin
     *                         is a right-handed slice / kick:
     *                         curves left of intended line in flight
     *                         (where it engages V_y) and kicks left on
     *                         bounce (where N × ω produces tangential
     *                         contact velocity in -z).
     *
     * Sidespin used to live on ω_y, which is the pure Frisbee axis —
     * combines with V_x to give zero Magnus and combines with the
     * vertical surface normal to give zero bounce kick, so it did
     * nothing useful for the serves it was meant to model. */
    const double wz = -p->spin_rpm     * M_PI / 30.0;
    const double wx =  p->sidespin_rpm * M_PI / 30.0;

    double s[STATE_N] = {
        p->start_x, p->start_y, p->start_z,
        Vx, Vy, Vz,
        Vwx, 0.0, Vwz,
        wx, 0.0, wz,
    };

    double tx_min = NET_X,        tx_max = COURT_X_HI;
    double tz_min = COURT_Z_W_LO, tz_max = COURT_Z_W_HI;
    target_box(p->mode, &tx_min, &tx_max, &tz_min, &tz_max);

    const size_t cap = (size_t)(T_MAX / DT) + MAX_BOUNCES + 4;
    Trajectory tr = {
        .x = malloc(cap * sizeof(double)),
        .y = malloc(cap * sizeof(double)),
        .z = malloc(cap * sizeof(double)),
        .n = 0,
        .max_height = s[1],
        .landing_dist = 0.0,
        .landing_x = NAN, .landing_z = NAN,
        .net_clearance = NAN,
        .bounce_height_m = 0.0,
        .bounce_apex_x = NAN, .bounce_apex_z = NAN,
        .n_bounces = 0,
        .hit_net = false,
        .verdict = VERDICT_NO_BOUNCE,
        .target_x_min = tx_min, .target_x_max = tx_max,
        .target_z_min = tz_min, .target_z_max = tz_max,
    };

    tr.x[tr.n] = s[0]; tr.y[tr.n] = s[1]; tr.z[tr.n] = s[2]; tr.n++;

    double t = 0.0;
    while (t < T_MAX) {
        double prev[STATE_N];
        memcpy(prev, s, sizeof prev);
        rk4_step(s, DT, &fc);
        t += DT;

        if (!tr.hit_net && (prev[0] - NET_X) * (s[0] - NET_X) < 0.0) {
            const double f = (NET_X - prev[0]) / (s[0] - prev[0]);
            const double y_at_net = prev[1] + f * (s[1] - prev[1]);
            const double z_at_net = prev[2] + f * (s[2] - prev[2]);
            if (isnan(tr.net_clearance)) tr.net_clearance = y_at_net - NET_HEIGHT;
            if (y_at_net < NET_HEIGHT
                && z_at_net >= COURT_Z_DBL_LO
                && z_at_net <= COURT_Z_DBL_HI) {
                for (int i = 0; i < STATE_N; i++)
                    s[i] = prev[i] + f * (s[i] - prev[i]);
                tr.x[tr.n] = s[0]; tr.y[tr.n] = s[1]; tr.z[tr.n] = s[2]; tr.n++;
                tr.hit_net = true;
                tr.verdict = VERDICT_NET;
                break;
            }
        }

        if (s[1] < 0.0 && prev[1] >= 0.0) {
            const double f = prev[1] / (prev[1] - s[1]);
            for (int i = 0; i < STATE_N; i++)
                s[i] = prev[i] + f * (s[i] - prev[i]);
            s[1] = 0.0;

            if (tr.n_bounces == 0) {
                tr.landing_x = s[0];
                tr.landing_z = s[2];
                tr.landing_dist = sqrt(s[0] * s[0] + s[2] * s[2]);
            }
            tr.x[tr.n] = s[0]; tr.y[tr.n] = 0.0; tr.z[tr.n] = s[2]; tr.n++;

            /* apply_impact in place on the velocity / spin sub-vectors */
            double v[3] = { s[3], s[4], s[5] };
            double w[3] = { s[9], s[10], s[11] };
            const double v0[3] = {0,0,0};
            apply_impact(v, w, v0, n_ground, p->cor, p->cof, bp.radius_m);
            s[3] = v[0]; s[4] = v[1]; s[5] = v[2];
            s[9] = w[0]; s[10] = w[1]; s[11] = w[2];

            tr.n_bounces++;
            if (tr.n_bounces >= MAX_BOUNCES) break;
            continue;
        }

        tr.x[tr.n] = s[0]; tr.y[tr.n] = s[1]; tr.z[tr.n] = s[2]; tr.n++;
        if (s[1] > tr.max_height) tr.max_height = s[1];

        /* Peak of the FIRST post-bounce arc — how high the serve kicks up off
         * the court. Only the arc between bounce 1 and bounce 2 counts. */
        if (tr.n_bounces == 1 && s[1] > tr.bounce_height_m) {
            tr.bounce_height_m = s[1];
            tr.bounce_apex_x = s[0];
            tr.bounce_apex_z = s[2];
        }

        const double speed_sq = s[3]*s[3] + s[4]*s[4] + s[5]*s[5];
        if (tr.n_bounces > 0 && speed_sq < 0.25 && s[1] < 0.05) break;
    }

    if (!tr.hit_net) {
        if (tr.n_bounces == 0) {
            tr.verdict = VERDICT_NO_BOUNCE;
        } else if (tr.landing_x < tx_min) {
            tr.verdict = VERDICT_SHORT;
        } else if (tr.landing_x > tx_max) {
            tr.verdict = VERDICT_LONG;
        } else if (tr.landing_z < tz_min || tr.landing_z > tz_max) {
            tr.verdict = VERDICT_WIDE;
        } else {
            tr.verdict = VERDICT_IN;
        }
    }
    return tr;
}
