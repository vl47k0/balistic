#define _USE_MATH_DEFINES
#include "strike.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Spin saturation (racket impact). The sliding-friction model lets a glancing
 * brush convert unbounded tangential slip into unbounded spin (it assumes the
 * contact always reaches rolling). Real strings can't grip for the full bite in
 * the ~4 ms dwell, so spin has a physical ceiling. We soft-saturate the
 * racket-imparted angular rate: rates below SPIN_KNEE pass through untouched
 * (realistic serves/strokes unaffected), and beyond it the total spin asymptotes
 * toward SPIN_MAX via tanh. Tuned to pro-measured serve spins (~2.5–5k rpm). */
#define SPIN_KNEE_RADS  366.5   /* ~3500 rpm — below this, no change */
#define SPIN_MAX_RADS   576.0   /* ~5500 rpm — soft asymptote */

const char *string_material_str(StringMaterial m) {
    switch (m) {
        case STRING_POLY:   return "Polyester";
        case STRING_NYLON:  return "Nylon / multi";
        case STRING_GUT:    return "Natural gut";
    }
    return "?";
}

/* Per-direction coefficient contributions. */
static double material_mu_base(StringMaterial m) {
    switch (m) {
        case STRING_POLY:  return 0.20;
        case STRING_NYLON: return 0.15;
        case STRING_GUT:   return 0.13;
    }
    return 0.18;
}
static double material_cor_base(StringMaterial m) {
    switch (m) {
        case STRING_POLY:  return 0.84;
        case STRING_NYLON: return 0.88;
        case STRING_GUT:   return 0.92;
    }
    return 0.86;
}
static double material_se_base(StringMaterial m) {
    switch (m) {
        case STRING_POLY:  return 1.10;
        case STRING_NYLON: return 0.90;
        case STRING_GUT:   return 0.85;
    }
    return 1.00;
}
static double shape_mu_mult(StringShape s) {
    switch (s) {
        case STRING_ROUND:   return 1.00;
        case STRING_SHAPED:  return 1.15;
        case STRING_TWISTED: return 1.22;
    }
    return 1.00;
}
static double shape_se_mult(StringShape s) {
    switch (s) {
        case STRING_ROUND:   return 1.00;
        case STRING_SHAPED:  return 1.10;
        case STRING_TWISTED: return 1.18;
    }
    return 1.00;
}
const char *string_shape_str(StringShape s) {
    switch (s) {
        case STRING_ROUND:   return "Round";
        case STRING_SHAPED:  return "Shaped";
        case STRING_TWISTED: return "Twisted / octagonal";
    }
    return "?";
}
const char *string_pattern_str(StringPattern p) {
    switch (p) {
        case STRING_OPEN_16x19:  return "Open 16x19";
        case STRING_DENSE_18x20: return "Dense 18x20";
    }
    return "?";
}

/* Approximate centre-to-centre mains spacing for a typical 100 sq in
 * head, in metres. 16 mains across ~22 cm of head width gives ~14
 * mm; 18 mains gives ~12.5 mm. Doesn't model variable spacing
 * (Yonex's "isometric" pattern spaces wider near the centre than at
 * the edges), but that's a refinement beyond the scope of the
 * derived display. */
static double mains_spacing_m(StringPattern p) {
    switch (p) {
        case STRING_OPEN_16x19:  return 0.0140;
        case STRING_DENSE_18x20: return 0.0125;
    }
    return 0.0140;
}

/* Combine mains + crosses into single string-bed coefficients.
 *
 * Weights reflect what each direction actually does during the impact:
 *   - μ (Coulomb friction): mains dominate — they're the strings the ball
 *     drags across as it slides; crosses are underneath. ~75/25.
 *   - Spin efficiency (snap-back): mains dominate even more — the lateral
 *     snap-back of the deflected mains is what gives polys their spin
 *     reputation. ~80/20.
 *   - COR (string-bed power): both contribute, mains slightly more because
 *     they're the longer, more elastic direction. ~60/40.
 *
 * Tension affects each direction's contribution: lower tension on a direction
 * means more deflection AND more snap-back FROM THAT DIRECTION. */
void strings_recompute(Strings *s) {
    const double w_mu_m  = 0.75, w_mu_c  = 0.25;
    const double w_se_m  = 0.80, w_se_c  = 0.20;
    const double w_cor_m = 0.60, w_cor_c = 0.40;

    /* Per-direction tension multipliers (centred on 55 lb). */
    double tm_m = 1.0 + 0.010 * (55.0 - s->mains.tension_lbs);
    double tm_c = 1.0 + 0.010 * (55.0 - s->crosses.tension_lbs);
    if (tm_m < 0.7) tm_m = 0.7;
    if (tm_m > 1.3) tm_m = 1.3;
    if (tm_c < 0.7) tm_c = 0.7;
    if (tm_c > 1.3) tm_c = 1.3;

    /* Per-direction μ contribution (material × shape × tension). */
    double mu_m = material_mu_base(s->mains.material)
                * shape_mu_mult(s->mains.shape) * tm_m;
    double mu_c = material_mu_base(s->crosses.material)
                * shape_mu_mult(s->crosses.shape) * tm_c;
    double pattern_mu_mult = (s->pattern == STRING_OPEN_16x19) ? 1.05 : 0.95;
    s->mu_strings = (w_mu_m * mu_m + w_mu_c * mu_c) * pattern_mu_mult;

    /* Per-direction COR contribution. Tension on each direction stiffens its
     * own contribution slightly (higher tension = a bit more COR for polys). */
    double cor_m = material_cor_base(s->mains.material)
                 + 0.002 * (55.0 - s->mains.tension_lbs);
    double cor_c = material_cor_base(s->crosses.material)
                 + 0.002 * (55.0 - s->crosses.tension_lbs);
    double cor = w_cor_m * cor_m + w_cor_c * cor_c;
    if (cor < 0.78) cor = 0.78;
    if (cor > 0.95) cor = 0.95;
    s->cor_strings = cor;

    /* Per-direction spin-efficiency contribution (snap-back). */
    double se_m = material_se_base(s->mains.material)
                * shape_se_mult(s->mains.shape) * tm_m;
    double se_c = material_se_base(s->crosses.material)
                * shape_se_mult(s->crosses.shape) * tm_c;
    double pattern_se_mult = (s->pattern == STRING_OPEN_16x19) ? 1.05 : 0.95;
    double se = (w_se_m * se_m + w_se_c * se_c) * pattern_se_mult;

    /* Differential-tension snapback. Stringers exploit a known trick:
     * pull the mains a kilogram or two tighter than the crosses, and
     * pair them with slick (round, non-textured) crosses. During the
     * compression phase of impact the mains lift slightly off the
     * cross intersections — normal force between the two directions
     * drops, mains-on-crosses friction drops, and the mains slide
     * laterally far enough to snap back during decompression. That
     * snap-back is what imparts the extra angular momentum to the
     * ball over and above the surface-friction term modelled above.
     *
     *   bonus  = 1 + 0.03 · min(5, Δlb) · cross_gate
     *   Δlb    = mains_lb − crosses_lb (only positive differentials)
     *   gate   = 1 / shape_se_mult(crosses) — round (1.0), shaped
     *            (0.91), twisted (0.85). Rougher crosses grip the
     *            mains and shrink the bonus.
     *
     * At the cap (mains +5 lb over crosses, round crosses) the
     * spin-eff multiplier reaches +15 %. The default kit (mains 52,
     * crosses 50, twisted crosses) only sees a +5 % bonus — small
     * but in the direction match data shows. */
    double diff_lb = s->mains.tension_lbs - s->crosses.tension_lbs;
    if (diff_lb > 0.0) {
        if (diff_lb > 5.0) diff_lb = 5.0;
        double cross_gate = 1.0 / shape_se_mult(s->crosses.shape);
        se *= 1.0 + 0.03 * diff_lb * cross_gate;
    }

    if (se < 0.60) se = 0.60;
    if (se > 1.60) se = 1.60;
    s->spin_efficiency = se;
}

double racket_effective_mass(const Racket *r) {
    /* Empirical sweet-spot factor. Head-heavy rackets (balance > L/2)
     * give a larger effective mass at impact; head-light is smaller. */
    double bal_frac = r->balance_mm / 1000.0 / r->length_m;
    if (bal_frac < 0.40) bal_frac = 0.40;
    if (bal_frac > 0.60) bal_frac = 0.60;
    double head_fraction = 0.30 + 0.55 * bal_frac;  /* ~0.52 at balanced */
    return r->weight_kg * head_fraction;
}

double racket_frame_factor(const Racket *r) {
    /* Stiffer frame → less energy lost to flex → higher COR contribution. */
    double f = 1.0 + 0.005 * (r->stiffness_RA - 60.0);
    if (f < 0.90) f = 0.90;
    if (f > 1.10) f = 1.10;
    return f;
}

void strike_params_defaults(StrikeParams *sp, ShotMode mode) {
    /* Geometric contact-point inputs default to NaN so existing
     * callers (and the test fixtures) keep using the face_angle /
     * swing_path_az construction. Live UI code overrides these in
     * read_strike_params before each Swing. */
    sp->ball_contact_lon_deg = NAN;
    sp->ball_contact_lat_deg = NAN;
    sp->racket_velocity_override_mps = NAN;

    /* Sensible defaults — modern poly setup, type-2 ball, moderate swing. */
    sp->racket.length_m     = 0.6858;
    sp->racket.weight_kg    = 0.310;
    sp->racket.balance_mm   = 320.0;
    sp->racket.stiffness_RA = 66.0;
    sp->racket.arm_length_m = 0.60;

    /* Modern "spin" hybrid: poly twisted mains @ 52 lb, multi round crosses
     * @ 50 lb (~2 lb softer is conventional for comfort). */
    sp->strings.mains.material     = STRING_POLY;
    sp->strings.mains.shape        = STRING_TWISTED;
    sp->strings.mains.tension_lbs  = 52.0;
    sp->strings.crosses.material   = STRING_NYLON;
    sp->strings.crosses.shape      = STRING_ROUND;
    sp->strings.crosses.tension_lbs= 50.0;
    sp->strings.pattern            = STRING_OPEN_16x19;
    strings_recompute(&sp->strings);

    sp->ball_type = BALL_ITF_TYPE2;

    /* Spin ceiling on by default (pepper's behaviour). A consumer that wants the
     * legacy uncapped sliding-friction result (rakija parity) builds with
     * -DBALLISTIC_SPIN_CAP_DEFAULT=false, or sets sp->spin_cap=false itself. */
    sp->spin_cap = BALLISTIC_SPIN_CAP_DEFAULT;

    kinetic_chain_defaults(&sp->kc, mode);

    if (mode == MODE_GROUNDSTROKE) {
        /* Mid-paced rally ball (after one bounce on opponent's side, just
         * cleared the net at moderate speed with some topspin still on it). */
        sp->in_speed         = 15.0;
        sp->in_elevation_deg = -5.0;
        sp->in_azimuth_deg   = 0.0;
        sp->in_spin_rpm      = 800.0;
        sp->in_sidespin_rpm  = 0.0;
        /* Contact ~waist-high, ~2.65 m behind own baseline. Baseline
         * sits at x = COURT_X_LO; contact_z centred. */
        sp->contact_x        = COURT_X_LO - 2.65;
        sp->contact_y        = 0.95;
        sp->contact_z        = COURT_Z_MID;
        /* Solid topspin forehand. swing_path_elev is the *arm direction*
         * (the conscious low-to-high motion). Hip+shoulder contributions
         * from the kinetic chain add their own forward velocity, which
         * tilts the resolved swing path flatter than the arm direction. */
        sp->swing_path_elev_deg = 35.0;
        sp->swing_path_az_deg   = 0.0;
        sp->face_angle_deg      = 6.0;
    } else {
        /* Serve: ball nearly at rest (toss apex), contact high. */
        const bool ad = (mode == MODE_SERVE_AD);
        sp->in_speed         = 1.0;
        sp->in_elevation_deg = -90.0; /* falling */
        sp->in_azimuth_deg   = 0.0;
        sp->in_spin_rpm      = 0.0;
        sp->in_sidespin_rpm  = 0.0;
        /* Serve: contact just inside own baseline (~0.5 m forward
         * of the baseline). +z is the player's right; ad serve
         * stands ad-court-side (+z half), deuce serve deuce-side. */
        sp->contact_x        = COURT_X_LO + 0.5;
        sp->contact_y        = 2.44;
        sp->contact_z        = ad ? COURT_Z_MID + 1.5 : COURT_Z_MID - 1.5;
        sp->swing_path_elev_deg = -3.0;
        sp->swing_path_az_deg   = ad ? -8.0 : +8.0;
        sp->face_angle_deg      = 0.0;
    }
}

void kinetic_chain_defaults(KineticChain *kc, ShotMode mode) {
    if (mode == MODE_GROUNDSTROKE) {
        /* ATP-pro-ish forehand: leaning into the shot ~1 m/s forward,
         * no jump, no lateral drift. Hip ~600°/s, shoulder ~900°/s,
         * arm ~1500°/s. Slight forward torso lean (~5°) is typical. */
        kc->body_v_forward    = 1.0;
        kc->body_v_vertical   = 0.0;
        kc->body_v_lateral    = 0.0;
        kc->hip_rate_dps      = 600.0;
        kc->shoulder_rate_dps = 900.0;
        kc->arm_rate_dps      = 1500.0;
        kc->lean_forward_deg  = 5.0;
        kc->lean_lateral_deg  = 0.0;
    } else {
        /* Serve: jumping up + slightly forward into the court, faster
         * shoulder/arm because of the trophy-position uncoil + wrist snap.
         * Slight back-arch into the trophy means a tiny BACKWARD lean. */
        kc->body_v_forward    = 1.5;
        kc->body_v_vertical   = 1.5;
        kc->body_v_lateral    = 0.0;
        kc->hip_rate_dps      = 700.0;
        kc->shoulder_rate_dps = 1100.0;
        kc->arm_rate_dps      = 2000.0;
        kc->lean_forward_deg  = -8.0;
        kc->lean_lateral_deg  = 0.0;
    }
}

void kinetic_chain_racket_velocity(const KineticChain *kc,
                                   double arm_length_m,
                                   double arm_dir_elev_deg,
                                   double arm_dir_az_deg,
                                   double out_v[3]) {
    /* Fixed body geometry constants for a typical right-handed forehand.
     * Each is a unit-ish direction vector tangent to the segment's rotation
     * at the racket contact point. */
    const double HIP_R = 0.50;   /* hip axis -> racket head radius (m) */
    const double SHO_R = 0.70;   /* shoulder axis -> racket head radius (m) */
    /* HIP_DIR/SHO_DIR: tangent directions at racket. Both rotations are
     * around the vertical (body-axis), so they contribute mostly horizontal
     * velocity at the racket; the small +y on SHO_DIR captures the natural
     * upward lift of the shoulder during the kinetic chain. */
    const double HIP_DIR[3] = { 1.00, 0.00, 0.05 };
    const double SHO_DIR[3] = { 0.95, 0.10, 0.20 };

    double hip_rad = kc->hip_rate_dps      * M_PI / 180.0;
    double sho_rad = kc->shoulder_rate_dps * M_PI / 180.0;
    double arm_rad = kc->arm_rate_dps      * M_PI / 180.0;

    double v_hip_mag = hip_rad * HIP_R;
    double v_sho_mag = sho_rad * SHO_R;
    double v_arm_mag = arm_rad * arm_length_m;

    double a_elev = arm_dir_elev_deg * M_PI / 180.0;
    double a_az   = arm_dir_az_deg   * M_PI / 180.0;
    double arm_dir[3] = {
        cos(a_elev) * cos(a_az),
        sin(a_elev),
        cos(a_elev) * sin(a_az),
    };

    /* Rotational kinetic-chain contribution, expressed in body frame
     * (axes aligned with the player's torso). */
    double v_rot[3] = {
        v_hip_mag * HIP_DIR[0] + v_sho_mag * SHO_DIR[0] + v_arm_mag * arm_dir[0],
        v_hip_mag * HIP_DIR[1] + v_sho_mag * SHO_DIR[1] + v_arm_mag * arm_dir[1],
        v_hip_mag * HIP_DIR[2] + v_sho_mag * SHO_DIR[2] + v_arm_mag * arm_dir[2],
    };

    /* Apply body lean (body frame -> world frame). Forward lean rotates
     * around the lateral axis (+z); positive lean tilts the body's +y axis
     * toward +x. Lateral lean rotates around the forward axis (+x); positive
     * tilts +y toward +z. */
    double lf = kc->lean_forward_deg * M_PI / 180.0;
    double ll = kc->lean_lateral_deg * M_PI / 180.0;
    double cf = cos(lf), sf = sin(lf);
    double cl = cos(ll), sl = sin(ll);

    /* R_z(lean_forward) : forward lean */
    double x1 =  v_rot[0] * cf + v_rot[1] * sf;
    double y1 = -v_rot[0] * sf + v_rot[1] * cf;
    double z1 =  v_rot[2];
    /* R_x(lean_lateral) : sideways lean */
    double x2 = x1;
    double y2 = y1 * cl - z1 * sl;
    double z2 = y1 * sl + z1 * cl;

    /* World-frame total = rotated kinetic-chain contribution + CoM translation
     * (which is already in world frame — body motion isn't tilted with lean). */
    out_v[0] = x2 + kc->body_v_forward;
    out_v[1] = y2 + kc->body_v_vertical;
    out_v[2] = z2 + kc->body_v_lateral;
}

static inline void norm3(double v[3]) {
    double n = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (n > 1e-12) { v[0] /= n; v[1] /= n; v[2] /= n; }
}

static inline double dot3(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static inline void cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* Sliding-friction impact for the racket strike. Unlike apply_impact (which
 * assumes the ball ends at pure rolling on the surface — appropriate for a
 * ground bounce where cof ≈ 0.6), this routine treats the contact as Coulomb
 * sliding capped by the rolling threshold. With realistic string friction
 * (mu ≈ 0.20) the ball doesn't reach rolling, and *more* friction means
 * *more* spin — the intuition we want for racket strikes.
 *
 * Mutates v_in/w_in. Inputs:
 *   m_ratio = m_racket_eff / (m_racket_eff + m_ball) — finite-mass correction
 */
static void strike_impact(double v_in[3], double w_in[3],
                          const double v_surf[3], const double n_surf[3],
                          double e_a, double mu, double spin_eff,
                          double ball_r, double m_ratio, bool spin_cap) {
    double vr[3] = { v_in[0] - v_surf[0], v_in[1] - v_surf[1], v_in[2] - v_surf[2] };
    double vn_dot = dot3(vr, n_surf);

    /* Normal velocity reflection with finite-mass + COR: */
    double dv_n_mag = (1.0 + e_a) * m_ratio * vn_dot;
    v_in[0] -= dv_n_mag * n_surf[0];
    v_in[1] -= dv_n_mag * n_surf[1];
    v_in[2] -= dv_n_mag * n_surf[2];

    /* Contact-point slip velocity (ball-surface velocity at the impact patch,
     * relative to the racket surface). Contact point on ball is at -R*n.
     * v_contact = v_ball + w × (-R n). */
    double minus_Rn[3] = { -ball_r * n_surf[0], -ball_r * n_surf[1], -ball_r * n_surf[2] };
    double w_cross[3]; cross3(w_in, minus_Rn, w_cross);
    double v_contact[3] = { v_in[0] + w_cross[0], v_in[1] + w_cross[1], v_in[2] + w_cross[2] };
    double slip[3]  = { v_contact[0] - v_surf[0], v_contact[1] - v_surf[1], v_contact[2] - v_surf[2] };
    double slip_n  = dot3(slip, n_surf);
    double slip_t[3] = { slip[0] - slip_n*n_surf[0], slip[1] - slip_n*n_surf[1], slip[2] - slip_n*n_surf[2] };
    double slip_mag = sqrt(dot3(slip_t, slip_t));
    if (slip_mag < 1e-9) return;
    double slip_dir[3] = { slip_t[0]/slip_mag, slip_t[1]/slip_mag, slip_t[2]/slip_mag };

    /* Friction impulse per unit ball mass. Coulomb cap or rolling-stop cap,
     * whichever is smaller. The 2/7 factor comes from combining translation
     * and spin contributions to the contact-point velocity for a solid sphere. */
    double Jn_per_m = (1.0 + e_a) * m_ratio * fabs(vn_dot);
    double Jt_max_per_m = mu * Jn_per_m;
    double Jt_stop_per_m = (2.0 / 7.0) * slip_mag;
    double Jt_per_m = Jt_max_per_m < Jt_stop_per_m ? Jt_max_per_m : Jt_stop_per_m;

    /* Apply tangential impulse on ball CoM and resulting torque on spin. */
    v_in[0] -= Jt_per_m * slip_dir[0];
    v_in[1] -= Jt_per_m * slip_dir[1];
    v_in[2] -= Jt_per_m * slip_dir[2];

    /* Δω from torque about ball centre: r × F.
     * Force on ball direction = -slip_dir (friction opposes slip).
     * Lever arm r = -R*n. Δω = (-R*n) × (-slip_dir * Jt) / I  with I=(2/5)mR².
     * = (5/(2R)) * (n × slip_dir) * Jt_per_m.
     * spin_eff multiplies the angular impulse to capture string snap-back. */
    double nxs[3]; cross3(n_surf, slip_dir, nxs);
    double dw_factor = 5.0 / (2.0 * ball_r) * Jt_per_m * spin_eff;
    w_in[0] += dw_factor * nxs[0];
    w_in[1] += dw_factor * nxs[1];
    w_in[2] += dw_factor * nxs[2];

    /* Spin ceiling: soft-saturate the resulting spin rate so a steeper brush
     * can't manufacture unbounded RPM (see top of file). Rates below the knee
     * are untouched; above, the magnitude asymptotes toward SPIN_MAX. */
    double wmag = sqrt(dot3(w_in, w_in));
    if (spin_cap && wmag > SPIN_KNEE_RADS) {
        double sat = SPIN_KNEE_RADS + (SPIN_MAX_RADS - SPIN_KNEE_RADS)
                     * tanh((wmag - SPIN_KNEE_RADS) / (SPIN_MAX_RADS - SPIN_KNEE_RADS));
        double s = sat / wmag;
        w_in[0] *= s; w_in[1] *= s; w_in[2] *= s;
    }
}

StrikeOutput compute_strike(const StrikeParams *sp,
                            SwingParams *out_swing,
                            const SwingParams *conditions) {
    BallProps bp = ball_props_for_type(sp->ball_type);
    double m_eff = racket_effective_mass(&sp->racket);

    /* Composite apparent COR: strings * frame * ball. */
    double e_a = sp->strings.cor_strings
               * racket_frame_factor(&sp->racket)
               * bp.cor_with_strings;
    if (e_a < 0.20) e_a = 0.20;
    if (e_a > 0.95) e_a = 0.95;

    /* Finite-mass correction factor: how much momentum the racket can
     * transfer to the ball given their mass ratio. Heavier racket -> closer to 1. */
    double m_ratio = m_eff / (m_eff + bp.mass_kg);
    double e_eff = m_ratio * (1.0 + e_a) - 1.0;  /* exposed for inspection only */

    /* Friction coefficient at the impact, with ball multiplier. */
    double mu = sp->strings.mu_strings * bp.friction_mult;

    /* Incoming ball velocity vector.
     * The ball is flying toward the player; in the world frame this is the
     * negative of (in_elev, in_az) — i.e. its velocity component along +x
     * (downrange from server) is negative if the receiver is at +x. We follow
     * the convention that the simulation always lays out the player at the
     * +x-low end firing toward +x. Incoming ball therefore has -x-direction
     * velocity. */
    double in_elev = sp->in_elevation_deg * M_PI / 180.0;
    double in_az   = sp->in_azimuth_deg   * M_PI / 180.0;
    double v_in[3] = {
        -sp->in_speed * cos(in_elev) * cos(in_az),
         sp->in_speed * sin(in_elev),
         -sp->in_speed * cos(in_elev) * sin(in_az),
    };
    /* Spin-axis convention matches simulate_swing (see physics.c):
     *   ω_x =  sidespin_rpm — slice / kick axis
     *   ω_z = -spin_rpm     — topspin axis */
    double w_in[3] = {
         sp->in_sidespin_rpm * M_PI / 30.0,
         0.0,
        -sp->in_spin_rpm     * M_PI / 30.0,
    };

    /* Racket head velocity at the moment of contact.
     *
     * Preferred path: the UI's 3D swing-arrow widget supplies the
     * full racket-head velocity vector (magnitude + direction), so
     * compute_strike just unpacks it. This is the "swing plus
     * movement" combined into the single vector the user has
     * already pre-summed.
     *
     * Legacy path: when racket_velocity_override_mps is NaN we fall
     * back to kinetic_chain_racket_velocity, which composes v_r
     * from body translation, hip/shoulder/arm rotations, and the
     * swing-path arm-direction angles. The test fixtures and any
     * pre-geometry callers still take this path. */
    double v_r[3];
    if (!isnan(sp->racket_velocity_override_mps)) {
        double elev = sp->swing_path_elev_deg * M_PI / 180.0;
        double az   = sp->swing_path_az_deg   * M_PI / 180.0;
        double mag  = sp->racket_velocity_override_mps;
        v_r[0] = mag * cos(elev) * cos(az);
        v_r[1] = mag * sin(elev);
        v_r[2] = mag * cos(elev) * sin(az);
    } else {
        kinetic_chain_racket_velocity(&sp->kc, sp->racket.arm_length_m,
                                      sp->swing_path_elev_deg,
                                      sp->swing_path_az_deg, v_r);
    }
    double swing_speed_resolved = sqrt(v_r[0]*v_r[0] + v_r[1]*v_r[1] + v_r[2]*v_r[2]);
    double resolved_elev = (swing_speed_resolved > 1e-6)
                         ? asin(v_r[1] / swing_speed_resolved)
                         : 0.0;
    double resolved_az = atan2(v_r[2], v_r[0]);

    /* Racket face normal at the impact.
     *
     * Preferred path (geometric): the racket face is tangent to the ball
     * at the contact point, so the face normal is exactly the inverse
     * of the ball-frame outward normal at (lon, lat). Convert the ball
     * frame (+z = direction of travel, +y up, +x ball's right) into
     * the world frame (+x downrange, +y up, +z player's right):
     *   ball +z ↔ world -x,  ball +y ↔ world +y,  ball +x ↔ world +z.
     * Ball outward normal in world:
     *   (-cos(lat)cos(lon), sin(lat), cos(lat)sin(lon))
     * Racket face normal = - that:
     *   ( cos(lat)cos(lon), -sin(lat), -cos(lat)sin(lon)).
     *
     * Legacy path (when both contact fields are NaN): fall back to the
     * old face_angle + resolved swing-azimuth construction so callers
     * that haven't been updated still work. */
    double n_r[3];
    int have_geom = !isnan(sp->ball_contact_lon_deg)
                 && !isnan(sp->ball_contact_lat_deg);
    if (have_geom) {
        double lon = sp->ball_contact_lon_deg * M_PI / 180.0;
        double lat = sp->ball_contact_lat_deg * M_PI / 180.0;
        n_r[0] =  cos(lat) * cos(lon);
        n_r[1] = -sin(lat);
        n_r[2] = -cos(lat) * sin(lon);
    } else {
        double face = sp->face_angle_deg * M_PI / 180.0;
        n_r[0] = cos(face) * cos(resolved_az);
        n_r[1] = sin(face);
        n_r[2] = cos(face) * sin(resolved_az);
    }
    norm3(n_r);

    /* String-bed pocketing + dwell time.
     *
     * Model the ball-on-strings impact as a simple harmonic
     * oscillator with the ball as the mass and the string bed as the
     * spring. Spring constant scales linearly with average tension:
     *
     *   k_bed = K_BED_REF · (T_avg / T_REF)
     *
     * with K_BED_REF = 50 kN/m at T_REF = 55 lb chosen to land the
     * default kit at ~3.4 ms dwell — the middle of the 3-5 ms band
     * Cross & Bower measure for tour setups.
     *
     * Pocket depth (deflection at peak compression) follows from
     * energy conservation against the spring:
     *   D = V_n · √(m / k_bed)
     * Dwell time (full half-period of compression + decompression):
     *   t_dwell = π · √(m / k_bed)
     *
     * Dwell time then modulates spin transfer: longer dwell lets
     * the friction impulse integrate longer before contact ends,
     * raising the post-impact ω. The sensitivity is small (α = 0.30
     * on the dwell ratio, capped at ±20 %) because the existing
     * tension multipliers in strings_recompute already capture most
     * of the "low-tension = more spin" effect — this layer only
     * adds the *time-integral* contribution on top. */
    const double K_BED_REF_NM = 50000.0;  /* N/m at T_REF_LB */
    const double T_REF_LB     = 55.0;
    const double DWELL_ALPHA  = 0.30;
    double T_avg_lb = 0.5 * (sp->strings.mains.tension_lbs +
                             sp->strings.crosses.tension_lbs);
    double k_bed = K_BED_REF_NM * (T_avg_lb / T_REF_LB);
    if (k_bed < 10000.0) k_bed = 10000.0;
    double t_dwell_s     = M_PI * sqrt(bp.mass_kg / k_bed);
    double t_dwell_ref_s = M_PI * sqrt(bp.mass_kg / K_BED_REF_NM);

    /* Normal-velocity approximation: project (v_in − v_r) onto the
     * face normal. We compute it before strike_impact mutates v_in
     * so the pocket-depth diagnostic stays meaningful. */
    double v_rel_pre[3] = {
        v_in[0] - v_r[0], v_in[1] - v_r[1], v_in[2] - v_r[2],
    };
    double v_n_pre = fabs(v_rel_pre[0]*n_r[0] + v_rel_pre[1]*n_r[1] +
                          v_rel_pre[2]*n_r[2]);
    /* Linear SHO would give D = V_n · √(m/k), but real strings
     * stiffen at large deflection so the measured peak depth lands
     * at roughly half the linear prediction (Cross 2003: ~18 mm at
     * a 30 m/s impact, where pure linear would give ~33 mm). The
     * 0.5 calibration matches that; we also cap at 80 % of the ball
     * radius because anything deeper means the ball would be
     * geometrically submerged in the bed, which is impossible. */
    const double POCKET_CAL = 0.5;
    double pocket_m = POCKET_CAL * v_n_pre * sqrt(bp.mass_kg / k_bed);
    double pocket_cap = 0.8 * bp.radius_m;
    if (pocket_m > pocket_cap) pocket_m = pocket_cap;

    /* Contact-patch diameter on the ball: the ball sinks D into the
     * bed, so the bed surface cuts a circle of radius
     *   r_patch = √(R² − (R − D)²) = √(2RD − D²)
     * across the ball, and the diameter is 2·r_patch. Dividing by
     * the pattern's mains spacing gives the count of mains the ball
     * physically contacts at peak compression. Fractional values
     * are kept so the UI can show "≈4.7". */
    double under = 2.0 * bp.radius_m * pocket_m - pocket_m * pocket_m;
    double d_patch = under > 0.0 ? 2.0 * sqrt(under) : 0.0;
    double mains_engaged = d_patch / mains_spacing_m(sp->strings.pattern);

    double dwell_factor = pow(t_dwell_s / t_dwell_ref_s, DWELL_ALPHA);
    if (dwell_factor < 0.85) dwell_factor = 0.85;
    if (dwell_factor > 1.20) dwell_factor = 1.20;

    /* Apply the strike impact (sliding-friction model + snap-back).
     * Spin transfer is the snap-back efficiency × the dwell-time
     * scaling. */
    strike_impact(v_in, w_in, v_r, n_r, e_a, mu,
                  sp->strings.spin_efficiency * dwell_factor,
                  bp.radius_m, m_ratio, sp->spin_cap);

    /* Convert outgoing velocity vector back to (speed, angle, azimuth). */
    double speed_out = sqrt(v_in[0]*v_in[0] + v_in[1]*v_in[1] + v_in[2]*v_in[2]);
    double angle_out = (speed_out > 1e-6) ? asin(v_in[1] / speed_out) : 0.0;
    double az_out    = atan2(v_in[2], v_in[0]); /* in [-pi, pi] */

    /* Convert outgoing spin (rad/s) to SwingParams (rpm). Inverse of
     * input map: spin_rpm = -wz * 30/pi, sidespin_rpm = wx * 30/pi
     * (slice axis). */
    double spin_rpm     = -w_in[2] * 30.0 / M_PI;
    double sidespin_rpm =  w_in[0] * 30.0 / M_PI;

    /* Fill in the SwingParams for the flight model. Copy environment-only
     * params from the caller's conditions struct. */
    out_swing->mode          = conditions->mode;
    out_swing->start_x       = sp->contact_x;
    out_swing->start_y       = sp->contact_y;
    out_swing->start_z       = sp->contact_z;
    out_swing->power         = speed_out;
    out_swing->angle_deg     = angle_out * 180.0 / M_PI;
    out_swing->azimuth_deg   = az_out    * 180.0 / M_PI;
    out_swing->spin_rpm      = spin_rpm;
    out_swing->sidespin_rpm  = sidespin_rpm;
    out_swing->windspeed     = conditions->windspeed;
    out_swing->winddirection = conditions->winddirection;
    out_swing->cor           = conditions->cor;
    out_swing->cof           = conditions->cof;
    out_swing->air_density   = conditions->air_density;
    out_swing->ball_type     = sp->ball_type;

    StrikeOutput out = {
        .racket_speed         = swing_speed_resolved,
        .racket_path_elev_deg = resolved_elev * 180.0 / M_PI,
        .racket_path_az_deg   = resolved_az   * 180.0 / M_PI,
        .exit_speed           = speed_out,
        .exit_angle_deg       = angle_out * 180.0 / M_PI,
        .exit_azimuth_deg     = az_out    * 180.0 / M_PI,
        .exit_spin_rpm        = spin_rpm,
        .exit_sidespin_rpm    = sidespin_rpm,
        .e_a_effective        = e_eff,
        .mu_used              = mu,
        .m_eff_kg             = m_eff,
        .pocket_depth_m       = pocket_m,
        .dwell_ms             = t_dwell_s * 1000.0,
        .mains_engaged        = mains_engaged,
    };
    return out;
}
