/* Headless tests for pepper's math core — no GTK. Exercises the toss
 * integrator, the swing-plane arc, the space-time contact solver, and the
 * handoff into rakija's strike + flight physics. Also a couple of sanity
 * checks on the borrowed strike model so we'd notice if the copy drifted.
 *
 * Build + run: make test */

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "physics.h"
#include "strike.h"
#include "toss.h"
#include "pose.h"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, ...) do {                                          \
    if (cond) { g_pass++; }                                            \
    else { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__);          \
           printf("  [%s:%d]\n", __FILE__, __LINE__); }                \
} while (0)

static void test_strike_core(void) {
    printf("strike core (borrowed from rakija):\n");
    /* A flat-ish serve strike still produces a fast forward ball. */
    StrikeParams sp; strike_params_defaults(&sp, MODE_SERVE_DEUCE);
    sp.racket_velocity_override_mps = 38.0;
    sp.swing_path_elev_deg = -4.0;
    sp.swing_path_az_deg = 6.0;
    sp.ball_contact_lon_deg = 6.0;
    sp.ball_contact_lat_deg = -4.0;
    sp.in_speed = 0.5; sp.in_elevation_deg = -90.0;
    SwingParams cond; memset(&cond, 0, sizeof cond);
    cond.mode = MODE_SERVE_DEUCE; cond.cor = 0.73; cond.cof = 0.60;
    cond.ball_type = sp.ball_type;
    SwingParams out; memset(&out, 0, sizeof out);
    StrikeOutput so = compute_strike(&sp, &out, &cond);
    CHECK(so.exit_speed > 30.0, "serve exit speed %.1f should exceed racket-ish 30", so.exit_speed);
    CHECK(out.power == so.exit_speed, "out.power should mirror exit_speed");
}

/* Reusable: run defaults for a mode, optionally tweak, return result. */
static ServeResult run_default(ShotMode mode, void (*tweak)(ServeParams*)) {
    ServeParams p; serve_params_defaults(&p, mode);
    if (tweak) tweak(&p);
    return serve_simulate(&p);
}

static void test_toss_arc(void) {
    printf("toss arc:\n");
    ServeResult r = run_default(MODE_SERVE_DEUCE, NULL);
    CHECK(r.toss_n > 10, "toss should have samples (%zu)", r.toss_n);
    /* Toss must rise above the release height before falling. */
    double peak = -1e9;
    for (size_t i = 0; i < r.toss_n; i++) if (r.toss_y[i] > peak) peak = r.toss_y[i];
    CHECK(peak > 2.5, "toss peak %.2f m should clear contact height region", peak);
    /* Ball ends up on or below the ground by the end of the search. */
    CHECK(r.toss_y[r.toss_n-1] < 0.2, "toss should fall back to ground (end y=%.2f)",
          r.toss_y[r.toss_n-1]);
    serve_result_free(&r);
}

static void test_clean_contact(void) {
    printf("clean contact:\n");
    ServeResult r = run_default(MODE_SERVE_DEUCE, NULL);
    CHECK(r.contact, "default deuce serve should make contact (miss dist %.3f)",
          r.miss_distance_m);
    if (r.contact) {
        /* The reach tilts forward so the apex sits up-and-forward over the ball
         * (even though the pivot is the back-right shoulder), so a well-timed
         * swing contacts at full extension (phi ≈ 0). */
        CHECK(fabs(r.phi_contact_deg) < 15.0,
              "clean serve contacts near full extension (phi=%.1f deg)", r.phi_contact_deg);
        CHECK(r.contact_pos[1] > 2.0,
              "serve contact should be high (y=%.2f)", r.contact_pos[1]);
        CHECK(r.strike_out.exit_speed > 25.0,
              "serve exit speed %.1f m/s should be fast", r.strike_out.exit_speed);
    }
    serve_result_free(&r);
}

static void test_clean_serve_lands_in(void) {
    printf("clean serve verdict:\n");
    ServeResult d = run_default(MODE_SERVE_DEUCE, NULL);
    printf("    deuce: verdict=%s land=(%.2f,%.2f) exit=%.1f m/s phi=%.1f\n",
           d.contact ? verdict_str(d.flight.verdict) : "NO-CONTACT",
           d.flight.landing_x, d.flight.landing_z, d.strike_out.exit_speed,
           d.phi_contact_deg);
    CHECK(d.contact && d.flight.verdict == VERDICT_IN,
          "default deuce serve should land IN (got %s)",
          d.contact ? verdict_str(d.flight.verdict) : "NO-CONTACT");
    serve_result_free(&d);

    ServeResult a = run_default(MODE_SERVE_AD, NULL);
    printf("    ad:    verdict=%s land=(%.2f,%.2f) exit=%.1f m/s phi=%.1f\n",
           a.contact ? verdict_str(a.flight.verdict) : "NO-CONTACT",
           a.flight.landing_x, a.flight.landing_z, a.strike_out.exit_speed,
           a.phi_contact_deg);
    CHECK(a.contact && a.flight.verdict == VERDICT_IN,
          "default ad serve should land IN (got %s)",
          a.contact ? verdict_str(a.flight.verdict) : "NO-CONTACT");
    serve_result_free(&a);
}

/* Mistiming the swing should wreck the serve: swinging much too early (the
 * racket reaches full extension long before the ball arrives) either misses
 * or no longer lands cleanly in the box. */
static void tweak_late_swing(ServeParams *p)  { p->apex_time_s += 0.30; }
static void tweak_early_swing(ServeParams *p) { p->apex_time_s -= 0.30; }

static void test_mistime(void) {
    printf("mistiming:\n");
    ServeResult late  = run_default(MODE_SERVE_DEUCE, tweak_late_swing);
    ServeResult early = run_default(MODE_SERVE_DEUCE, tweak_early_swing);
    printf("    late : contact=%d verdict=%s miss=%.3f phi=%.1f\n",
           late.contact, late.contact ? verdict_str(late.flight.verdict) : "-",
           late.miss_distance_m, late.phi_contact_deg);
    printf("    early: contact=%d verdict=%s miss=%.3f phi=%.1f\n",
           early.contact, early.contact ? verdict_str(early.flight.verdict) : "-",
           early.miss_distance_m, early.phi_contact_deg);
    int late_bad  = !late.contact  || late.flight.verdict  != VERDICT_IN;
    int early_bad = !early.contact || early.flight.verdict != VERDICT_IN;
    CHECK(late_bad,  "a 0.30 s late swing should not land a clean IN serve");
    CHECK(early_bad, "a 0.30 s early swing should not land a clean IN serve");
    serve_result_free(&late);
    serve_result_free(&early);
}

/* The swing-through elevation should drive the exit elevation: raising
 * plane_elev (swing through the ball more upward) should raise the exit
 * angle — a monotone check that the swing orientation steers the result. */
static void tweak_up(ServeParams *p) { p->plane_elev_deg += 10.0; }

static void test_swing_steers(void) {
    printf("swing steers exit:\n");
    ServeResult base = run_default(MODE_SERVE_DEUCE, NULL);
    ServeResult up   = run_default(MODE_SERVE_DEUCE, tweak_up);
    if (base.contact && up.contact) {
        printf("    base exit_angle=%.1f, more-up exit_angle=%.1f\n",
               base.strike_out.exit_angle_deg, up.strike_out.exit_angle_deg);
        CHECK(up.strike_out.exit_angle_deg > base.strike_out.exit_angle_deg + 0.5,
              "swinging more upward should raise the exit angle (%.1f -> %.1f)",
              base.strike_out.exit_angle_deg, up.strike_out.exit_angle_deg);
    } else {
        CHECK(0, "both base and up should make contact (%d/%d)",
              base.contact, up.contact);
    }
    serve_result_free(&base);
    serve_result_free(&up);
}

/* Groundstroke: the same arm-sweep contact solver, fed an INCOMING ball instead
 * of a toss, makes a chest-high topspin contact in front and lands the return
 * deep + IN; mistiming the swing misses. */
static void test_groundstroke(void) {
    printf("groundstroke (incoming-ball track):\n");
    ServeParams p; serve_params_defaults(&p, MODE_GROUNDSTROKE);
    ServeResult r = serve_simulate(&p);
    printf("    contact=%d at h=%.2f phi=%.1f  exit=%.1f m/s spin=%.0f rpm  verdict=%s land=(%.2f,%.2f)\n",
           r.contact, r.contact_pos[1], r.phi_contact_deg, r.strike_out.exit_speed,
           r.strike_out.exit_spin_rpm,
           r.contact ? verdict_str(r.flight.verdict) : "-", r.flight.landing_x, r.flight.landing_z);
    CHECK(r.contact, "default groundstroke makes contact with the incoming ball");
    CHECK(r.contact_pos[1] > 0.8 && r.contact_pos[1] < 2.3, "contact is chest/waist height (%.2f m)", r.contact_pos[1]);
    CHECK(r.contact && r.flight.verdict == VERDICT_IN, "groundstroke lands IN");
    CHECK(r.strike_out.exit_spin_rpm > 500.0, "low-to-high brush produces topspin (%.0f rpm)", r.strike_out.exit_spin_rpm);
    CHECK(r.flight.landing_x > NET_X, "the return clears into the opponent's court");
    serve_result_free(&r);

    ServeParams late = p; late.apex_time_s += 0.30;
    ServeResult rl = serve_simulate(&late);
    CHECK(!rl.contact || rl.flight.verdict != VERDICT_IN, "a 0.30 s mistimed groundstroke doesn't land a clean IN");
    serve_result_free(&rl);
}

/* The inverse toss solver hits the requested landing + apex + side. */
static void test_toss_solver(void) {
    printf("toss solver hits its target:\n");
    struct { double fwd, apex, side; } tg[] = {
        { 0.40, 3.10, 0.00 }, { 0.30, 3.40, 0.20 }, { 0.60, 2.90, -0.20 },
    };
    for (int i = 0; i < 3; i++) {
        ServeParams p; serve_params_defaults(&p, MODE_SERVE_DEUCE);
        serve_solve_toss(&p, tg[i].fwd, tg[i].apex);
        serve_solve_toss_side(&p, tg[i].side);
        ServeResult r = serve_simulate(&p);
        double fwd  = r.toss_te[0] - COURT_X_LO;
        double side = r.toss_te[2] - r.ls_pos[2];
        printf("    target (%.2f,%.2f,%.2f) -> got (%.2f,%.2f,%.2f)\n",
               tg[i].fwd, tg[i].apex, tg[i].side, fwd, r.toss_apex[1], side);
        CHECK(fabs(fwd - tg[i].fwd) < 0.08,  "toss lands fwd %.2f, wanted %.2f", fwd, tg[i].fwd);
        CHECK(fabs(r.toss_apex[1] - tg[i].apex) < 0.08, "toss apex %.2f, wanted %.2f", r.toss_apex[1], tg[i].apex);
        CHECK(fabs(side - tg[i].side) < 0.08, "toss side %.2f, wanted %.2f", side, tg[i].side);
        serve_result_free(&r);
    }
}

static void test_humidity(void) {
    printf("humidity in the air model:\n");
    /* RH=0 is byte-identical to the dry two-arg function (neutrality), and the
     * anchor still holds. */
    double dry = air_density(0.0, 30.0);
    CHECK(air_density_humid(0.0, 30.0, 0.0) == dry, "RH=0 must equal air_density()");
    CHECK(fabs(air_density(0.0, AIR_TEMP_REF_C) - AIR_RHO_SEA) < 1e-9,
          "anchor (0 m, ref temp, dry) == AIR_RHO_SEA");

    /* Humid air is lighter, monotonically in RH; ~1.5% thinner saturated at 30C. */
    double half = air_density_humid(0.0, 30.0, 50.0);
    double sat  = air_density_humid(0.0, 30.0, 100.0);
    printf("    30C sea level: dry=%.4f  50%%=%.4f  100%%=%.4f kg/m^3\n", dry, half, sat);
    CHECK(sat < half && half < dry, "dry denser than 50%% denser than saturated");
    double drop = (dry - sat) / dry * 100.0;
    CHECK(drop > 0.5 && drop < 3.0, "saturated-30C density drop %.2f%% in (0.5, 3)", drop);

    /* Tetens sanity: water boils (~1 atm vapor pressure) near 100C. */
    double psat100 = saturation_vapor_pressure_pa(100.0);
    CHECK(fabs(psat100 - 101325.0) / 101325.0 < 0.05, "Psat(100C) ~ 1 atm, got %.0f Pa", psat100);

    /* altitude_from_density_humid inverts air_density_humid (UI round-trip). */
    double rho = air_density_humid(1200.0, 25.0, 70.0);
    double alt = altitude_from_density_humid(rho, 25.0, 70.0);
    printf("    round-trip: 1200 m @25C/70%% -> %.4f kg/m^3 -> %.1f m\n", rho, alt);
    CHECK(fabs(alt - 1200.0) < 1.0, "humid altitude round-trips, got %.1f m", alt);
}

/* ===== Biomechanics (pose) — FK / IK / sampling / inverse-FK ===== */
static void test_pose(void) {
    printf("pose: racket-tip IK:\n");
    PoseRig rig;
    pose_default_prep_forehand(&rig.prep);
    pose_default_contact_forehand(&rig.contact);
    rig.scrub = 1.0; rig.swing_duration_s = 0.15;
    skeleton_from_height(&rig.skel, 1.78);

    Pose p = rig.contact; PoseJoints j;
    pose_fk(&p, &rig.skel, &j);
    double tip0[3] = { j.r_racket_tip[0], j.r_racket_tip[1], j.r_racket_tip[2] };

    pose_rig_ik_racket(&rig, &p, tip0);            /* no-op solve */
    pose_fk(&p, &rig.skel, &j);
    double e0 = hypot(hypot(j.r_racket_tip[0]-tip0[0], j.r_racket_tip[1]-tip0[1]),
                      j.r_racket_tip[2]-tip0[2]);
    CHECK(e0 < 0.05, "no-op IK lands the tip within 5 cm (err %.4f)", e0);

    double t1[3] = { tip0[0] + 0.05, tip0[1], tip0[2] };
    p = rig.contact; pose_rig_ik_racket(&rig, &p, t1); pose_fk(&p, &rig.skel, &j);
    double e1 = hypot(hypot(j.r_racket_tip[0]-t1[0], j.r_racket_tip[1]-t1[1]),
                      j.r_racket_tip[2]-t1[2]);
    CHECK(e1 < 0.05, "+5cm target lands tip within 5 cm (err %.4f)", e1);

    double far[3] = { tip0[0] + 5.0, tip0[1], tip0[2] };
    p = rig.contact; pose_rig_ik_racket(&rig, &p, far); pose_fk(&p, &rig.skel, &j);
    CHECK(isfinite(j.r_racket_tip[0]), "unreachable target clamps to a finite tip");

    printf("pose: joint trajectory sampling:\n");
    rig.swing_duration_s = 0.10;
    enum { N = 16 }; PoseJoints joints[N];
    pose_rig_sample_trajectory(&rig, N, joints);
    PoseJoints cd; pose_fk(&rig.contact, &rig.skel, &cd);
    double terr = hypot(hypot(joints[N-1].r_racket_tip[0]-cd.r_racket_tip[0],
                              joints[N-1].r_racket_tip[1]-cd.r_racket_tip[1]),
                        joints[N-1].r_racket_tip[2]-cd.r_racket_tip[2]);
    CHECK(terr < 1e-6, "trajectory sample N-1 == contact FK");
    double v[3], vmag = pose_rig_racket_tip_velocity(joints, N, rig.swing_duration_s, v);
    printf("    racket-tip |v| = %.1f m/s\n", vmag);
    CHECK(vmag > 10.0 && vmag < 80.0, "racket-tip speed in [10,80] m/s (got %.1f)", vmag);
    rig.swing_duration_s = 0.05;
    pose_rig_sample_trajectory(&rig, N, joints);
    double vf[3], vmag_fast = pose_rig_racket_tip_velocity(joints, N, 0.05, vf);
    CHECK(vmag_fast > vmag * 1.5, "shorter duration -> higher tip speed");

    printf("pose: FK -> joints -> Pose round-trip:\n");
    Skeleton skel; skeleton_from_height(&skel, 1.78);
    Pose pr[3];
    pose_default_contact_forehand(&pr[0]);
    pose_default_contact_backhand_1h(&pr[1]);
    pose_default_contact_serve_flat(&pr[2]);
    const char *nm[3] = { "forehand", "1-hand bh", "flat serve" };
    int all_ok = 1;
    for (int k = 0; k < 3; k++) {
        PoseJoints jj; pose_fk(&pr[k], &skel, &jj);
        Pose back; pose_from_joints(&jj, &skel, &back);
        double f[][2] = {
            {pr[k].spine_pitch_deg, back.spine_pitch_deg}, {pr[k].spine_roll_deg, back.spine_roll_deg},
            {pr[k].hip_yaw_deg, back.hip_yaw_deg}, {pr[k].shoulder_yaw_deg, back.shoulder_yaw_deg},
            {pr[k].r_arm_alpha_deg, back.r_arm_alpha_deg}, {pr[k].r_arm_beta_deg, back.r_arm_beta_deg},
            {pr[k].r_elbow_flex_deg, back.r_elbow_flex_deg}, {pr[k].r_wrist_flex_deg, back.r_wrist_flex_deg},
            {pr[k].l_arm_alpha_deg, back.l_arm_alpha_deg}, {pr[k].l_arm_beta_deg, back.l_arm_beta_deg},
            {pr[k].l_elbow_flex_deg, back.l_elbow_flex_deg},
            {pr[k].knee_flex_r_deg, back.knee_flex_r_deg}, {pr[k].ankle_flex_r_deg, back.ankle_flex_r_deg},
            {pr[k].knee_flex_l_deg, back.knee_flex_l_deg}, {pr[k].ankle_flex_l_deg, back.ankle_flex_l_deg},
        };
        double max_err = 0.0;
        for (unsigned i = 0; i < sizeof(f)/sizeof(f[0]); i++)
            if (fabs(f[i][0]-f[i][1]) > max_err) max_err = fabs(f[i][0]-f[i][1]);
        printf("    %-12s max err %.4f deg\n", nm[k], max_err);
        if (max_err > 1e-6) all_ok = 0;
    }
    CHECK(all_ok, "FK -> joints -> Pose recovers angles within 1e-6 deg");
    PoseJoints j0; pose_fk(&pr[0], &skel, &j0);
    Pose b0; pose_from_joints(&j0, &skel, &b0);
    double comerr = hypot(hypot(b0.com[0]-pr[0].com[0], b0.com[1]-pr[0].com[1]), b0.com[2]-pr[0].com[2]);
    CHECK(comerr < 1e-9, "CoM round-trips exactly");
}

int main(void) {
    printf("=== pepper math-core tests ===\n");
    test_strike_core();
    test_toss_arc();
    test_clean_contact();
    test_clean_serve_lands_in();
    test_mistime();
    test_swing_steers();
    test_toss_solver();
    test_groundstroke();
    test_humidity();
    test_pose();
    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
