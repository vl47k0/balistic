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

int main(void) {
    printf("=== pepper math-core tests ===\n");
    test_strike_core();
    test_toss_arc();
    test_clean_contact();
    test_clean_serve_lands_in();
    test_mistime();
    test_swing_steers();
    test_toss_solver();
    test_humidity();
    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
