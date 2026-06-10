/* pose.c — pose data model + forward kinematics + IK + default poses.
 *
 * The biomechanics half of balistic (the ball half is physics/strike/toss).
 * Pure C — no glib/JSON. The on-disk trajectory JSON I/O is a consumer-side
 * adapter (rakija's body_rig_json.c), not part of the library.
 *
 * Coordinate frame matches the rest of the physics: +x downrange
 * (toward the net), +y up, +z player's right. A right-handed
 * forehand swings the racket from the +z-back through the body
 * out to +x. */

#include "pose.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===== Small 3-vector helpers ===== */

static inline void v3_set(double v[3], double x, double y, double z) {
    v[0] = x; v[1] = y; v[2] = z;
}

static inline void v3_add_scaled(const double base[3], const double dir[3],
                                 double s, double out[3]) {
    out[0] = base[0] + dir[0] * s;
    out[1] = base[1] + dir[1] * s;
    out[2] = base[2] + dir[2] * s;
}

/* Clamp helper — used by both the inverse-FK and the IK code below. */
static inline double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* In-place rotations around the world axes (right-hand rule). */

/* rot_y: yaw — +x rotates toward +z when angle > 0. */
static void rot_y(double deg, double v[3]) {
    double r = deg * M_PI / 180.0;
    double c = cos(r), s = sin(r);
    double xn = c * v[0] + s * v[2];
    double zn = -s * v[0] + c * v[2];
    v[0] = xn; v[2] = zn;
}

/* rot_x: pitch — +y rotates toward +z when angle > 0. */
static void rot_x(double deg, double v[3]) {
    double r = deg * M_PI / 180.0;
    double c = cos(r), s = sin(r);
    double yn = c * v[1] - s * v[2];
    double zn = s * v[1] + c * v[2];
    v[1] = yn; v[2] = zn;
}

/* rot_z: roll — +x rotates toward +y when angle > 0. We use
 *   rot_z(forward_lean)
 * so positive forward lean tilts spine_top (which starts as +y)
 * toward +x — the coach's "lean into the shot" direction. */
static void rot_z(double deg, double v[3]) {
    double r = deg * M_PI / 180.0;
    double c = cos(r), s = sin(r);
    double xn = c * v[0] - s * v[1];
    double yn = s * v[0] + c * v[1];
    v[0] = xn; v[1] = yn;
}

/* ===== Skeleton sizing ===== */

void skeleton_from_height(Skeleton *s, double h) {
    /* Approximate adult body proportions. The Drillis-Contini
     * anthropometric tables give roughly:
     *   leg               0.530 h (hip to floor)
     *   torso (T+pelvis)  0.288 h
     *   shoulder span     0.259 h
     *   upper arm         0.186 h
     *   forearm + hand    0.254 h
     * Leg splits into roughly equal thigh + shin; foot adds a
     * short distal segment for the ankle-flex visualisation. */
    if (h <= 0.0) h = 1.78;
    s->thigh          = 0.225 * h;
    s->shin           = 0.225 * h;
    s->foot           = 0.10  * h;
    s->torso          = 0.30  * h;
    s->neck_to_head   = 0.05  * h;
    s->head_r         = 0.06  * h;
    s->upper_arm      = 0.16  * h;
    s->forearm        = 0.18  * h;
    s->shoulder_half  = 0.10  * h;
    s->hip_half       = 0.07  * h;
    s->racket         = 0.6858;   /* 27" */
}

/* ===== Default poses (right-handed forehand) ===== */

void pose_default_prep_forehand(Pose *p) {
    /* Pelvis dropped + behind: athletic loaded stance. */
    v3_set(p->com, -0.20, 0.85, 0.0);
    p->spine_pitch_deg  =  3.0;
    p->spine_roll_deg   =  0.0;
    /* Loaded coil: hips +45° (away from net), shoulders +100°.
     * The 55° gap is the X-factor — modern ATP forehand peak
     * separation runs 40-65°. */
    p->hip_yaw_deg      = 45.0;
    p->shoulder_yaw_deg = 100.0;
    /* Arm cocked behind the body: alpha 85° (just below horizontal),
     * beta 170° (nearly fully behind the shoulder line). */
    p->r_arm_alpha_deg  = 85.0;
    p->r_arm_beta_deg   = 170.0;
    /* Elbow tucked, wrist laid back. */
    p->r_elbow_flex_deg = 90.0;
    p->r_wrist_flex_deg = 20.0;
    /* Both knees bent for loading; ankles neutral. */
    p->knee_flex_l_deg  = 25.0;
    p->knee_flex_r_deg  = 25.0;
    p->ankle_flex_l_deg =  0.0;
    p->ankle_flex_r_deg =  0.0;
    /* Left arm extended out to the player's left side, tracking
     * the incoming ball. β=90° + mirror = -z direction (player's
     * left); slight bend at elbow keeps it natural. */
    p->l_arm_alpha_deg  = 80.0;
    p->l_arm_beta_deg   = 90.0;
    p->l_elbow_flex_deg = 35.0;
    p->l_wrist_flex_deg =  0.0;
}

void pose_default_contact_forehand(Pose *p) {
    /* Pelvis rises + moves forward (drive + weight transfer). */
    v3_set(p->com,  0.10, 0.95, 0.0);
    p->spine_pitch_deg  =  8.0;
    p->spine_roll_deg   = -3.0;   /* slight lateral lean over the front foot */
    /* Decoiled: hips facing the net, shoulders just past. */
    p->hip_yaw_deg      =  0.0;
    p->shoulder_yaw_deg = -15.0;
    /* Arm extended out and forward (contact in front of the player). */
    p->r_arm_alpha_deg  = 100.0;  /* slightly above horizontal */
    p->r_arm_beta_deg   = 10.0;
    p->r_elbow_flex_deg = 20.0;   /* nearly straight */
    p->r_wrist_flex_deg =  0.0;
    /* Legs mostly driven through; back ankle plantarflexed
     * (push-off). */
    p->knee_flex_l_deg  =  5.0;
    p->knee_flex_r_deg  =  5.0;
    p->ankle_flex_l_deg = 10.0;
    p->ankle_flex_r_deg = 20.0;
    /* Left arm folded across body, helping counter-rotate. */
    p->l_arm_alpha_deg  = 70.0;
    p->l_arm_beta_deg   = 15.0;
    p->l_elbow_flex_deg = 75.0;
    p->l_wrist_flex_deg =  0.0;
}

/* ===== One-Handed Backhand ===== */

void pose_default_prep_backhand_1h(Pose *p) {
    /* Coil to the player's LEFT — opposite of forehand. */
    v3_set(p->com, -0.20, 0.85, 0.0);
    p->spine_pitch_deg  =   3.0;
    p->spine_roll_deg   =   0.0;
    p->hip_yaw_deg      = -45.0;
    p->shoulder_yaw_deg = -90.0;
    /* Right arm crosses to the body's left side, deeply coiled. */
    p->r_arm_alpha_deg  =  90.0;
    p->r_arm_beta_deg   = -150.0;
    p->r_elbow_flex_deg =  90.0;
    p->r_wrist_flex_deg = -20.0;   /* BH grip lays the racket back the
                                      opposite way from the FH */
    p->knee_flex_l_deg  =  25.0;
    p->knee_flex_r_deg  =  25.0;
    p->ankle_flex_l_deg =   0.0;
    p->ankle_flex_r_deg =   0.0;
    /* Left arm holds the throat / counterbalances back to the right. */
    p->l_arm_alpha_deg  =  80.0;
    p->l_arm_beta_deg   = -90.0;
    p->l_elbow_flex_deg =  60.0;
    p->l_wrist_flex_deg =   0.0;
}

void pose_default_contact_backhand_1h(Pose *p) {
    /* Decoil forward; the trademark 1HBH ends with the front side
     * extended and the off arm trailing behind. */
    v3_set(p->com,  0.10, 0.95, 0.0);
    p->spine_pitch_deg  =   8.0;
    p->spine_roll_deg   =   3.0;
    p->hip_yaw_deg      =  10.0;
    p->shoulder_yaw_deg =  30.0;
    p->r_arm_alpha_deg  = 100.0;
    p->r_arm_beta_deg   = -30.0;   /* arm forward + slightly left */
    p->r_elbow_flex_deg =  10.0;
    p->r_wrist_flex_deg =   0.0;
    p->knee_flex_l_deg  =   5.0;
    p->knee_flex_r_deg  =   5.0;
    p->ankle_flex_l_deg =  10.0;
    p->ankle_flex_r_deg =  20.0;
    /* Off arm flies back through the swing (signature 1HBH look). */
    p->l_arm_alpha_deg  =  80.0;
    p->l_arm_beta_deg   = -150.0;
    p->l_elbow_flex_deg =  30.0;
    p->l_wrist_flex_deg =   0.0;
}

/* ===== Flat Serve ===== */

void pose_default_prep_serve_flat(Pose *p) {
    /* Trophy position: weight loaded, back arched, racket arm cocked
     * overhead with elbow bent so forearm sits behind the head;
     * tossing arm extended straight up. */
    v3_set(p->com, 0.0, 0.85, 0.0);
    p->spine_pitch_deg  = -10.0;       /* slight back-arch */
    p->spine_roll_deg   =   0.0;
    p->hip_yaw_deg      =  30.0;
    p->shoulder_yaw_deg =  55.0;
    p->r_arm_alpha_deg  = 150.0;
    p->r_arm_beta_deg   = 100.0;
    p->r_elbow_flex_deg =  95.0;
    p->r_wrist_flex_deg =  40.0;       /* racket laid back */
    p->knee_flex_l_deg  =  40.0;
    p->knee_flex_r_deg  =  40.0;
    p->ankle_flex_l_deg =   0.0;
    p->ankle_flex_r_deg =   0.0;
    p->l_arm_alpha_deg  = 165.0;       /* tossing arm straight up */
    p->l_arm_beta_deg   =  10.0;
    p->l_elbow_flex_deg =  10.0;
    p->l_wrist_flex_deg =   0.0;
}

void pose_default_contact_serve_flat(Pose *p) {
    /* Full extension: racket arm fully straightened toward contact
     * point high above + slightly in front, body extended up with
     * plantarflexed ankles (push-off). Tossing arm coming down. */
    v3_set(p->com, 0.10, 1.05, 0.0);
    p->spine_pitch_deg  =   5.0;
    p->spine_roll_deg   =  -3.0;
    p->hip_yaw_deg      =   0.0;
    p->shoulder_yaw_deg = -10.0;
    p->r_arm_alpha_deg  = 175.0;
    p->r_arm_beta_deg   =  20.0;
    p->r_elbow_flex_deg =   5.0;
    p->r_wrist_flex_deg =   0.0;
    p->knee_flex_l_deg  =   5.0;
    p->knee_flex_r_deg  =   5.0;
    p->ankle_flex_l_deg =  25.0;
    p->ankle_flex_r_deg =  25.0;
    p->l_arm_alpha_deg  =  90.0;       /* L arm dropping */
    p->l_arm_beta_deg   =  60.0;
    p->l_elbow_flex_deg =  60.0;
    p->l_wrist_flex_deg =   0.0;
}

/* ===== Preset loader ===== */

void pose_preset_load(PoseRig *rig, PosePreset preset) {
    switch (preset) {
    case POSE_PRESET_FOREHAND:
        pose_default_prep_forehand   (&rig->prep);
        pose_default_contact_forehand(&rig->contact);
        return;
    case POSE_PRESET_BACKHAND_1H:
        pose_default_prep_backhand_1h   (&rig->prep);
        pose_default_contact_backhand_1h(&rig->contact);
        return;
    case POSE_PRESET_SERVE_FLAT:
        pose_default_prep_serve_flat   (&rig->prep);
        pose_default_contact_serve_flat(&rig->contact);
        return;
    }
}

/* ===== Interpolation ===== */

#define LERP(s, t, a, b) ((s) * (a) + (t) * (b))

void pose_interp(const Pose *a, const Pose *b, double t, Pose *out) {
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double s = 1.0 - t;
    out->com[0]            = LERP(s, t, a->com[0],            b->com[0]);
    out->com[1]            = LERP(s, t, a->com[1],            b->com[1]);
    out->com[2]            = LERP(s, t, a->com[2],            b->com[2]);
    out->spine_pitch_deg   = LERP(s, t, a->spine_pitch_deg,   b->spine_pitch_deg);
    out->spine_roll_deg    = LERP(s, t, a->spine_roll_deg,    b->spine_roll_deg);
    out->hip_yaw_deg       = LERP(s, t, a->hip_yaw_deg,       b->hip_yaw_deg);
    out->shoulder_yaw_deg  = LERP(s, t, a->shoulder_yaw_deg,  b->shoulder_yaw_deg);
    out->r_arm_alpha_deg   = LERP(s, t, a->r_arm_alpha_deg,   b->r_arm_alpha_deg);
    out->r_arm_beta_deg    = LERP(s, t, a->r_arm_beta_deg,    b->r_arm_beta_deg);
    out->r_elbow_flex_deg  = LERP(s, t, a->r_elbow_flex_deg,  b->r_elbow_flex_deg);
    out->r_wrist_flex_deg  = LERP(s, t, a->r_wrist_flex_deg,  b->r_wrist_flex_deg);
    out->knee_flex_l_deg   = LERP(s, t, a->knee_flex_l_deg,   b->knee_flex_l_deg);
    out->knee_flex_r_deg   = LERP(s, t, a->knee_flex_r_deg,   b->knee_flex_r_deg);
    out->ankle_flex_l_deg  = LERP(s, t, a->ankle_flex_l_deg,  b->ankle_flex_l_deg);
    out->ankle_flex_r_deg  = LERP(s, t, a->ankle_flex_r_deg,  b->ankle_flex_r_deg);
    out->l_arm_alpha_deg   = LERP(s, t, a->l_arm_alpha_deg,   b->l_arm_alpha_deg);
    out->l_arm_beta_deg    = LERP(s, t, a->l_arm_beta_deg,    b->l_arm_beta_deg);
    out->l_elbow_flex_deg  = LERP(s, t, a->l_elbow_flex_deg,  b->l_elbow_flex_deg);
    out->l_wrist_flex_deg  = LERP(s, t, a->l_wrist_flex_deg,  b->l_wrist_flex_deg);
}

/* ===== Forward kinematics ===== */

void pose_fk(const Pose *p, const Skeleton *s, PoseJoints *out) {
    /* Pelvis sits at com directly. */
    out->pelvis[0] = p->com[0];
    out->pelvis[1] = p->com[1];
    out->pelvis[2] = p->com[2];

    /* Spine direction: start as world +y, tilt by lateral roll
     * (about +x) then forward pitch (about +z). Order matters but
     * for small angles both work; we apply roll first so that
     * "lateral lean" is interpreted relative to the unlent torso. */
    double spine_dir[3] = { 0.0, 1.0, 0.0 };
    rot_x(p->spine_roll_deg,  spine_dir);
    rot_z(p->spine_pitch_deg, spine_dir);
    v3_add_scaled(out->pelvis, spine_dir, s->torso, out->spine_top);

    /* Torso landmark — midpoint of pelvis and spine_top. Aligned
     * with Varia/Kinect's TORSO joint. */
    out->torso[0] = (out->pelvis[0] + out->spine_top[0]) * 0.5;
    out->torso[1] = (out->pelvis[1] + out->spine_top[1]) * 0.5;
    out->torso[2] = (out->pelvis[2] + out->spine_top[2]) * 0.5;

    /* Head along the spine direction past the spine_top. */
    v3_add_scaled(out->spine_top, spine_dir,
                  s->neck_to_head + s->head_r, out->head_center);

    /* Hip line: starts as world +x, rotated about +y by hip_yaw.
     * Doesn't tilt with spine lean (the pelvis stays roughly level
     * relative to the floor — hips don't lean with the torso). */
    double hip_axis[3] = { 1.0, 0.0, 0.0 };
    rot_y(p->hip_yaw_deg, hip_axis);
    v3_add_scaled(out->pelvis, hip_axis,  s->hip_half, out->r_hip);
    v3_add_scaled(out->pelvis, hip_axis, -s->hip_half, out->l_hip);

    /* Legs: hip → knee (thigh, always straight down for now —
     * adding hip flex would let the leg swing forward but doubles
     * the leg DOF) → ankle (shin, tilted backward by knee_flex) →
     * toe (foot, in the body's forward direction at standing rest,
     * rotated downward by ankle_flex for plantarflexion).
     *
     * The "forward" direction the foot points in body-frame is
     * world +x rotated by hip_yaw — so the foot follows the hip's
     * facing, matching how a real player's foot stays planted as
     * the hips coil/uncoil only approximately. Good enough for the
     * stick-figure visualisation. */
    double down[3] = { 0.0, -1.0, 0.0 };
    /* Per-leg helper inlined twice — explicit is clearer than a
     * helper function with 8 args. */
    double forward_local[3] = { 1.0, 0.0, 0.0 };
    rot_y(p->hip_yaw_deg, forward_local);

    /* Right leg. */
    v3_add_scaled(out->r_hip, down, s->thigh, out->r_knee);
    double r_shin_dir[3] = { down[0], down[1], down[2] };
    rot_z(-p->knee_flex_r_deg, r_shin_dir);
    v3_add_scaled(out->r_knee, r_shin_dir, s->shin, out->r_ankle);
    double r_foot_dir[3] = { forward_local[0], forward_local[1], forward_local[2] };
    rot_z(-p->ankle_flex_r_deg, r_foot_dir);
    v3_add_scaled(out->r_ankle, r_foot_dir, s->foot, out->r_toe);

    /* Left leg. */
    v3_add_scaled(out->l_hip, down, s->thigh, out->l_knee);
    double l_shin_dir[3] = { down[0], down[1], down[2] };
    rot_z(-p->knee_flex_l_deg, l_shin_dir);
    v3_add_scaled(out->l_knee, l_shin_dir, s->shin, out->l_ankle);
    double l_foot_dir[3] = { forward_local[0], forward_local[1], forward_local[2] };
    rot_z(-p->ankle_flex_l_deg, l_foot_dir);
    v3_add_scaled(out->l_ankle, l_foot_dir, s->foot, out->l_toe);

    /* Shoulder line: like hips, rotated by shoulder_yaw, attached
     * to the spine_top. */
    double sh_axis[3] = { 1.0, 0.0, 0.0 };
    rot_y(p->shoulder_yaw_deg, sh_axis);
    v3_add_scaled(out->spine_top, sh_axis,  s->shoulder_half, out->r_shoulder);
    v3_add_scaled(out->spine_top, sh_axis, -s->shoulder_half, out->l_shoulder);

    /* Right upper arm. Spherical (alpha, beta) → direction in the
     * shoulder-yaw-aligned frame, then rotate the whole thing by
     * the shoulder yaw to land it in world frame.
     *
     *   alpha = angle from shoulder-down axis
     *   beta  = azimuth around shoulder-down (0 = forward, 90 = right)
     *
     * In the shoulder-yaw-aligned frame "forward" is body +x and
     * "right" is body +z. */
    double a_rad = p->r_arm_alpha_deg * M_PI / 180.0;
    double b_rad = p->r_arm_beta_deg  * M_PI / 180.0;
    double arm[3] = {
         sin(a_rad) * cos(b_rad),   /* forward */
        -cos(a_rad),                /* down (negative y) */
         sin(a_rad) * sin(b_rad),   /* right */
    };
    rot_y(p->shoulder_yaw_deg, arm);
    v3_add_scaled(out->r_shoulder, arm, s->upper_arm, out->r_elbow);

    /* Forearm: bend at elbow. The elbow plane is approximated by
     * rotating the upper-arm direction about the world +z axis;
     * positive elbow_flex bends the forearm "forward" relative to
     * a vertical hang. Loose for arbitrary upper-arm orientations
     * but produces the right shape across the forehand swing's
     * range of motion. */
    double forearm[3] = { arm[0], arm[1], arm[2] };
    rot_z(-p->r_elbow_flex_deg, forearm);
    v3_add_scaled(out->r_elbow, forearm, s->forearm, out->r_wrist);

    /* Racket from wrist. Wrist flex bends in the same plane as
     * elbow. */
    double racket_dir[3] = { forearm[0], forearm[1], forearm[2] };
    rot_z(-p->r_wrist_flex_deg, racket_dir);
    v3_add_scaled(out->r_wrist, racket_dir, s->racket, out->r_racket_tip);

    /* Left arm — mirrors the right-arm chain but with a flipped
     * z-component on the upper-arm direction so symmetric β values
     * produce symmetric poses. Elbow + wrist bend about the same
     * +z world axis as the right arm (loose anatomy, fine
     * visually). The forearm's hand position lives in l_hand and
     * the racket is conceptually absent on this side. */
    double la_rad = p->l_arm_alpha_deg * M_PI / 180.0;
    double lb_rad = p->l_arm_beta_deg  * M_PI / 180.0;
    double l_arm[3] = {
         sin(la_rad) * cos(lb_rad),
        -cos(la_rad),
        -sin(la_rad) * sin(lb_rad),     /* flipped z for left side */
    };
    rot_y(p->shoulder_yaw_deg, l_arm);
    v3_add_scaled(out->l_shoulder, l_arm, s->upper_arm, out->l_elbow);

    double l_forearm[3] = { l_arm[0], l_arm[1], l_arm[2] };
    rot_z(-p->l_elbow_flex_deg, l_forearm);
    /* l_hand is the wrist-end of the forearm. */
    double l_wrist_pos[3];
    v3_add_scaled(out->l_elbow, l_forearm, s->forearm, l_wrist_pos);
    out->l_hand[0] = l_wrist_pos[0];
    out->l_hand[1] = l_wrist_pos[1];
    out->l_hand[2] = l_wrist_pos[2];
    /* l_wrist_flex_deg is in the model but doesn't drive a racket;
     * we keep it for symmetry + future trophy/serve work where the
     * left hand might hold a tossed ball or grip a 2-hander. */
}

/* ===== Rate extraction ===== */

void pose_rig_derive_kc(const PoseRig *rig, PoseKineticChain *out) {
    double dt = rig->swing_duration_s;
    if (dt < 0.05) dt = 0.05;
    const Pose *p = &rig->prep;
    const Pose *c = &rig->contact;

    /* Hip / shoulder yaw deltas. Rates are magnitudes — the existing
     * physics encodes swing direction in HIP_DIR / SHO_DIR. */
    out->hip_rate_dps      = fabs(c->hip_yaw_deg      - p->hip_yaw_deg)      / dt;
    out->shoulder_rate_dps = fabs(c->shoulder_yaw_deg - p->shoulder_yaw_deg) / dt;

    /* Arm rate: total angular distance in (alpha, beta) sphere. The
     * Euclidean distance in the angle pair undercounts at the poles
     * but is good enough for the rate magnitude the strike model
     * consumes. */
    double da = c->r_arm_alpha_deg - p->r_arm_alpha_deg;
    double db = c->r_arm_beta_deg  - p->r_arm_beta_deg;
    out->arm_rate_dps = sqrt(da * da + db * db) / dt;

    /* Lean from the contact pose. */
    out->lean_forward_deg = c->spine_pitch_deg;
    out->lean_lateral_deg = c->spine_roll_deg;

    /* CoM velocity from Δposition. */
    out->body_v_forward  = (c->com[0] - p->com[0]) / dt;
    out->body_v_vertical = (c->com[1] - p->com[1]) / dt;
    out->body_v_lateral  = (c->com[2] - p->com[2]) / dt;
}

/* ===== Inverse FK =====
 *
 * Each chain analytically inverts the corresponding FK step. We
 * exploit the same simplifying assumptions FK uses (elbow + wrist
 * + knee + ankle all bend about world +z), which means each bend
 * angle reduces to a difference of xy-plane azimuths in the world
 * frame. */

/* atan2 → degrees. */
static inline double atan2_deg(double y, double x) {
    return atan2(y, x) * 180.0 / M_PI;
}

/* Normalise an angle to (−180, 180]. */
static inline double wrap_180(double deg) {
    while (deg >  180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

/* Solve α + β + elbow_flex (+ optional wrist_flex) for one arm
 * chain. `mirror_z` flips the z component of the upper-arm direction
 * before decomposition (left-arm mirror convention in FK). The
 * tip parameter is optional — pass NULL to leave wrist_flex
 * unwritten. */
static void arm_inverse(const double shoulder[3],
                        const double elbow[3],
                        const double wrist[3],
                        const double *tip,           /* may be NULL */
                        double shoulder_yaw_deg,
                        bool mirror_z,
                        double *out_alpha_deg,
                        double *out_beta_deg,
                        double *out_elbow_flex_deg,
                        double *out_wrist_flex_deg) {
    /* Upper arm direction in world. */
    double u[3] = {
        elbow[0] - shoulder[0],
        elbow[1] - shoulder[1],
        elbow[2] - shoulder[2],
    };
    double ul = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    if (ul < 1e-6) return;
    u[0] /= ul; u[1] /= ul; u[2] /= ul;

    /* Back-rotate by shoulder yaw to land in shoulder-local frame. */
    double ul_local[3] = { u[0], u[1], u[2] };
    rot_y(-shoulder_yaw_deg, ul_local);

    /* For the left arm, FK uses z_local = -sin(α)·sin(β); the right
     * arm uses +. Flip back here so α + β fall out symmetrically. */
    double zc = mirror_z ? -ul_local[2] : ul_local[2];

    *out_alpha_deg = acos(clampd(-ul_local[1], -1.0, 1.0)) * 180.0 / M_PI;
    *out_beta_deg  = atan2_deg(zc, ul_local[0]);

    /* Forearm direction in world. elbow_flex is the world-z-axis
     * rotation that takes upper → forearm. */
    double f[3] = {
        wrist[0] - elbow[0],
        wrist[1] - elbow[1],
        wrist[2] - elbow[2],
    };
    double fl = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (fl < 1e-6) return;
    f[0] /= fl; f[1] /= fl; f[2] /= fl;
    /* FK: forearm = rot_z(−elbow_flex) · upper. Inverse: rotation
     * angle from forearm.xy to upper.xy in the +z sense is the flex. */
    *out_elbow_flex_deg = wrap_180(
        atan2_deg(u[1], u[0]) - atan2_deg(f[1], f[0]));

    if (!tip || !out_wrist_flex_deg) return;
    double r[3] = {
        tip[0] - wrist[0],
        tip[1] - wrist[1],
        tip[2] - wrist[2],
    };
    double rl = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    if (rl < 1e-6) return;
    r[0] /= rl; r[1] /= rl; r[2] /= rl;
    *out_wrist_flex_deg = wrap_180(
        atan2_deg(f[1], f[0]) - atan2_deg(r[1], r[0]));
}

/* Solve knee_flex + ankle_flex for one leg. The thigh always points
 * straight down in FK so we don't need it; shin's tilt about +z gives
 * knee_flex, foot direction (in xy plane) less the hip-yaw azimuth
 * gives ankle_flex. */
static void leg_inverse(const double knee[3],
                        const double ankle[3],
                        const double toe[3],
                        double hip_yaw_deg,
                        double *out_knee_flex_deg,
                        double *out_ankle_flex_deg) {
    /* Shin direction. */
    double s[3] = {
        ankle[0] - knee[0],
        ankle[1] - knee[1],
        ankle[2] - knee[2],
    };
    double sl = sqrt(s[0]*s[0] + s[1]*s[1] + s[2]*s[2]);
    if (sl > 1e-6) {
        s[0] /= sl; s[1] /= sl; s[2] /= sl;
        /* FK: shin = rot_z(−knee_flex) on (0, −1, 0).
         * → shin = (−sin(kf), −cos(kf), 0). */
        *out_knee_flex_deg = atan2_deg(-s[0], -s[1]);
    }

    /* Foot direction. FK builds it as
     *   foot = rot_z(−ankle_flex) · forward_local
     * where forward_local = rot_y(hip_yaw) · (1, 0, 0)
     *                     = (cos(hip_yaw), 0, −sin(hip_yaw)).
     *
     * rot_z preserves z, so foot_xy = rot2d(−af) · (cos(hy), 0).
     * The signed angle in xy from forward to foot is −af. Factoring
     * cos(hip_yaw) into both atan2 arguments handles the sign of
     * forward_x correctly (atan2(s·a, s·b) flips by π when s<0,
     * matching the case where forward points toward −x). */
    double f[3] = {
        toe[0] - ankle[0],
        toe[1] - ankle[1],
        toe[2] - ankle[2],
    };
    double fl = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (fl > 1e-6) {
        f[0] /= fl; f[1] /= fl; f[2] /= fl;
        double fwd_x = cos(hip_yaw_deg * M_PI / 180.0);
        if (fabs(fwd_x) > 1e-6) {
            *out_ankle_flex_deg = -atan2_deg(fwd_x * f[1], fwd_x * f[0]);
        }
    }
}

void pose_from_joints(const PoseJoints *j, const Skeleton *s, Pose *out) {
    (void)s;

    /* Pelvis position. */
    out->com[0] = j->pelvis[0];
    out->com[1] = j->pelvis[1];
    out->com[2] = j->pelvis[2];

    /* Spine direction → pitch + roll. FK builds spine_dir as:
     *   spine = rot_z(pitch) · rot_x(roll) · (0, 1, 0)
     *         = (−sin(pitch)·cos(roll),
     *             cos(pitch)·cos(roll),
     *             sin(roll))
     * Invert: roll = asin(z), pitch = atan2(−x, y). */
    double sp[3] = {
        j->spine_top[0] - j->pelvis[0],
        j->spine_top[1] - j->pelvis[1],
        j->spine_top[2] - j->pelvis[2],
    };
    double spl = sqrt(sp[0]*sp[0] + sp[1]*sp[1] + sp[2]*sp[2]);
    if (spl > 1e-6) {
        sp[0] /= spl; sp[1] /= spl; sp[2] /= spl;
        out->spine_roll_deg  = asin(clampd(sp[2], -1.0, 1.0)) * 180.0 / M_PI;
        out->spine_pitch_deg = atan2_deg(-sp[0], sp[1]);
    } else {
        out->spine_roll_deg = 0.0;
        out->spine_pitch_deg = 0.0;
    }

    /* Hip yaw — direction from L-hip to R-hip. FK applies
     * rot_y(hip_yaw) to (1, 0, 0) so the hip line is
     * (cos, 0, −sin). Inverse: yaw = atan2(−z, x). */
    out->hip_yaw_deg = atan2_deg(
        -(j->r_hip[2] - j->l_hip[2]),
         (j->r_hip[0] - j->l_hip[0]));

    /* Shoulder yaw — same construction. */
    out->shoulder_yaw_deg = atan2_deg(
        -(j->r_shoulder[2] - j->l_shoulder[2]),
         (j->r_shoulder[0] - j->l_shoulder[0]));

    /* Right arm. */
    arm_inverse(j->r_shoulder, j->r_elbow, j->r_wrist, j->r_racket_tip,
                out->shoulder_yaw_deg, false,
                &out->r_arm_alpha_deg, &out->r_arm_beta_deg,
                &out->r_elbow_flex_deg, &out->r_wrist_flex_deg);

    /* Left arm — mirror_z flip. No racket → leave wrist flex at 0. */
    out->l_wrist_flex_deg = 0.0;
    arm_inverse(j->l_shoulder, j->l_elbow, j->l_hand, NULL,
                out->shoulder_yaw_deg, true,
                &out->l_arm_alpha_deg, &out->l_arm_beta_deg,
                &out->l_elbow_flex_deg, NULL);

    /* Legs. */
    leg_inverse(j->r_knee, j->r_ankle, j->r_toe, out->hip_yaw_deg,
                &out->knee_flex_r_deg, &out->ankle_flex_r_deg);
    leg_inverse(j->l_knee, j->l_ankle, j->l_toe, out->hip_yaw_deg,
                &out->knee_flex_l_deg, &out->ankle_flex_l_deg);
}

/* ===== Trajectory sampling ===== */

void pose_rig_sample_trajectory(const PoseRig *rig, int n_samples,
                                PoseJoints *out_joints) {
    if (n_samples < 1 || !out_joints) return;
    if (n_samples == 1) {
        Pose curr;
        pose_interp(&rig->prep, &rig->contact, 0.5, &curr);
        pose_fk(&curr, &rig->skel, &out_joints[0]);
        return;
    }
    for (int i = 0; i < n_samples; i++) {
        double t = (double)i / (double)(n_samples - 1);
        Pose curr;
        pose_interp(&rig->prep, &rig->contact, t, &curr);
        pose_fk(&curr, &rig->skel, &out_joints[i]);
    }
}

double pose_rig_racket_tip_velocity(const PoseJoints *joints,
                                    int n_samples, double duration_s,
                                    double out_v[3]) {
    if (n_samples < 2 || !joints || duration_s <= 0.0) {
        if (out_v) { out_v[0] = 0; out_v[1] = 0; out_v[2] = 0; }
        return 0.0;
    }
    double dt = duration_s / (double)(n_samples - 1);
    const double *t1 = joints[n_samples - 1].r_racket_tip;
    const double *t0 = joints[n_samples - 2].r_racket_tip;
    out_v[0] = (t1[0] - t0[0]) / dt;
    out_v[1] = (t1[1] - t0[1]) / dt;
    out_v[2] = (t1[2] - t0[2]) / dt;
    return sqrt(out_v[0]*out_v[0] + out_v[1]*out_v[1] + out_v[2]*out_v[2]);
}

/* ===== Racket-tip IK ===== */

void pose_rig_ik_racket(const PoseRig *rig, Pose *p,
                        const double target[3]) {
    /* FK the current pose to read the right shoulder position. The
     * shoulder is fixed by hip yaw + spine lean + shoulder yaw +
     * com — none of which IK touches — so the position stays put
     * after we write back the arm fields. */
    PoseJoints j;
    pose_fk(p, &rig->skel, &j);
    const double *S = j.r_shoulder;

    double D[3] = {
        target[0] - S[0],
        target[1] - S[1],
        target[2] - S[2],
    };
    double d2 = D[0]*D[0] + D[1]*D[1] + D[2]*D[2];
    double d  = sqrt(d2);

    const double L1 = rig->skel.upper_arm;
    const double L2 = rig->skel.forearm;
    const double L3 = rig->skel.racket;

    /* Effective elbow→tip distance, given the current wrist break. */
    double w_rad   = p->r_wrist_flex_deg * M_PI / 180.0;
    double L_eff_sq = L2 * L2 + L3 * L3 + 2.0 * L2 * L3 * cos(w_rad);
    double L_eff    = sqrt(L_eff_sq);

    /* Clamp d to the reachable annulus. Below L_min the chain
     * "folds"; above L_max it would have to stretch. Both clamp
     * to the boundary so the IK never reports unreachable. */
    double L_min = fabs(L1 - L_eff);
    double L_max = L1 + L_eff;
    if (d > L_max) {
        double s = L_max / d;
        D[0] *= s; D[1] *= s; D[2] *= s;
        d = L_max; d2 = L_max * L_max;
    } else if (d < L_min) {
        if (d < 1e-6) {
            /* Target == shoulder. Drop a default direction so the
             * IK doesn't divide by zero. */
            D[0] = L_min; D[1] = 0.0; D[2] = 0.0;
            d = L_min; d2 = L_min * L_min;
        } else {
            double s = L_min / d;
            D[0] *= s; D[1] *= s; D[2] *= s;
            d = L_min; d2 = L_min * L_min;
        }
    }

    double D_unit[3] = { D[0] / d, D[1] / d, D[2] / d };

    /* Two-link IK. α is the shoulder angle between upper arm and
     * S→T; β is the elbow interior angle between E→S and E→T. */
    double cos_alpha = (L1 * L1 + d2 - L_eff_sq) / (2.0 * L1 * d);
    double cos_beta  = (L1 * L1 + L_eff_sq - d2) / (2.0 * L1 * L_eff);
    cos_alpha = clampd(cos_alpha, -1.0, 1.0);
    cos_beta  = clampd(cos_beta,  -1.0, 1.0);
    double alpha    = acos(cos_alpha);
    double beta_ang = acos(cos_beta);

    /* Elbow bend direction: preserve the side the elbow currently
     * sits on. Decompose the current S→E vector into its component
     * perpendicular to D_unit; that perpendicular is the bend
     * direction. This makes IK with target = current_tip a no-op,
     * keeps small drags from teleporting the elbow across the body,
     * and stays smooth as the user drags. Falls back to projecting
     * world-down onto the plane (elbow drops below S→T) when the
     * current elbow happens to be colinear with S→T (rare). */
    double SE[3] = {
        j.r_elbow[0] - S[0],
        j.r_elbow[1] - S[1],
        j.r_elbow[2] - S[2],
    };
    double SE_along = SE[0]*D_unit[0] + SE[1]*D_unit[1] + SE[2]*D_unit[2];
    double bend[3] = {
        SE[0] - SE_along * D_unit[0],
        SE[1] - SE_along * D_unit[1],
        SE[2] - SE_along * D_unit[2],
    };
    double bend_len = sqrt(bend[0]*bend[0] + bend[1]*bend[1] + bend[2]*bend[2]);
    if (bend_len < 1e-4) {
        double down_dot = -D_unit[1];
        bend[0] = -down_dot * D_unit[0];
        bend[1] = -1.0 - down_dot * D_unit[1];
        bend[2] = -down_dot * D_unit[2];
        bend_len = sqrt(bend[0]*bend[0] + bend[1]*bend[1] + bend[2]*bend[2]);
        if (bend_len < 1e-4) {
            bend[0] = 0.0; bend[1] = 0.0; bend[2] = 1.0;
            bend_len = 1.0;
        }
    }
    bend[0] /= bend_len;
    bend[1] /= bend_len;
    bend[2] /= bend_len;

    /* Upper arm world direction = cos(α)·D_unit + sin(α)·bend. */
    double upper[3] = {
        cos(alpha) * D_unit[0] + sin(alpha) * bend[0],
        cos(alpha) * D_unit[1] + sin(alpha) * bend[1],
        cos(alpha) * D_unit[2] + sin(alpha) * bend[2],
    };

    /* Back-rotate by the current shoulder yaw to recover the
     * shoulder-local arm direction. Then decompose into (α_arm,
     * β_arm) per the Pose convention:
     *   α_arm = acos(-y_local)        (angle from shoulder-down)
     *   β_arm = atan2(z_local, x_local) (azimuth around shoulder-down) */
    double upper_local[3] = { upper[0], upper[1], upper[2] };
    rot_y(-p->shoulder_yaw_deg, upper_local);

    double new_alpha_rad = acos(clampd(-upper_local[1], -1.0, 1.0));
    double new_beta_rad  = atan2(upper_local[2], upper_local[0]);

    /* Elbow flex from the planar geometry. ψ is the angle between
     * the upper arm direction (S→E) and the elbow→tip direction
     * (E→T); φ accounts for the wrist break pulling the racket out
     * of pure forearm extension. */
    double psi = M_PI - beta_ang;
    double phi = atan2(L3 * sin(w_rad), L2 + L3 * cos(w_rad));
    double new_elbow_rad = psi - phi;

    p->r_arm_alpha_deg  = clampd(new_alpha_rad * 180.0 / M_PI,   0.0, 180.0);
    p->r_arm_beta_deg   = new_beta_rad * 180.0 / M_PI;
    p->r_elbow_flex_deg = clampd(new_elbow_rad * 180.0 / M_PI,   0.0, 160.0);
}
