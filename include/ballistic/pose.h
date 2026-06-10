#ifndef BALLISTIC_POSE_H
#define BALLISTIC_POSE_H

/* Biomechanics half of balistic — the poseable stick-figure rig, forward +
 * inverse kinematics, and swing sampling. Pure C (math only). The on-disk
 * trajectory JSON I/O is a consumer-side adapter (it needs a JSON lib), so it
 * lives in the app, not here — see rakija's body_rig_json.{c,h}. */

/* Poseable stick-figure rig for swing definition. The kinetic-chain
 * scalars (hip_rate_dps, shoulder_rate_dps, …) become *derived*
 * outputs of two poses + a swing duration; users define a swing by
 * dragging joints in a Prep pose and a Contact pose, and the
 * physics rates fall out as Δangle / Δtime.
 *
 * For now (phase 1 + 2 of the migration) this file owns the data
 * model and the forward-kinematics that resolves a Pose into world-
 * frame joint positions. Drawing + interaction live in setup_body.c
 * and pose-driven physics wiring lands in a later phase. */

/* A single instant of the swinger's posture. All angles in degrees,
 * positions in metres (world frame, +x downrange, +y up, +z player's
 * right). */
typedef struct Pose {
    /* Pelvis position. The body floats from here; in v1 we don't
     * model legs anchoring to the ground, so com[y] sets the hip
     * height directly. */
    double com[3];

    /* Spine lean. Forward lean (pitch) tips the spine's top toward
     * +x; lateral lean (roll) toward +z. Pitch is positive forward
     * because that's the coach's "lean into the shot" direction. */
    double spine_pitch_deg;
    double spine_roll_deg;

    /* Hip + shoulder rotation about world vertical. Tracked
     * independently so the X-factor (shoulder − hip) emerges from
     * Δposes rather than being hand-tuned. 0 = facing +x downrange,
     * positive = rotated counterclockwise viewed from above (a
     * right-hander's loaded forehand). */
    double hip_yaw_deg;
    double shoulder_yaw_deg;

    /* Right-arm chain. Upper-arm direction in shoulder-local frame
     * uses spherical coords: alpha is the angle from "shoulder
     * down" (0 = arm hanging at side, 90 = horizontal, 180 = above
     * head). beta is the azimuth around that down axis (0 = arm
     * forward, 90 = arm to the right side of body, 180 = behind,
     * -90 = across body). */
    double r_arm_alpha_deg;
    double r_arm_beta_deg;
    /* Elbow flexion (0 = straight, 90 = right angle). */
    double r_elbow_flex_deg;
    /* Wrist flexion (sagittal — racket lays back or comes through). */
    double r_wrist_flex_deg;

    /* Leg articulation. Each leg has a knee flex (sagittal bend —
     * shin tilts backward as the angle grows, heel toward buttocks)
     * and an ankle flex (plantarflexion — positive tilts the toe
     * downward, the push-off direction). Both legs are tracked
     * independently because real loading rarely splits evenly
     * between feet (e.g. open-stance forehand loads the back leg).
     *
     * Knee flex: 0 = straight, ~25° = athletic loaded stance,
     * 90° = deep squat. Clamped 0..120 in interactive edits.
     * Ankle flex: 0 = neutral (foot perpendicular to shin),
     * positive = plantarflexed toes-down. Range roughly -20 to +30. */
    double knee_flex_l_deg;
    double knee_flex_r_deg;
    double ankle_flex_l_deg;
    double ankle_flex_r_deg;

    /* Left arm chain. Mirrors the R-arm spherical convention but
     * with a flipped z-component in FK so symmetric (α, β) values
     * produce symmetric (mirror-image) poses across the sagittal
     * plane. For a right-handed forehand the left arm extends
     * toward the incoming ball at prep (β ≈ 90° in this frame =
     * out toward the player's left side) and folds across the body
     * at contact.
     *
     * Doesn't drive any physics — purely visual / balance — so the
     * strike model ignores it entirely. Left arm wrist_flex is
     * included for symmetry (no racket but the joint exists). */
    double l_arm_alpha_deg;
    double l_arm_beta_deg;
    double l_elbow_flex_deg;
    double l_wrist_flex_deg;
} Pose;

/* Bone lengths in metres, derived from person_height. */
typedef struct Skeleton {
    double thigh;          /* hip to knee */
    double shin;           /* knee to ankle */
    double foot;           /* ankle to toe */
    double torso;          /* hip to base of neck */
    double neck_to_head;   /* base of neck to head centre */
    double head_r;         /* head radius (drawing only) */
    double upper_arm;      /* shoulder to elbow */
    double forearm;        /* elbow to wrist (incl. hand) */
    double shoulder_half;  /* spine_top to one shoulder */
    double hip_half;       /* pelvis to one hip */
    double racket;         /* wrist to racket tip (fixed 27" by default) */
} Skeleton;

/* World-frame joint positions, the output of pose_fk. Joint set
 * aligned with Varia 2018's THETIS refined-skeleton 15-joint schema
 * (NECK = spine_top, TORSO = torso, FOOT = ankle), plus rakija
 * extras (pelvis as root, toes for FK completeness, r_racket_tip
 * extrapolated). */
typedef struct PoseJoints {
    double pelvis[3];
    double spine_top[3];    /* Varia NECK — midpoint of shoulders. */
    double torso[3];        /* Varia TORSO — mid-trunk landmark
                             * (midpoint of spine_top and pelvis). */
    double head_center[3];
    double l_hip[3], r_hip[3];
    double l_knee[3], r_knee[3];
    double l_ankle[3], r_ankle[3];
    double l_toe[3], r_toe[3];
    double l_shoulder[3], r_shoulder[3];
    double r_elbow[3], r_wrist[3], r_racket_tip[3];
    double l_elbow[3], l_hand[3];
} PoseJoints;

/* Two-pose swing definition + scrub state. */
typedef struct PoseRig {
    Pose     prep;
    Pose     contact;
    double   scrub;             /* 0..1 — 0 = prep, 1 = contact */
    double   swing_duration_s;  /* time between prep and contact */
    Skeleton skel;
} PoseRig;

/* Drillis-Contini-ish body proportions. h = person height in metres. */
void skeleton_from_height(Skeleton *s, double h);

/* Named-shot pose presets. Each pair loads a sensible Prep +
 * Contact for the named shot; users can then drag joints to tune.
 * Right-handed conventions throughout (sorry, lefties — mirror via
 * spine_yaw signs in the meantime). */
void pose_default_prep_forehand     (Pose *p);
void pose_default_contact_forehand  (Pose *p);
void pose_default_prep_backhand_1h  (Pose *p);
void pose_default_contact_backhand_1h(Pose *p);
void pose_default_prep_serve_flat   (Pose *p);
void pose_default_contact_serve_flat(Pose *p);

/* Preset identifiers + bulk load. Sets both rig.prep and rig.contact
 * at once; doesn't touch scrub, swing_duration_s, or skel. */
typedef enum PosePreset {
    POSE_PRESET_FOREHAND = 0,
    POSE_PRESET_BACKHAND_1H,
    POSE_PRESET_SERVE_FLAT,
} PosePreset;
void pose_preset_load(PoseRig *rig, PosePreset preset);

/* Linear interpolation between two poses. t clamps to [0, 1]. */
void pose_interp(const Pose *a, const Pose *b, double t, Pose *out);

/* Forward kinematics — resolve a Pose into world-frame joint
 * positions given the skeleton. The pose's com[] sets the pelvis;
 * spine direction is the body-local +y axis tilted by spine_roll
 * (about +x) and spine_pitch (about +z); hips/shoulders pivot
 * independently about the world vertical via hip_yaw / shoulder_yaw.
 *
 * Legs are simplified: feet hang straight down from each hip. (Phase
 * 1 v1 — leg articulation belongs to a later phase if useful.)
 * Left arm hangs relaxed from the left shoulder.
 *
 * The right arm uses the spherical (alpha, beta) → direction mapping
 * documented on Pose; elbow flex bends the forearm in the local
 * elbow plane (approximation: rotation around the body-frame +z
 * axis), and wrist flex bends the racket from the forearm in the
 * same plane. The elbow-plane approximation is anatomically loose
 * but stays sensible across the swing's range of motion. */
void pose_fk(const Pose *p, const Skeleton *s, PoseJoints *out);

/* Extracted kinetic-chain scalars derived from Δ(prep → contact)
 * divided by rig.swing_duration_s. Used by phase 5 to push the
 * pose-driven defaults into AppState's kc_* spinbuttons. */
typedef struct PoseKineticChain {
    double hip_rate_dps;
    double shoulder_rate_dps;
    double arm_rate_dps;
    double lean_forward_deg;   /* spine_pitch at contact */
    double lean_lateral_deg;   /* spine_roll at contact */
    double body_v_forward;
    double body_v_vertical;
    double body_v_lateral;
} PoseKineticChain;

/* Compute the rates + lean + body velocity that match the rig's two
 * poses. Rates are magnitudes (|Δ| / Δt) because the existing
 * kinetic-chain physics treats them as signed magnitudes whose
 * direction is fixed by HIP_DIR / SHO_DIR / arm-direction
 * constants. Lean is taken from the *contact* pose because that's
 * the body posture at impact. CoM velocity is the simple linear
 * difference over the swing window. swing_duration_s is clamped to
 * a minimum (50 ms) so a zero-duration rig doesn't divide-by-zero. */
void pose_rig_derive_kc(const PoseRig *rig, PoseKineticChain *out);

/* Sample N joint poses uniformly along the prep→contact interpolation
 * in world coordinates. `out_joints` must point at an array of at
 * least n_samples PoseJoints; entry i corresponds to time
 *   t_i = (i / (n_samples − 1)) · rig.swing_duration_s
 * with i=0 at prep and i=n_samples−1 at contact.
 *
 * The resulting array is a per-joint world-coordinate time series —
 * the same data shape an external mocap source (GPS / IMU / video
 * pose-estimation) would land in, so future capture-from-real-swing
 * paths can drop into the same downstream code (trails, velocity
 * extraction, possibly velocity-vector feed into the strike model). */
void pose_rig_sample_trajectory(const PoseRig *rig, int n_samples,
                                PoseJoints *out_joints);

/* World-frame velocity of the right-racket tip at contact, computed
 * as a backward difference between the last two trail samples:
 *   v_tip = (tip[N−1] − tip[N−2]) / Δt
 * where Δt = swing_duration_s / (N − 1). Writes the 3-vector to
 * out_v (m/s) and returns its magnitude. N must be ≥ 2; with N=1
 * out_v is zeroed and the return value is 0. */
double pose_rig_racket_tip_velocity(const PoseJoints *joints,
                                    int n_samples, double duration_s,
                                    double out_v[3]);

/* Pose-driven swing: turn a body pose-pair (prep → contact over
 * rig->swing_duration_s) into the swing knobs the serve/groundstroke pipeline
 * consumes, so the swing's pace + topspin come from how the body uncoils rather
 * than from raw sliders. Samples the prep→contact trajectory, reads the racket
 * tip's velocity at contact, and writes:
 *   *out_swing_speed_mps — the tip speed (faster / shorter-duration uncoil →
 *                          more pace),
 *   *out_plane_elev_deg  — the tip-velocity elevation (a low-to-high path → a
 *                          positive angle → topspin in the strike model).
 * Either out pointer may be NULL. A caller writes these into a ServeParams
 * (swing_speed_mps + plane_elev_deg) before serve_simulate — no engine change. */
void pose_drive_swing(const PoseRig *rig, double *out_swing_speed_mps,
                      double *out_plane_elev_deg);

/* Inverse FK — back-solve a Pose from world-frame joint positions.
 * Each chain solves analytically:
 *
 *   pelvis           → com[]
 *   spine_top        → spine_pitch + spine_roll
 *   l_hip / r_hip    → hip_yaw         (from the cross-line dir)
 *   l_shoulder /
 *     r_shoulder     → shoulder_yaw    (from the cross-line dir)
 *   R-arm chain      → r_arm_α/β, r_elbow_flex, r_wrist_flex
 *                      (decomposed in the shoulder-local frame; the
 *                      elbow + wrist bends invert via xy-angle
 *                      differences in the world frame)
 *   L-arm chain      → l_arm_α/β, l_elbow_flex  (l_wrist_flex left
 *                      at 0 — no racket on this side to anchor it)
 *   per-leg knee +
 *     ankle          → knee_flex, ankle_flex
 *
 * Joint positions must be in the world frame the FK uses. The
 * Skeleton parameter is unused at the moment (bone-length factors
 * cancel out in the direction calculations) but is part of the
 * signature for future per-bone normalisation. Round-trip is exact
 * for poses pose_fk produced, modulo trig roundoff. */
void pose_from_joints(const PoseJoints *j, const Skeleton *s, Pose *out);

/* The on-disk JSON trajectory format (sampled swings — what kadar/terka and any
 * external mocap source write) is read/written by a consumer-side adapter that
 * pulls in a JSON library: rakija's `pose_rig_trajectory_{load,save}_json` in
 * body_rig_json.c. It (de)serialises arrays of the PoseJoints above; the schema
 * is documented there. balistic stays JSON-free. */

/* Analytic 3-link IK for the R-arm chain.
 *
 * Solves for r_arm_alpha_deg, r_arm_beta_deg, r_elbow_flex_deg such
 * that the racket tip lands at `target` (world frame), keeping the
 * current r_wrist_flex_deg. The shoulder position is whatever the
 * pose currently FK's to — we read it once before solving. Out-of-
 * reach targets clamp to the closest reachable distance.
 *
 * Why analytic + not full IK on all 4 DOF: the elbow and wrist
 * bends both rotate in the same world-+z plane in our FK, so the
 * chain elbow→wrist→tip is planar. Effective elbow→tip distance
 * L_eff depends only on wrist_flex; with that fixed the problem
 * reduces to standard 2-link IK plus a closed-form recovery of
 * elbow_flex from the elbow interior angle.
 *
 * Elbow bend direction prefers the in-plane perpendicular to
 * (target−shoulder) that points downward in world space — keeps
 * the elbow below the S→T line, which is the natural tennis arm
 * posture. Falls back to +z when target is directly overhead. */
void pose_rig_ik_racket(const PoseRig *rig, Pose *p,
                        const double target[3]);

#endif
