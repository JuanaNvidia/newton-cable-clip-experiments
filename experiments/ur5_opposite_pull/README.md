# UR5 insertion with a conditional pull from the other side of the clip

<!-- recorded-result:start -->
## Recorded result

The first insertion left **6/6** cables inside at the recovery check. Opposite-side recovery was **not needed (the fallback was not exercised in this recording)**; actual lateral pull was **0.00 mm**. The final checked interval after withdrawal retained **6/6 cables**, and **6/6** satisfied the criterion throughout the last second. Observation extends 4.00 seconds after final withdrawal, with checks starting 0.3 seconds after clearance. The final lid angle is 0.00°.

The complete strict checks **did not pass**. Maximum rod-joint gap is 0.170 mm (0.2 mm check); distal attachment error is 0.001 mm (0.1 mm check). Maximum sampled penetration is 0.678 mm at the clip and 0.789 mm at the gripper (0.2 mm checks). Sampled robot/table penetration is 0.000 mm.

[Video](ur5_cable_clip.mp4) · [Isaac-compatible playback](ur5_playback.usdc) · [Task validation](validation.json) · [Clip contacts](contact_validation.json) · [Gripper contacts](gripper_contact_validation.json) · [Table clearance](robot_clearance_validation.json)

These are uncalibrated prototype results; geometric entry and retained cables do not waive contact or other numerical failures.
<!-- recorded-result:end -->

This variant repeats the interior-table, two-connector experiment, then checks the cables after the first release and withdrawal. If any cable fails the retention check, the UR5 regrips the bundle **on the seated-connector side of the clip** and pulls sideways toward the channel before releasing again.

## Sequence

1. Repeat the first pickup and insertion with the same six 457.2 mm × 2 mm cables, unscaled clip, table, spring, robot and physical fingertip supports.
2. At 18 seconds, check the six cable crossings at the clip center plane against the original clip geometry. The controller and validator use the same geometric criterion. If all six are inside, skip the second grasp and observe at home until 22 seconds.
3. Otherwise, approach a second grasp at approximately world Y = 135 mm, 65 mm before the clip center plane. Track the selected cable material sections during approach, orient the wrist to the local bundle direction, then freeze the grasp target before closing.
4. Close the Robotiq pad gap from 40 mm to 5.99 mm over 0.8 seconds. The existing wedge support lips capture the bundle through collision contact. Lift and align it; no cable-to-gripper constraints or direct cable forces are introduced.
5. Pull in negative world X at up to 4 mm/s, with a 24 mm travel limit and a six-second window. Maintain the gripper's distance from the seated connector. Stop advancing whenever all six satisfy the retention criterion; resume if that condition is lost. Record an entry event after 0.25 seconds continuously inside. An entry event is not proof of retention after release.
6. Open to 85 mm, lift 100 mm clear of the cables, return home, and observe until 37 seconds when recovery was required. The bounded sequence continues to release even if the recovery check is never satisfied; results report this explicitly.

The first connector remains fixed in its seated pose. The second connector is a free dynamic 15 g body, connected to all six distal cable ends and able to slide, rotate and lift. The clip sits on solid tabletop, 242 mm from its nearest edge. The robot is driven by kinematic joint commands, not simulated motor torques. Controller feedback uses exact simulator state, not perception.

## Physics and gripper geometry

Newton 1.6 SolverVBD / AVBD with compliant ALM runs 80 iterations at 3840 Hz. Newton supplies cable dynamics, collision detection, spring-joint response and contact response. SciPy solves robot inverse kinematics and Robotiq linkage geometry; no PhysX or MuJoCo physics engine runs.

Each cable has 91 capsule segments, radius 1 mm, effective density 1000 kg/m³ and approximately 1.436 g mass. Material rest curvature and twist are zero. EI = 0.0011459155902616466 N m²; segment bend/twist stiffness is EI divided by segment length, with damping 0.005729577951308233 × 0.005 divided by segment length. Stretch/shear stiffness is 1e6 N/m and damping 0.1 N s/m.

Contact stiffness is 1e6 N/m, damping 100, fixture/cable friction 0.5, and pad/support friction 1.0. Contact-generation gaps are 0.2 mm for cables/fixtures and 0.1 mm for pads. The uncalibrated preloaded clip spring remains 0.114591559 N m/rad, damping 0.002864789 N m s/rad and constant closing preload 0.03 N m, with −75..0° limits.

The Robotiq retains the **physical support lips from the previous experiment**: 22 mm wide, 2.5 mm inward projection, 0.10 mm leading-edge thickness and 1 mm back thickness. The 5.99 mm pad gap leaves 0.99 mm between the lips. The TCP lies 1.05 mm above the original pad tips. The support geometry participates in simulation, video, USD and clearance audits, and is exported as `fingertip_support.stl`. This is a modified fingertip setup, not a stock-gripper performance claim. Grip-force limits, actuator compliance and robot self-collision avoidance are not validated.

## Reproduce and inspect

From the repository root:

```bash
python tools/run.py ur5-opposite-pull --output runs/ur5-opposite-01 --steps simulate render usd
```

In the copied run directory, run `python validate.py`, `python check_clip_contacts.py`, `python check_gripper_contacts.py`, and `python check_robot_clearance.py`. Strict failures return nonzero and are preserved in JSON reports.

`retention.py` defines the shared geometry check: a cable center-plane crossing must lie within bottom-local X = 5..28 mm, above local Z = 11.8 mm, and at least 0.8 mm below the actual posed lid underside. This is a sampled geometric check, not proof of long-term retention or a closed latch. Separate signed-distance checks report cable/clip and cable/gripper overlap. The recording includes the per-frame feedback decisions and selected second-grasp cable indices (−1 when recovery is skipped). The reference motion was finalized byte-for-byte from its completed 22-second solver checkpoint once the no-recovery observation succeeded; no poses were edited or restarted. The simulator includes the equivalent automatic early completion.

The MP4 replays solved poses through Newton's offscreen viewer. USD files contain those recorded transforms for visual playback in Isaac Sim, with no active PhysX. Source video or private customer messages are not included.

## Assets

Original unscaled `clipTop.stl` and `clipBottom.stl`; approximate reconstructed connector geometry; procedural tabletop and fingertip support inserts. UR5 CB3 assets come from [ROS-Industrial universal_robot](https://github.com/ros-industrial/universal_robot/tree/39ad110d8f2e8f66856a201cca88aa7a7025e3eb/ur_description), BSD-3-Clause. Robotiq assets come from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/robotiq_2f85), BSD-2-Clause, as geometry and linkage dimensions only. Original licenses and pinned provenance are included under `robot_assets/`.
