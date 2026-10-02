# UR5 cable pickup and clip insertion

<!-- recorded-result:start -->
## Recorded result

**6/6 cables stayed inside the clip throughout the checked interval after withdrawal; 6/6 were inside during the final second.** The recording observes 3.98 seconds after gripper withdrawal; retention checks start 0.3 seconds after it clears. The complete strict validation did not pass.

The recorded maximum rod-joint gap is 0.390 mm (0.2 mm check), and maximum hinge-anchor error is 0.026 mm (0.05 mm check). The final lid angle is -9.37 degrees. Maximum sampled cable/clip penetration is 0.596 mm (0.2 mm check). Two cables temporarily fail the retention geometry check during release/withdrawal settling, before all six satisfy it after withdrawal. These are uncalibrated prototype results, not a validated hardware trajectory. Failed checks are preserved in [validation.json](validation.json) and [contact_validation.json](contact_validation.json). Retention uses the actual posed lid underside from the original STL; the closed-lid bounding envelope is also reported separately. [Robot/table clearance](robot_clearance_validation.json) is a sampled mesh check.

[Watch the recorded motion](ur5_cable_clip.mp4) · [Isaac-compatible visual playback](ur5_playback.usdc)
<!-- recorded-result:end -->

The connector starts seated and locked. A UR5 CB3 with a Robotiq 2F-85 approaches six loose cables, pinches the bundle, lifts it, carries it through the original spring clip, opens its fingers, lowers the open jaws to clear the hanging tails, withdraws sideways, and returns home.

The clip sits at the downstream table edge to give the fingers clearance beneath the bundle. The cable tails are free, with no distal connector or endpoint clamp. The cables settle on the table and hang over its edge before pickup. This changes the boundary conditions from the previous two-ended arch experiment.

## Simulation and control

- Six 457.2 mm (18 inch), 2 mm diameter cables, 91 capsule segments each. Explicit zero bend/twist rest curvature. Nominal bending parameter EI = 0.0011459155902616466 N m²; rod bending stiffness EI / segment length. Effective material density is 1000 kg/m³ (1.436 g per cable); capsule density is corrected for overlapping spherical ends. Stretch and shear stiffness are 1e6 N/m, with 0.1 N s/m damping; bend/twist stiffness is EI / segment length, and bend/twist damping 0.005729577951308233 × 0.005 / segment length.
- Newton 1.6 SolverVBD / AVBD, compliant ALM, 60 iterations, 3840 Hz. Newton generates collision contacts and the same solver handles rod and contact response. No MuJoCo or PhysX physics solver runs.
- UR5 nominal ROS-Industrial URDF dimensions and meshes. Six-joint numerical inverse kinematics generates a prescribed trajectory. Links are kinematic position-controlled collision bodies; motor torques, joint servo dynamics, robot compliance and hardware calibration are not simulated. Kinematic link mass/inertia values are placeholders, not UR5 hardware properties.
- Robotiq linkage angles satisfy the four-bar closure geometrically. SciPy `least_squares` handles UR5 inverse kinematics, `root` solves the four-bar closure, and `brentq` maps jaw gap to driver angle; these are kinematic calculations, while Newton VBD supplies the physics. The original finger collision boxes and mesh visuals move together. Jaw opening goes from 30 mm to 1.99 mm to pinch the complete bundle from above and below using the full pad depth. Pad friction coefficient is an assumed 1.0; other contacts use 0.5. These are uncalibrated values.
- Contact stiffness is 1e6 N/m and damping 100 for cables, clip, table and pads. Contact-generation gaps are 0.2 mm for cables/fixtures and 0.1 mm for pads; pad mesh margins are zero. Newton SAP broad phase uses a 100000-contact capacity, latest-contact matching and contact history. Solver settings include a 512-contact per-body buffer, friction epsilon 1e-4, rigid joint linear stiffness 1e7 and angular stiffness 1e5. Gravity is -9.81 m/s² along Z. Analytic cable inertias are restored after model finalization to avoid the default absolute inertia floor at this scale.
- **Contact-only grasp:** no cable-to-gripper joints, no cable teleporting, no direct cable grip forces. Contacts and friction transmit the robot motion to the cables.
- The connector is a fixed boundary in its already seated pose. This experiment assumes a locked connector and does not simulate its locking mechanism.
- Original unscaled clipTop/clipBottom STL geometry and the same fitted revolute hinge. The prior strengthened clip spring is retained: 10× original stiffness/damping, plus 0.03 N m constant closing preload. This is not a measured hardware spring.

The pickup uses a -60° wrist roll so the jaws close across the hanging cables. After gripping, the wrist rolls the bundle level during the lift. This avoids the roughly 60° forced bend caused by the earlier horizontal-jaw pickup.

A hardcoded acquisition controller tracks the same material section of the six cables using simulator state during approach, then freezes the pickup pose before closing. After closing, the controller keeps the jaw gap fixed and follows scripted insertion waypoints, including a 17 mm lengthwise pull to take up slack. The bundle is pulled lengthwise while still outside, then swept through the gate along an arc that maintains distance from the connector. The TCP height is computed so the straight chord from the seated connector to the gripper crosses the clip plane at world Z = 14 mm. With this connector height, the gripper itself travels around 5–6 mm high during insertion. This commands only the robot: Newton still solves cable bending, stretch and contact. The controller holds that feed height while the clip settles. During the bounded insertion window, a scripted entry check reads the cable centers at the clip plane. The check also transforms cable crossings into the moving lid frame and requires them to pass its entrance lip into the flat clamping area (lid-local X -2 to 23 mm). If any cable has not crossed into that area, the robot advances farther sideways at 12 mm/s for up to 2.5 seconds, with a -40 mm world-X travel limit. The sequence continues after that window even if the entry condition is unmet; the recorded metadata reports whether it was reached. The jaws then open to 85 mm. After a short pause, the open gripper lowers by 27 mm to clear the drooping tails, withdraws along +X, and returns home. Cable retention is observed after withdrawal. This check uses exact simulator state, not perception. Axial cable sliding during this pull is measured separately from slip during the initial lift. Robot-to-robot collision pairs are filtered; arm self-collision avoidance is not certified. This is perfect-state feedback, not camera perception or a learned policy. The motion data includes the initial 1.5-second settling interval. The preview and default USD scene start after it, with the connector already seated and the tails hanging.

## Files and reproduction

`simulate.py` generates `motion.npz` and `motion.json`; `validate.py` writes `validation.json`; `render.py` generates the MP4 and stills. `export_playback.py` writes an Isaac-compatible geometry scene and recorded USD playback. USD is a visualization of Newton results, with no active PhysX simulation.

From the repository root, use the supplied environment and run:

```bash
python tools/run.py ur5-clip --output runs/ur5-clip-01 --steps simulate render usd
```

To run the strict diagnostic checks, use `python validate.py`, `python check_clip_contacts.py`, and `python check_robot_clearance.py` from the copied run directory. These return nonzero when their stated tolerances fail; inspect the reports rather than treating a completed video as validation.

Rendering/export requires `pycollada` to read the UR5 DAE meshes. The customer reference video is not included. This is a qualitative simulation, not a validated real-robot trajectory.

## Robot asset sources and licenses

UR5 assets: [ROS-Industrial universal_robot](https://github.com/ros-industrial/universal_robot/tree/39ad110d8f2e8f66856a201cca88aa7a7025e3eb/ur_description), BSD-3-Clause. The generated `robot_assets/ur5.urdf` uses the default UR5 nominal configuration. Original license: `robot_assets/ur/ur_description/LICENSE`.

Robotiq 2F-85 assets: [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/robotiq_2f85), BSD-2-Clause. Original XML is used as a source of geometry and kinematic dimensions, not as a MuJoCo simulation. Original license: `robot_assets/robotiq/robotiq_2f85/LICENSE`.

`robot_assets/provenance.json` records pinned source commits. Every downloaded source file was checked against its upstream Git blob hash.

## Diagnostic history

`diagnostic_history.json` records earlier unsuccessful trials; their source snapshots are in `diagnostics/`. To run a snapshot, copy the entire experiment to a fresh directory, replace its `simulate.py` with the chosen snapshot, and run there. Those snapshots are historical diagnostics and retain their original limitations. Early UR5 layouts used a guide curve shorter than the requested cable length, compressing the last initial segment. The current layout covers the full material length and checks segment spacing in `initialization_check.json`. It also moves the initial bundle farther outside the clip corner and maintains connector-to-grip distance during the transverse sweep.

The [entry-criterion audit](entry_criterion_audit.json) and [cross-section figure](entry_criterion.png) show why the earlier fixed-position entry test was too permissive: three cables were under the raised outside flare, where lid closure could push them back out. The full earlier recording is `shallow_entry_motion.npz`, with its failed validation reports and matching source snapshot.
