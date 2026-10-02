# UR5 cable insertion on an interior tabletop, with two end connectors

<!-- recorded-result:start -->
## Recorded result

**5/6 cables satisfy the retention criterion throughout the checked interval after withdrawal; 5/6 during the final second.** Observation extends 3.98 seconds after withdrawal, with retention checks starting 0.3 seconds after the gripper clears. The final lid angle is -4.35°. The complete strict checks **did not pass**. All six cables were lifted, but the outer blue cable remains at the entrance; the bounded entry feedback did not reach its all-six condition.

Maximum rod-joint gap: 0.093 mm (0.2 mm check). Maximum distal endpoint attachment error: 0.001 mm (0.1 mm check). Maximum sampled cable/clip penetration: 0.738 mm (0.2 mm check). Maximum sampled cable/gripper penetration: 0.779 mm (0.2 mm check). Maximum sampled robot/table penetration: 0.000 mm. The second connector moved 99.0 mm from its initial pose.

[Video](ur5_cable_clip.mp4) · [Isaac-compatible playback](ur5_playback.usdc) · [Task validation](validation.json) · [Clip contacts](contact_validation.json) · [Gripper contacts](gripper_contact_validation.json) · [Table clearance](robot_clearance_validation.json)

These are prototype, uncalibrated simulation results. Numerical failures are preserved, not waived.
<!-- recorded-result:end -->

The seated connector stays fixed. All six other cable ends attach to one **free dynamic connector** (15 g), which can slide, rotate, and lift under cable tension and contact with the table. It is not pinned to the table or moved by a controller.

The table now extends from X = −310 to +300 mm and Y = −120 to +520 mm. The original unscaled clip remains at X = −18 mm, Y = 215 mm on solid tabletop, 242 mm from the nearest edge. This preserves the seated-connector-to-clip spacing while removing the previous edge clearance. All table geometry is shared by simulation, rendering, USD export, and clearance checks through `scene.py`.

The UR5 CB3 and Robotiq 2F-85 approach from above. The pads close sideways across the six-cable bundle from 30 mm to 5.99 mm. The commanded TCP lies 1.05 mm above the physical pad tips so the fingers can grip cables resting on the table without going through it. The wrist aligns with the initial bundle direction (−16.7° yaw) before closing; the narrower grasp allows the cables to bunch into multiple rows. The grasp is transmitted by contact and friction, including contacts between adjacent cables; interior cables need not touch both pads directly. There are no cable-to-gripper attachments or direct cable manipulation forces.

**Fingertip modification:** the stock flat pads slipped during tabletop pickup. This variant adds visible, physical wedge support lips to the original pads: 22 mm wide, projecting 2.5 mm inward, with a 0.10 mm leading edge and 1 mm back thickness. They close beneath the cables and support the bundle geometrically. At the 5.99 mm pad gap, the two lips leave a 0.99 mm gap, smaller than a cable diameter. The insert is also exported as `fingertip_support.stl` in the pad-local frame. They use the same contact material as the pads and are included in the collision model, video, USD, and robot/table clearance audit. This is a modified Robotiq fingertip setup, not a stock-gripper performance claim.

After pickup, the robot lifts the bundle, takes up slack with a 17 mm lengthwise motion, and feeds it sideways through the clip. The insertion target maintains the connector-to-grip distance and aims its straight chord through world Z = 14 mm at the clip center plane. Newton determines the actual curved cable shape. A bounded 2.5-second feedback window checks entry relative to the moving lid and can advance farther sideways; the sequence continues when the window ends even if that criterion is unmet. The jaws open to 85 mm, lift vertically by 100 mm, and return home. The final interval observes the cables without gripper support.

## Physics and control

- Six 457.2 mm cables, 2 mm diameter, 91 capsule segments each; zero material rest curvature and twist.
- Effective density 1000 kg/m³, approximately 1.436 g per cable. Capsule density is corrected for overlapping spherical ends.
- Bending rigidity EI = 0.0011459155902616466 N m²; segment bending/twist stiffness EI / segment length. Bending/twist damping = 0.005729577951308233 × 0.005 / segment length.
- Stretch/shear stiffness 1e6 N/m; damping 0.1 N s/m.
- Newton 1.6 SolverVBD / AVBD with compliant ALM, 80 iterations at 3840 Hz. Native Newton collision detection and contact response. No PhysX or MuJoCo physics solver is used.
- Contact stiffness 1e6 N/m, damping 100; friction 0.5 for cables, fixtures and table, 1.0 for gripper pads. Generation gap 0.2 mm for cables/fixtures and 0.1 mm for pads.
- Same uncalibrated preloaded clip spring as the previous robot experiment: stiffness 0.114591559 N m/rad, damping 0.002864789 N m s/rad, closing preload 0.03 N m, angular limits −75 to 0 degrees.
- The second connector uses the same approximate plug geometry, with a box inertia for its 15 g mass. Each distal cable endpoint is fixed to it; the connector body is dynamic.
- Robot links follow prescribed URDF forward kinematics. SciPy inverse kinematics and four-bar linkage calculations generate their poses. Motor torques, servo dynamics, grip force limits, and robot compliance are not simulated. Robot self-collision pairs are filtered.
- Approach and entry feedback read exact simulator state; this is not perception or a learned policy.

## Reproduction and checks

From the repository root:

```bash
python tools/run.py ur5-interior-clip --output runs/ur5-interior-01 --steps simulate render usd
```

In the copied run directory, run `python validate.py`, `python check_clip_contacts.py`, `python check_gripper_contacts.py`, and `python check_robot_clearance.py` for strict diagnostics. Failed checks return nonzero and remain visible in their JSON reports. A completed animation alone does not establish valid contact physics.

Validation includes both end attachments, cable pickup, robot limits and FK, sampled tabletop clearance, actual lid-relative retention, and the clip's distance from the table edges. Retention is a sampled center-plane geometric criterion, not proof of long-term retention or a fully closed latch. USD files contain geometry and recorded Newton transforms for visual playback in Isaac Sim, with no active PhysX simulation. The movie replays the same recorded poses in Newton's offscreen viewer.

## Assets

The clip uses the original unscaled `clipTop.stl` and `clipBottom.stl`. Both end connectors use the previously reconstructed approximate geometry; no customer video is included.

UR5 CB3 source: [ROS-Industrial universal_robot](https://github.com/ros-industrial/universal_robot/tree/39ad110d8f2e8f66856a201cca88aa7a7025e3eb/ur_description), BSD-3-Clause. Robotiq source: [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/4d038b3feae26ec82b46a4d586379114012a8ac7/robotiq_2f85), BSD-2-Clause. The latter supplies geometry and linkage dimensions only. Pinned commits, original licenses, and verified source provenance are included under `robot_assets/`.

`pickup_diagnostics.json` records the earlier stock-pad trials that slipped during lift. The final setup adds the physical support lips described above.
