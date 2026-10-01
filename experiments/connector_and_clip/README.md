# Longer cables: connector insertion followed by spring-clip routing

This combines the corrected fixed-receiver insertion with the original clipTop/clipBottom cable-management clip. Six cables are now **457.2 mm (18 inches)** long, up from 304.8 mm. Diameter remains **2 mm**, with 2.5 mm connector pitch and unchanged bending rigidity. The original clip is not scaled. Two spring settings are included: the original spring and a stronger, preloaded variant (the default).

The hand target approaches at 25 degrees; the dynamic white plug may rotate earlier under cable load. The plug approaches at an angle and rotates into the fixed black receiver. With the longer bundle supported, it finishes seating as the grips lower, before the lateral pull through the clip. Two cable grip regions support the bundle throughout connector insertion. A supporting hand remains on the plug while the cable grips lower the bundle beside the clip and pull it through the spring-loaded gate. Cable grips release at 11.2 seconds; connector support releases at 12.2 seconds. The final 1.8 seconds show passive settling. **The stronger variant retains all six cables, but the lid settles about 3.4 degrees open. It does not fully close.** The original spring reopens about 31.5 degrees.

## What is simulated and what is assumed

All dynamic bodies, cable rods, contacts and the original clip revolute joint are solved together by Newton 1.6 SolverVBD/AVBD with compliant ALM. Warp 1.17 runs the solver on CUDA. The black receiver is fixed throughout and has no hinge. The white plug and the spring clip lid are dynamic. Grips apply bounded forces to cable segments; their poses are not prescribed. Cyan gripper bars in the preview indicate those virtual forces and are not collision bodies.

**Connector retention is an explicit approximation.** The preceding fixed-receiver example had no locking catch. Its longer-cable trial pulled the plug out after release. This experiment therefore adds a pose-triggered, bounded spring capture to represent the concealed lock. It engages only when the dynamic plug reaches the seating tolerances; it does not teleport the plug or add a fixed joint. The spring's limits are assumptions, not measured hardware ratings. The hidden catch geometry, physical snap-through and electrical contacts are not reconstructed.

The real original clip geometry is used with its earlier collision decomposition and native spring revolute joint. The original spring reopens under the longer, end-constrained cables after the grips release; that result is preserved as `original_spring.mp4` and `original_spring_motion.npz`. The default variant tests a stronger preloaded closing spring. Its settings are hypothetical design parameters, not measured hardware properties. Its opening is driven by cable contact, not an animated lid. Connector geometry remains a visual approximation based on the reference footage.

## Parameters

- Six cables, length 457.2 mm each; radius 1 mm; 91 capsule segments per cable, each 5.02418 mm long.
- Bending rigidity EI = 0.0011459155902616466 N m²; rod bend stiffness = EI / segment length. Bend damping = 0.005729577951308233 × 0.005 / segment length. Stretch stiffness 1,000,000 N/m and damping 0.1. Shear and twist use the Newton rod constructor's inherited settings.
- Continuous-cylinder density target 1,000 kg/m³, using capsule density correction to avoid overlapping end-cap mass. Geometry-derived inertias replace Newton's absolute minimum-inertia correction.
- Rod rest bend and twist are explicitly zero, despite an initially curved pose.
- Physics 3,840 Hz, 64 substeps per rendered physics frame, 40 solver iterations. Recorded poses at 60 Hz, preview at 30 fps.
- Contact stiffness 1,000,000; damping 100; friction 0.5; gap 0.2 mm; compliant ALM, contact history, friction epsilon 0.0001.
- Original clip spring: stiffness 0.0114591559 N m/rad, damping 0.0002864789 N m s/rad, no preload.
- Default preloaded variant: stiffness 0.114591559 N m/rad, damping 0.002864789 N m s/rad, constant closing preload 0.03 N m. Newton clamps drive targets to joint limits, so preload is applied as a time-independent torque in addition to the native spring. Both variants use axis Y, limits −75 to 0 degrees, and scale 1.
- Clip translation: (−18, 215, 1) mm; clip centre plane Y = 200 mm. Far connector remains supported at (0, 402, 10) mm.
- Two grips are centred on segments 32 and 49 of each cable; each acts over seven adjacent segments (a 30.145 mm region) to control local direction as well as position. Target Y spacing 89.4 mm, about 29.7 mm beyond either edge of the 30 mm-wide clip. Lateral pull 37 mm. The grips support the bundle from time zero; during connector insertion they shift sideways by 37 mm and lower by 65 mm, then lower to the clip approach height during 5.5–7.5 seconds. At each of the seven samples: position stiffness 600 N/m, damping 0.12 N s/m, force capped at ±0.5 N per axis. Each grip therefore applies at most 3.5 N per axis per cable across its region. Grip targets finish at Z = 15 mm.
- Plug hand: position stiffness 4,000 N/m, damping 8 N s/m, ±20 N per axis; small-angle angular stiffness 2 N m/rad, damping 0.003 N m s/rad, ±0.15 N m per axis.
- Approximate retention: position stiffness 8,000 N/m, damping 6 N s/m, force magnitude capped at 5 N; small-angle angular stiffness 6 N m/rad, damping 0.008 N m s/rad, torque magnitude capped at 0.2 N m. Capture requires X/Y errors below 1 mm, Z below 0.4 mm and angular error below approximately 3 degrees. Capture disengages beyond 4 mm displacement or approximately 12 degrees, and does not reengage after breaking away.

Exact constants and paths are in `simulate.py` and `motion.json`.

## Reproduce and inspect

Use the repository's pinned Newton/Warp environment, then run:

```bash
python assets.py
python simulate.py
python validate.py
python render.py
python export_playback.py
```

Reproduce the original-spring comparison with `python simulate.py --original-clip-spring --output original_spring_motion`, then `python render.py --motion original_spring_motion --output original_spring`. Its validation is expected to report failure of the cable-retention criteria; the unsuccessful result is deliberately preserved.

Optional controls are `--no-retention`, `--no-clip-contact`, and `--no-receiver-contact`. The last also disables the retention approximation. Use `--output` to preserve the reference motion.

- `connector_and_clip.mp4`: combined sequence using the stronger preloaded spring.
- `original_spring.mp4`: same combined sequence with the original spring, which reopens after release.
- `original_spring_motion_validation.json`: original-spring comparison measurements; intentionally not a passing retention result.
- `assets/`: standalone approximate connector USD/STL assets; coordinates in metres.
- `clipTop.stl`, `clipBottom.stl`: original unscaled source files, in millimetres.
- `actual_clip_scene.usda`: earlier geometry source supplying the exact lid decomposition. Its old cable visuals are not used in the combined scene.
- `motion.npz`: recorded receiver, plug, six 91-body rods, then clip top. Rod capsules use local Z.
- `connector_playback.usdc`: Isaac-compatible visual playback, with overview, insertion and clip cameras. No active PhysX solver.
- `validation.json`: numerical checks of fixed receiver, seating, attachment continuity, clip opening/closure, cable retention, and sampled penetration.

Full lid closure (within 1 degree) is an explicit diagnostic, separate from the required retention checks; it fails in both variants. A passing retention result must not be read as proof of full lid closure.

The validation uses the earlier clip experiment’s 0.2 mm rod-joint continuity and mesh-penetration tolerances (10% of cable diameter), a 1 mm seating tolerance and a 2 degree final plug alignment tolerance. It requires the plug to be more than 10 degrees tilted at 2.5 seconds before rotation completes. The validation is sampled rather than a continuous collision proof. Passing does not establish real cable material calibration or actual connector holding force. Prior independent experiments remain unchanged.
