# Twelve-inch Newton cables

Separate variant of `six_cable_newton`; the original is preserved. The target is a qualitative resemblance to IMG_1595.MOV: broad curves that resist gravity but can bend under a hand. The video is not a measured material calibration.

## Material and discretization

- Six cables, 2 mm diameter, each 304.8 mm (12 inches) long.
- 61 native Newton rod segments per cable; segment length 4.9967213 mm.
- Straight, untwisted rest state. No prescribed curved rest shape or animated middle segments.
- Bending rigidity EI = 0.00114591559 N m², twice the preceding material setting.
- Per-joint bending stiffness = EI / segment length = approximately 0.229334 N m/rad; twist uses the same value.
- Per-joint bend/twist damping = approximately 0.00573334 N m s/rad. The distributed damping is unchanged from the preceding material.
- Stretch/shear stiffness 1,000,000 N/m; stretch/shear damping 0.1 N s/m.
- Physical line density corresponds to a solid 2 mm cylinder at 1000 kg/m³: 3.14159 g/m, approximately 0.957557 g per cable. Capsule density is corrected for overlapping end caps rather than counting overlapping mass twice.
- Analytic capsule inertias are restored after Newton's absolute inertia floor, as in the preceding implementation.
- Contact stiffness 1e6 N/m, damping 100 N s/m, friction coefficient 0.5, contact generation gap 0.2 mm per shape. Gravity 9.81 m/s² downward.
- Newton 1.6 / Warp 1.17, SolverVBD with compliant ALM, 40 iterations, 3840 physics steps/s. All cable and clip physics use Newton.

## Arch test

`arch_test.py` compares 0.1x, 1x, and 2x the previous bending rigidity. Each cable is 304.8 mm long, its endpoints are clamped 240 mm apart and 15 mm above the board, and its middle is unconstrained. Endpoint positions and initial tangent orientations are held, representing secured connectors. Comparison cables cannot collide with one another. The same upward circular arc is used only as an initially deformed state; the constitutive rest state is straight. The short endpoint spacing is what permits an arch. A free cable with both ends unsupported is not expected to float in an arch.

Gravity acts throughout. A downward half-sine force, peaking at 0.02 N, acts at the midpoint between 3 and 4 seconds, then is removed. This load is an illustrative disturbance, not a force measured from the reference footage. `arch_results.json` reports geometry, recovery, anchor error, and joint error. `arch_comparison.mp4` is actual Newton pose playback, with colors identifying each stiffness. This is a finite-duration demonstration of this support arrangement, not proof of stability for arbitrary endpoint positions or disturbances.

## Longer cable insertion

`simulate_newton.py` uses the actual clipTop/clipBottom geometry and spring revolute hinge, the higher-entry hand trajectory, and the new cable material. Both connectors remain dynamic and attached to all six cables. Grips are approximately 25 mm outside the sides of the clip. The clip remains at its original scale. Grip forces release at 3.2 seconds. `validation.json` records the outcome instead of inferring success from appearance.

`actual_assets_newton.mp4` is an OpenGL rendering of solved Newton poses. `actual_clip_playback.usdc` is recorded visual playback that can be opened in Isaac Sim; it does not run a second physics solver. `actual_motion.npz` contains the recorded poses. These previews were not rendered with Isaac RTX.

## Reproduction

Use Python with `requirements-newton.txt`, then run in this directory:

```sh
python arch_test.py
python validate_arch.py
python render_arch.py
python simulate_newton.py
python validate_actual.py
python export_playback.py
python render_newton.py
```

The source contains the exact parameters and boundary conditions. Matching real wire more closely would require measured free-span length, endpoint positions/orientations, sag, and a known-load deflection or release recording. Conductor, insulation, residual curvature, plasticity, and hysteresis are not separately modeled.

## Verified results

The selected material retains an arch 78.38 mm above the endpoint elevation in the specified fixture after eight seconds. Its apex deflects 0.217 mm during the 0.02 N test pulse and returns within 0.001 mm of the pre-load height. The original material also retains an arch with these clamped ends, but deflects 0.733 mm. A 0.1x-stiffness control collapses onto the board. These heights depend on the chosen span and endpoint tangents; they are not measurements from the footage.

All arch checks in `arch_validation.json` and all insertion checks in `validation.json` pass. All six cables remain seated for the 1.78-second observation after hand release. Maximum sampled cable/clip penetration is 0.00146 mm and cable/cable overlap is 0.00113 mm. Maximum cable-joint separation is 0.00836 mm. This variant did not repeat the predecessor's gate-contact-disabled control run.
