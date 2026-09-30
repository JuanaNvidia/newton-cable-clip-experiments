# Six-cable clip insertion — Newton VBD

This is a separate Newton version of the saved higher-entry, two-hand pull. It uses six 2 mm cables, 2.5 mm connector attachment pitch, two shared connectors, the original unscaled `clipTop.stl` and `clipBottom.stl`, and the measured hinge bore at x ≈ 3 mm, z ≈ 22 mm.

**Newton 1.6.0 SolverVBD solves the entire assembly:** native rod constraints for the cables, rigid-body contacts, fixed connector attachments, and a native spring-driven revolute joint for the clip. PhysX does not participate. The video is a replay of solved Newton poses in Newton's OpenGL renderer. An Isaac-compatible USD playback and an optional Isaac RTX rendering script are also supplied.

## Motion and model

The two virtual grips remain approximately 1.08 inches beyond either side of the clip base. They act on segments 15 and 32 of each 48-segment cable. The intervening span bends freely. Cyan bars are visual grasp markers, without physical collisions.

The higher-entry trajectory is preserved: bundle-center x moves from 55 to 18 mm between 0.5 and 2.5 seconds; the grip height rises from 14 to 20 mm, then returns to 14 mm by 3 seconds. A 4 mm outward spring-target offset tensions the span. Grasp forces fade between 3.0 and 3.2 seconds, followed by passive settling through 5 seconds. Grip stiffness, damping, and force caps match the previous higher-entry run.

The clip's hinge target stays at zero, with limits of -75 to 0 degrees. Contacts alone open it. The top uses the existing 68 convex prisms, preserving the hole and entry shape. The base uses its original mesh plus the existing contained entry support.

Newton runs on the GPU through Warp 1.17.0 at **3,840 Hz**, using **40 VBD iterations per step** and compliant augmented-Lagrangian rigid constraints. Motion is recorded at 60 Hz. The cables use Newton ROD constraints with stretch, shear, bend, and twist; they are not PhysX capsule joints or a separate FEM mesh.

Angular spring values are converted from the original USD per-degree convention to Newton's per-radian convention: hinge stiffness 0.011459 N·m/rad and damping 0.00028648 N·m·s/rad; cable bend/twist stiffness 0.114592 N·m/rad and damping 0.00572958 N·m·s/rad. Rod stretch/shear stiffness is 1e6 N/m, with 0.1 N·s/m damping. Newton and PhysX have different constraint formulations, so matching these nominal parameters does not establish material equivalence.

The builder's absolute inertia floor would inflate these millimetre-scale capsule inertias. The simulation independently verifies positive eigenvalues and the inertia triangle inequalities, then restores geometry-derived inertia tensors and their inverses before creating the solver. The installed Isaac/Newton environment was not modified; a separate pinned Newton runtime was used.

## Files

- `six_cables_newton.mp4`: assembly views and half-speed Newton insertion preview.
- `actual_assets_newton.mp4`: normal-speed close-up.
- `actual_clip_playback.usdc`: Isaac-compatible recorded animation, including grip markers.
- `actual_clip_scene.usda` / `.usdc`: visual geometry and cameras, with all PhysX physics removed. Run `simulate_newton.py` for dynamics.
- `actual_motion.npz` / `.json`: recorded poses and solver settings.
- `validation.json`: geometry, retention, ordering, attachment, and contact-control checks.
- `no_gate_contact.npz` / `.json`: counterfactual run with only cable-to-top contact disabled.
- Source meshes, pinned requirements, simulation, validation, and rendering scripts.

## Reproduce

Use a separate Python environment with the packages in `requirements-newton.txt` and an available CUDA GPU. FFmpeg is required for video encoding. Do not install the newer Warp into an existing Isaac environment; keep the Newton runtime separate.

```bash
python simulate_newton.py
python simulate_newton.py --seconds 3 --no-gate-contact --output no_gate_contact
python validate_actual.py --control
python export_playback.py
python render_newton.py
python make_video.py
```

For an additional Isaac RTX render, run `render_isaac.py` using the Isaac Sim Python environment after simulation finishes. It replays the recorded Newton poses and writes `actual_assets_isaac.mp4`. Only one Isaac/Kit process should run at a time.

This remains an uncalibrated material model. Validation samples recorded frames and capsule axes; it is not a continuous collision proof or a prediction of real insertion forces.

References: [Newton VBD solver](https://newton-physics.github.io/newton/latest/api/_generated/newton.solvers.SolverVBD.html), [USD angular-drive units](https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html).

## Verified outcome

All six cables remained inside for the full 1.78 seconds observed after release. Their left-to-right order was preserved, with only 0.044 mm final center-height spread. The clip opened 25.32 degrees and returned to approximately zero degrees.

Maximum sampled overlap was 0.083 mm against the top, 0.000 mm against the base, and 0.0013 mm between cables. All were below the unchanged 0.2 mm acceptance limit. Maximum cable-joint gap was 0.0145 mm, connector attachment error 0.0010 mm, and hinge-anchor error 0.0011 mm.

With cable-to-top contact disabled, maximum hinge opening was 0.000036 degrees. This control supports contact-driven opening rather than prescribed hinge animation. All listed validation checks pass.
