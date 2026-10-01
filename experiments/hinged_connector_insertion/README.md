# Video-inspired insertion into a hinged table connector

This experiment reconstructs the visible connector mating sequence in the user-provided reference video, particularly approximately 20–24 seconds (plug/fixture details) and 33–38 seconds (alignment, lowering, and rocking into place). The geometry is a parametric surrogate, not customer CAD or measured reverse engineering.

## Assets

- `assets/cable_end_connector.usda` / `.usdc`: light-gray multiway plug, raised guides, keyed ends, recessed-contact appearance, and blue wire support. Main housing is 42 × 27 × 8 mm. Twenty contact details are visual approximations; electrical contact is not simulated.
- `assets/hinged_table_connector.usda` / `.usdc`: black pedestal and open receiver cavity, side guides, end stops, pivot hardware, and mounting hardware. Base and receiver remain separate transform groups. Hinge metadata is provided; Newton articulation and detent physics are built by `simulate.py`.
- `assets/cable_end_connector.stl`, `table_connector_base.stl`, `table_connector_receiver.stl`: mesh exports, coordinates in **metres**. STL does not preserve materials or hinge articulation.
- `assets/geometry.json` and `assets.py`: editable geometry, colors, collision assignments, and hinge/seat coordinates.

The receiver's pivot, clearances, mass, and detent behavior are assumptions because the hand obscures the real mechanism. The receiver has a genuine open cavity represented by separate collision boxes; it is not one solid convex hull. Contact recesses, blue support, and decorative hardware are visual details rather than individually simulated electrical components.

## Simulation

Six 2 mm cables, each 304.8 mm long, connect the dynamic insertion plug to a stationary far connector. The far endpoint is anchored as a task boundary condition. Each cable uses 61 native Newton rod segments, EI = 0.00114591559 N m², stretch/shear stiffness 1e6 N/m, stretch/shear damping 0.1 N s/m, bend/twist stiffness EI / segment length, and bend/twist damping approximately 0.00573334 N m s/rad. Capsule masses are corrected to a solid-cylinder line density of 3.14159 g/m. Analytic inertia tensors are restored after the builder's absolute inertia floor.

**Rest curvature and twist are explicitly zeroed in SolverVBD's rod-rest caches.** The initially arched cable shape is therefore an initial deformation, not its material rest shape. These version-specific cache assignments are pinned to Newton 1.6. This corrects the implicit curved-rest behavior found in the earlier arch experiments; those historical videos should not be interpreted as straight-rest material calibration.

The 15 g plug is dynamic. A finite force/torque controller represents a hand gripping it: position stiffness 4000 N/m, damping 8 N s/m, force limit 20 N per axis; small-angle rotational stiffness 0.5 N m/rad, damping 0.003 N m s/rad, torque limit 0.15 N m per axis. These limits are numerical task choices, not measured hand forces. The controller lowers the plug, seats it, then rocks it toward horizontal. It fades out from 4.8–5.2 seconds; simulation continues through 7 seconds with no hand force.

The 25 g receiver has a native revolute joint around X, at (0, -18, 24) mm, with 0–45 degree limits. Its initial generalized joint coordinate is explicitly set to 25 degrees to match the initial body pose. A time-independent bistable detent torque approximates a fixture that can remain open or closed:

```
torque = -0.012 * sin(2*pi*angle / radians(25)) - 0.001 * angular_velocity
```

No receiver angle trajectory is prescribed. Contact with the plug/cables drives its motion. There is no seating teleport or newly added fixed joint at insertion. The source uses a 0.3 mm hand-target overtravel to establish contact with the cavity floor. The ideal plug center in the receiver frame is (0, 18, 6.5) mm.

Newton 1.6 / Warp 1.17, SolverVBD with compliant ALM, 40 iterations, 3840 Hz physics, 60 Hz recorded poses, CUDA. Contact ke=1e6 N/m, kd=100 N s/m, friction=0.5, gap=0.2 mm, margin=0. Friction smoothing is 1e-4 m/s. Nonadjacent rod self-contact and inter-cable contact are active. Adjacent rods and directly attached cable/plug pairs are filtered. The board has a rectangular opening and a wood-colored layer beneath it.

## Results and checks

`connector_insertion.mp4` is a Newton OpenGL replay of solved motion with an initial overview and a closer insertion view. It is not an Isaac RTX render. `connector_playback.usdc` contains the same recorded motion for viewing in Isaac Sim, with no active PhysX simulation. `connector_scene.usda` / `.usdc` provide the visual scene and cameras.

`validate.py` checks finite poses, initial opening and closure, seat/alignment after release, hinge drift, cable-joint and endpoint errors, and sampled plug-to-receiver penetration. The comparison run disables **all plug and cable contacts against the receiver**, leaving the detent, gravity, and hand program unchanged. An initial plug-only contact filter was insufficient because the cables could still move the receiver; that diagnostic is retained outside the reference outputs.

The normal run retains the plug for the 1.8-second release observation, closes the receiver to approximately zero degrees, and has maximum sampled plug/receiver penetration of about 0.0303 mm. Maximum post-release seat-center error is about 0.504 mm and relative orientation error about 1.374 degrees. See `validation.json` for the complete measured results and contact-disabled control outcome.

These checks are finite-duration, sampled-pose checks, not proof of exact CAD fit, continuous collision-free motion, accurate electrical connector forces, or arbitrary-load retention.

## Reproduce

Use a separate environment with `requirements-newton.txt` and a CUDA GPU. Rendering needs OpenGL, FFmpeg, and DejaVu Sans fonts. Do not replace the Warp version in an existing Isaac environment.

```bash
python assets.py
python simulate.py
python simulate.py --no-receiver-contact --output no_receiver_contact
python validate.py --control
python export_playback.py
python render.py
```

Asset creation, validation, and USD export can use ordinary Python with the relevant dependencies. Only simulation requires CUDA; replay uses Newton's CPU/OpenGL viewer. The video and footage analysis are qualitative visual matching, not exact identification of the hidden mechanism.
