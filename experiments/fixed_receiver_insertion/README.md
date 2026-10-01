# Angled insertion into a fixed receiver

Corrected interpretation of the reference task: the black table connector is fixed. The white cable plug approaches at 25 degrees, engages its leading edge, then rotates down into the receiver. There is no hinge, pin, revolute joint, or detent on the black connector. The earlier `hinged_connector_insertion` experiment is preserved as a superseded interpretation.

The gray housing, blue cable support, contact details, wood opening, metal table, and mounting pedestal retain the approximate proportions of the previous assets. Dimensions are inferred, not measured customer CAD. The tilted approach clears the front retaining wall before rotating into the cavity. The rear wall is shifted back by 1.5 mm to clear the swept plug corners during rotation (29.5 mm clear cavity length). The six cables are 2 mm in diameter and 304.8 mm long.

## Physics and scope

Newton 1.6 SolverVBD/AVBD with compliant ALM solves the plug, cables and contacts at 3,840 Hz, with 40 iterations per step. The receiver is a fixed kinematic body with no motion target and no applied forces. A bounded force/torque hand controller acts on the dynamic white plug; its pose is never prescribed. The hand target turns about the plug's leading top edge during seating, but no joint connects that edge to the receiver. Rod rest bend and twist are explicitly zeroed after solver construction.

The hand fades out from 4.8 to 5.2 seconds. The remaining 1.8 seconds show passive settling. This reconstructs the visible insertion and seated pose. **The concealed locking catch has not been reconstructed; remaining seated under cable load and gravity is not a verified snap lock or pull-out retention test.** The reference does not reveal enough geometry to claim matching lock forces.

Cable stiffness and contact settings are retained from the preceding connector experiment. `simulate.py` and `motion.json` contain the exact settings. There is no receiver detent or closure controller.

## Files and reproduction

- `assets.py`: editable procedural meshes and collision primitives.
- `assets/cable_end_connector.usdc` and `.stl`: white cable-end connector.
- `assets/fixed_table_connector.usdc` and `.stl`: complete fixed receiver and mounting base; separate STL components are also provided.
- `simulate.py`: Newton simulation and hand trajectory.
- `motion.npz`: recorded body poses at 60 Hz; receiver is body 0, white plug body 1, six 61-segment cables follow.
- `validate.py` / `validation.json`: fixed-receiver, insertion-angle, seating, attachment and sampled penetration checks.
- `render.py` / `connector_insertion.mp4`: offscreen Newton OpenGL playback at 30 fps.
- `export_playback.py` / `connector_playback.usdc`: Isaac-compatible visual playback with cameras; no active PhysX.

Run `assets.py`, `simulate.py`, `validate.py`, `render.py`, and `export_playback.py` in that order using the repository's pinned environment. STL and USD coordinates are metres. Sampled validation is not continuous collision proof or real-world calibration.
