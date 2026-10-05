# Experiment history

## Completed reference experiments

1. **240 mm six-cable insertion:** native Newton rods, native spring revolute clip hinge, fixed connector attachments, force-driven hand grips. 3840 Hz resolves the original coarse-step grip instability. All six cables retained after release. A cable-to-top-contact-disabled control remains closed, supporting contact-driven lid opening.
2. **304.8 mm six-cable insertion:** 61 segments, 2 mm diameter, double original bending rigidity, continuous-cylinder mass correction. All six retained and ordered; maximum sampled clip penetration about 0.00146 mm.
3. **304.8 mm arch comparison:** ends clamped 240 mm apart, starting in an upward arc that Newton also used as its curved rest state. Compare 0.1×, 1×, 2× stiffness under gravity and a 0.02 N downward half-sine midpoint force from 3–4 s. Selected material's apex settles about 78.38 mm above its endpoints, with approximately 0.217 mm force-pulse deflection. Original material also arches with these clamped ends.
4. **Bring ends to 76.2 mm apart:** same material comparison. Clamp positions move inward during 1–5 s, orientations stay fixed. No midpoint force. Hold until 8 s. Selected material's apex reaches 130.14 mm above the endpoints; actual final endpoint spacing is 76.209 mm.

## Diagnostic archive

- `archive/coarse_960hz/`: recorded earlier coarse-step insertion and control, plus the failed validation report. Grip oscillation, retention/order failure, and incomplete closure make this unsuitable as the reference. The exact historical source snapshot was not saved, so this is recorded evidence, not a claim of byte-for-byte rerun reproducibility.
- `archive/inflated_inertia/`: early warmup recording and metadata before the geometry-derived inertia restoration. Exact historical source snapshot was not saved. Newton's absolute inertia floor can dominate millimetre-scale rods.
- `archive/freely_rotating_endpoints/`: first arch trial with ball-joint endpoints. Endpoint rotation permits sideways tipping, which confounds a stiffness-only comparison. The cases were close enough to contact each other. The later reference uses clamped endpoint orientations and filters inter-case contact. This archived script and recording are preserved for inspection, not presented as calibrated evidence.

Cached compiled kernels, transient checkpoints, repeated identical download folders, and rendered frame sequences are omitted. The original source/output folders outside this repository remain untouched.

## Packaging changes

Physics coefficients and reference recordings were preserved. Repository preparation made render scratch directories portable and removed a workstation-specific Isaac app path. Added a runner, documentation, integrity checks, and a pose-based validator for the 3-inch experiment. These packaging changes do not retroactively change the saved simulation results.

## Superseded hinged connector interpretation

A fifth experiment incorrectly assumes a hinged black receiver. It reconstructs the visible gray plug, blue wire support, and black hinged table fixture, then solves insertion and rocking closure through Newton contact. It explicitly zeroes rod rest bend/twist. See [the modeling correction](MODELING_CORRECTION.md), [new experiment](../experiments/hinged_connector_insertion/README.md), and its validation/control results. Historical recordings remain unchanged.

## Corrected fixed receiver

The user confirmed that the black table connector has no hinge. `fixed_receiver_insertion` removes the hinge geometry, revolute joint and detent; the dynamic white plug enters at 25 degrees and rotates down about its leading edge under a force-driven hand. The receiver stays fixed throughout. Six 2 mm, 304.8 mm cables use explicit zero rod rest curvature. The concealed catch is not reconstructed: seating is validated, positive locking/pull-out retention is not. The earlier experiment remains available as a superseded model.

## Combined 18-inch connector and spring-clip sequence

`connector_and_clip` extends the cables to 457.2 mm (91 segments) and combines angled insertion into the fixed receiver with routing all six cables into the original unscaled clipTop/clipBottom. The supporting plug hand releases after the two cable grips. A bounded, pose-triggered retention spring approximates the concealed connector catch; its parameters are assumptions, not measured lock forces. Original experiments remain unchanged.

The combined example includes an original-clip-spring comparison that reopens under the longer cable load. Its default variant increases spring stiffness and damping by 10× and adds 0.03 N m constant closing preload. Both use the same unscaled clip meshes. These are design assumptions, not measured spring ratings.

## UR5 pickup from free hanging tails

The new [UR5 / Robotiq experiment](../experiments/ur5_cable_clip/README.md) starts with the connector fixed in its seated pose. Six free 18-inch cable tails hang outside the original clip. A nominal UR5 CB3 arm and Robotiq 2F-85 use scripted Cartesian motion and numerical IK. Newton contacts and friction provide the grasp; cable bodies are not attached to fingers. The controller checks entry relative to the moving lid, then releases, lowers the open jaws to clear hanging tails, withdraws sideways and returns home. Retention is measured after withdrawal. See the experiment reports for observed results and numerical limitations. This is a qualitative simulation, not a hardware-ready robot program.

## UR5 insertion on an interior tabletop, two connectors

The [interior-table robot experiment](../experiments/ur5_interior_clip/README.md) extends the tabletop beneath and beyond the clip, with 242 mm clearance from the nearest edge. One connector remains seated, and a second 15 g dynamic connector holds the six distal cable ends together. A top-down grasp closes sideways across the bundle, then lifts and inserts it using Newton contacts. The open gripper retreats vertically above the table. The prior edge-mounted experiment remains available separately. See the recorded reports for the actual result and numerical limitations.

## Conditional pull from the other side of the clip

The [opposite-side recovery experiment](../experiments/ur5_opposite_pull/README.md) repeats the interior-table insertion. At 18 seconds, it checks retention and, if necessary, regrips the bundle on the seated-connector side, approximately 65 mm before the clip. It pulls sideways at up to 4 mm/s over a bounded 24 mm range, then releases and observes. The controller shares its retention geometry with validation; an entry event is distinguished from retention after the final withdrawal. The same physical fingertip supports and dynamic second connector are retained.

## Thirty-inch harness: two installed connectors and three clips

The robot starts with the complete six-cable harness on the table, installs both end connectors, then attempts all three clips in a shallow arc. Each cable is 762 mm long and 2 mm in diameter. The connectors have added handling ribs; the gripper has physical fingertip support lips. The original clip assets are unscaled. Socket locks are idealized constraints gated by the actual seated pose.

The recorded result retains **5/6, 6/6, 6/6** after withdrawal. Both socket locks remain engaged. Initial feeds were followed by upstream recovery. Additional low-entry and tilted-grasp branches are stored separately. The selected branch is `assembly_recovery`. The recording contains development restarts with preserved poses and velocities; its per-segment solver iteration counts are 80 → 80 → 80 → 80 → 40. The intermediate 40-iteration attempt exceeded the cable-joint gap criterion.

Whole-record strict contact checks fail: maximum cable/clip overlap 0.794 mm, cable/gripper overlap 0.787 mm and connector/socket-or-pad overlap 1.594 mm, against a 0.2 mm limit. This is an experimental manipulation record, not validated real-world execution. See the [case documentation](../experiments/ur5_three_clips/README.md), [retention timeline](../experiments/ur5_three_clips/retention_timeline.png) and per-audit JSON reports.
