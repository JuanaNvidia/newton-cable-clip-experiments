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
