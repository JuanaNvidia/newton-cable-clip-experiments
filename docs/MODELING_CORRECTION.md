# Correction: rest curvature in the historical arch experiments

During development of the hinged connector experiment, inspection of the installed Newton 1.6 implementation revealed that SolverVBD initializes native rod rest bend/twist invariants from the initial transforms in `model.body_q`. Identity local joint-anchor rotations alone do **not** establish zero rod rest curvature when the initial bodies are arranged along a curve.

The relevant implementation is `SolverVBD._refresh_rod_rest_bend_twist_cache`, which calls `init_rod_rest_bend_twist`. It measures the initial relative material frames and stores `joint_rod_rest_kb_local` and `joint_rod_rest_twist`.

## Which results are affected?

- The fixed-span arch comparison in `experiments/long_cable_newton/arch_test.py` and the moving-endpoint comparison in `experiments/three_inch_arch/arch_test.py` start with curved body transforms. They therefore used an initially curved material rest shape. Earlier documentation and on-screen labels saying their rest shape was straight were incorrect.
- The freely rotating endpoint diagnostic has the same rest-state issue, as well as its documented boundary/contact limitations.
- The original insertion scripts initialize their cables straight, so this specific rest-curvature issue does not change those insertion results.

The historical recordings, reported poses, and geometric validation remain available without changing their recorded data. However, those arch experiments **do not demonstrate that a straight-rest cable with the reported stiffness has the shown arch behavior**. Endpoint geometry, curved rest state, gravity, and stiffness jointly determine those results. Old video labels must be read with this correction; the render scripts now label the historical rest state correctly if used again.

## New hinged connector experiment

`experiments/hinged_connector_insertion/simulate.py` explicitly zeroes `solver.joint_rod_rest_kb_local` and `solver.joint_rod_rest_twist` after solver construction, before any steps. That experiment uses an initially bent state with zero material rest curvature/twist. This is a version-specific implementation for the pinned Newton 1.6 runtime. If code later changes the model and calls a rest-cache refresh, zeroing must be reapplied or replaced by an explicit supported material-rest setup.

It also initializes the revolute joint's generalized coordinate to the initial 25-degree body angle. VBD combines a rest-relative angle with the stored generalized rest coordinate when applying limits. Leaving that coordinate at zero would incorrectly place the initial tilted pose at the lower stop.

This correction is a reason to preserve the experiment history and distinguish visual agreement from measured material calibration. No corrected rerun of the historical arch comparisons is claimed here.
