# Newton cable parameters

**Correction:** historical arch rest curvature was inferred from the initial curve, not zero. The new hinged-connector experiment explicitly zeroes rod rest bend/twist. See [details](MODELING_CORRECTION.md) and its [parameters and assets](../experiments/hinged_connector_insertion/README.md).

SI units are used in code. Angular rod coefficients are per radian. Per-joint stiffness is not whole-cable stiffness. The selected 12-inch material is the red 2× case in the arch comparisons.

## Cable geometry and mass

| Property | 240 mm insertion | 12-inch insertion and arch tests |
|---|---:|---:|
| Nominal centerline length | 0.240 m | 0.3048 m |
| Radius / diameter | 0.001 / 0.002 m | same |
| Capsule segments | 48 | 61 |
| Rod joints | 47 | 60 |
| Segment cylinder length | 0.005 m | 0.004996721311 m |
| Capsule total length including caps | 0.007 m | 0.006996721311 m |
| Shape density | 1000 kg/m³ | 789.364640884 kg/m³ |
| Mass per cable | 0.95504425 g | 0.95755744 g |
| Rest shape | straight, untwisted | Insertion: straight; historical arches: initially curved |

The baseline sums full capsule masses, including overlap. The longer variants use density `1000 * ds / (ds + 4*r/3)` so total mass equals a continuous cylinder at 1000 kg/m³. Corresponding line density is 3.14159265 g/m.

For each 12-inch segment, mass is 1.569766425e-5 kg. Transverse principal inertia is 5.703552214e-11 kg m²; longitudinal inertia is 7.518184859e-12 kg m². Before solver construction, analytic tensors and inverses replace Newton's absolute-floor-corrected tensors. Insertion scripts also check the inertia triangle inequalities; arch scripts check positive eigenvalues.

## Rod material

| Property | 240 mm baseline | 12-inch selected material |
|---|---:|---:|
| Bend rigidity EI | 0.000572957795 N m² | 0.001145915590 N m² |
| Bend stiffness per joint | 0.114591559 N m/rad | 0.229333501 N m/rad |
| Bend damping per joint | 0.005729578 N m s/rad | 0.005733338 N m s/rad |
| Twist stiffness/damping | inherit bend | inherit bend |
| Stretch stiffness | 1e6 N/m | same |
| Stretch damping | 0.1 N s/m | same |
| Shear stiffness/damping | inherit stretch | inherit stretch |

The 12-inch comparison factors relative to original EI are 0.1, 1, 2. Their bend/twist joint stiffness values are 0.011466675, 0.114666750, 0.229333501 N m/rad. All three use the same damping and mass. Joint frames use local +Z as the native rod tangent; saved insertion visual poses are converted to the source USD capsules' local Y convention.

No separate Young modulus, Poisson ratio, conductor/insulation layers, yield model, plasticity, hysteresis, breakage, or measured residual curvature is specified.

## Contact

| Setting | Insertion cable / clip / board | Arch cable | Arch board |
|---|---:|---:|---:|
| Normal stiffness ke (N/m) | 1e6 | 1e6 | 2500 |
| Normal damping kd (N s/m) | 100 | 100 | 100 |
| Friction mu | 0.5 | 0.5 | 1.0 |
| Detection gap (m) | 0.0002 | 0.0002 | 0.1 (builder default) |
| Margin (m) | 0 | 0 | 0 |

The arch board uses defaults because only its density is overridden. The gap expands contact search; it is not physical cable clearance. Contact material mixing uses arithmetic means of ke/kd and the geometric mean of mu, giving arch cable-board values 501250 N/m, 100 N s/m, and 0.70710678.

Shape defaults also store kf=1000, adhesion distance=0, restitution=0, torsional friction=0.005 m, and rolling friction=0.0001 m. These are not additional calibrated active effects in this VBD formulation. Adjacent rod segments are collision-filtered; nonadjacent self-contact remains active. Insertion includes inter-cable collisions; final arch comparisons filter inter-case collisions. No particles are present.

## Solver

Newton 1.6.0, Warp 1.17.0, CUDA, SolverVBD with rigid AVBD backend and compliant ALM. No separately instantiated PhysX, XPBD, MuJoCo, or FEM solver. Geometry-derived rigid inertia is restored before solver construction.

| Setting | Insertion | Arch tests |
|---|---:|---:|
| Iterations | 40 | 40 |
| Physics Hz / dt | 3840 / 1/3840 s | same |
| Substeps / recorded FPS | 64 / 60 | same |
| Gravity | (0,0,-9.81) m/s² | same |
| Contact history | enabled | enabled |
| Broad phase / matching | sap / latest | same |
| Contact maximum | 100000 | 10000 |
| Per-body contact buffer | 512 | 128 |
| Friction velocity smoothing | 1e-4 m/s | 1e-2 m/s |
| Non-rod structural linear stiffness | 1e7 N/m | 1e5 N/m |
| Non-rod structural angular stiffness | 1e5 N m/rad | 1e5 N m/rad |
| Non-rod structural linear/angular damping | 0 / 0 | 0 / 0 |
| ALM joint/contact alpha | 0 / 0 | 0 / 0 |
| Persisted multiplier gamma | 0.999 | 0.999 |
| Legacy penalty ramp beta | 0 | 0 |

Insertion passes legacy penalty seeds of 1e5 (linear joint) and 1e4 (contact); arch uses defaults. With beta=0 and compliant ALM, these do not constitute extra material stiffness. Likewise the legacy `rigid_contact_hard=True` default does not turn the compliant ALM material into an infinite-stiffness contact.

## Insertion geometry, connectors, hinge, and hands

Six cables, 2.5 mm connector pitch; total bundle width 14.5 mm. Two dynamic connectors, 15 g each, with 12 fixed cable attachments. Source clip scale is 1.0. Top collision geometry uses 68 convex pieces; the base has a contained support prism. The hinge axis is Y through the fitted bore near x=3 mm, z=22 mm.

Hinge zero-angle spring stiffness is 0.0114591559 N m/rad, damping 0.0002864789 N m s/rad, with limits -75 to 0 degrees. Opening is contact-driven.

Only the 12 grip segment centers receive hand-controller forces. The controller uses 120 N/m position gain, 0.0266666667 N s/m velocity damping, force caps ±0.5 N in X/Z and ±2 N in Y. Grip segments are [15,32] for the 48-segment case and [22,38] for the 61-segment case. The middle is unconstrained. Bundle X moves from 55 to 18 mm during 0.5–2.5 s. A 6 mm rise is applied during 0.5–1.1 s and removed during 2.5–3.0 s. A ±4 mm Y offset tensions the span. Forces fade during 3.0–3.2 s. Simulation ends at 5 s. Colored grip bars are visual markers, not colliding hand geometry.

## Arch boundaries

Three independent single-cable cases, initially offset in X by -45, 0, +45 mm. Endpoints start 240 mm apart and 15 mm above the board. Initial centerline is a circular arch with radius 130.391615 mm. End-segment tangents point approximately 65.87277 degrees upward into the arch. Historical arch rest curvature is inferred from the initial pose. Endpoint fixed joints retain those tangent orientations; the middle has no imposed shape.

- Fixed-span test: hold endpoints; apply a downward 0.02 N half-sine midpoint load from 3–4 s; simulate 8 s.
- Three-inch test: no midpoint load; hold for 1 s, move symmetrically to 76.2 mm separation from 1–5 s using cubic smoothstep, then hold through 8 s. Endpoint height and orientations remain fixed.

The reference video was used as a qualitative target, not a measured force/deflection calibration. Endpoint orientations are crucial to interpreting arch behavior.
