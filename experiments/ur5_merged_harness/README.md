# Connector insertion and three clip cable routing

The UR5 grasps the connectors by their existing side tabs, inserts both ends of a six-wire harness, then attempts to seat the wires in three original spring clips. The connectors have no added top handling rib. The Robotiq uses its plain pads, with no fingertip shelves or cable attachments.

The controller checks individual wires after releasing each grasp. Before a scheduled bulk insertion, four or more wires already inside trigger individual-wire recovery instead; a fully filled clip is observed without another grasp. If a wire is outside, it selects a protruding section on the opposite longitudinal side of the clip, attempts to grasp that section, and pulls toward the channel. It stops the pull after the selected wire passes nine consecutive geometric retention checks, then releases and checks again. Seating during a pull can be lost after release; neighboring wires and previously filled clips can also move.

## Recorded outcome

The 157.5 second recording includes both connector insertions and 8 individual-wire recovery attempts. Both connectors remain seated. The final two-second channel counts are **6/6, 5/6, and 5/6**. Strict physics validation **did not pass**.

| Clip | Inside channel and below moving lid | Fits closed lid envelope | Final lid angle |
|---|---|---|---|
| 1 | 6/6 | 4/6 | -71.16° |
| 2 | 5/6 | 5/6 | 0.00° |
| 3 | 5/6 | 5/6 | 0.00° |

The channel column requires every recorded frame in the final two seconds to pass. The closed-lid column samples the same wire positions against closed lid geometry; it does not simulate closing the lid. An open lid and a channel count of six are not proof of a secured bundle. Recovery selection uses the moving-lid test; it does not yet trigger a separate correction for the two high wires in the first clip.

The sampled robot-clearance and connector-contact checks pass. The maximum sampled cable-joint gap is 3.749 mm; the cable/table overlap is 1.080 mm. The tolerance for each is 0.2 mm. See the separate contact and robot-clearance reports for their measured overlaps. The recovery selection audit passes: selected wires, longitudinal sides, and retry limits are checked against recorded geometry. `recovery_outcomes.json` distinguishes temporary seating from the selected wire remaining inside after release.

[Video](ur5_cable_clip.mp4) · [Retention timeline](retention_timeline.png) · [Validation summary](validation_summary.json) · [Recorded USD playback](ur5_playback.usdc)

## Model

- Six cables, each with nominal length 762 mm and diameter 2 mm, with 152 rods per cable and zero rest curvature.
- Bending rigidity EI = 0.0011459155902616466 N m². Rod bend stiffness is EI divided by segment length; stretch stiffness is 1e6, stretch damping 0.1, and bend damping is 0.005729577951308233 × 0.005 divided by segment length.
- Original unscaled clipTop and clipBottom meshes. Revolute spring stiffness 0.114591559 N m/rad, damping 0.002864789 N m s/rad, closing preload 0.03 N m, and angular limits −75 to 0 degrees.
- Newton SolverVBD with compliant ALM, 1,920 substeps per second. The recording begins at 60 iterations per substep and uses 80 for individual-wire recoveries and subsequent clips. `motion.json` records the transitions.
- Kinematic UR5 inverse kinematics and Robotiq linkage kinematics. Cable motion comes from gravity, elastic rods, and Newton contacts. These are scripted grasps, without force sensing or a learned policy.
- Both connectors start dynamic. Idealized fixed socket locks activate only after alignment is within 1.5 mm and 6 degrees. The hidden physical catch is not reconstructed.

## Recovery trials

The first bundle attempt used a 1.8 mm commanded pad gap to collect the wires vertically between the pads. It retained five of six wires after withdrawal. Wider gaps of approximately 15.35 mm and 11.8 mm failed to lift the bundle and are preserved as diagnostic branches.

The first individual recovery approached from the opposite side and pulled at a low height; it failed and disturbed a neighbor. The revised controller lifts the selected wire, approaches outside the mouth at a nominal wire height of 22 mm, and pulls inward at 20 mm. It records both transient seating and the result after release.

The final far-clip retry at 143.5 seconds lowers the target wire axis to 14 mm and requires a fit beneath the closed-lid envelope before stopping the pull; previous recovery variants are preserved as controller snapshots. The selected retry opens the jaws only to 2.8 mm, lifts 80 mm, then opens fully. In the 60 mm release branch, four neighboring wires ended outside; that [failed release](diagnostics/failed_wide_release.mp4) is preserved as a separate diagnostic branch from the same 143.5 second state. The reruns diverged numerically before release, so the comparison does not isolate jaw width as the sole cause; `release_comparison.json` reports the difference.

The selected run permits three recovery attempts per clip and two consecutive failures per wire; exhausted limits are explicit events.

## Evidence and limits

`motion.json` contains latch events, actual wire selections, post-release retention, retry limits, and final counts. `validation_summary.json` and the individual audit reports distinguish task results from contact and numerical tolerances. The recording is experimental: contact overlap and rod-joint errors exceed the strict 0.2 mm tolerances in parts of the sequence. Counts describe wire intersections inside the channel and below the moving lid; they do not require the lid to close. Final lid angles and the number of wires fitting a fully closed lid envelope are reported separately. Do not interpret a visually seated wire as validated real-world retention.

The video replays the recorded Newton poses through Newton OpenGL. The USD is visual playback for Isaac Sim at 10 pose samples per second with interpolation, with no active PhysX simulation. The lossless recording retains all 60 samples per second. `retention_timeline.png` plots measured geometric retention rather than commanded target positions.

Run scripts in a copy of this experiment. `simulate.py --resume motion_boundary.npz` resumes only a completed phase with actual body poses and velocities; the collision warm start resets. `--stop-after` counts completed phases across restarts. Use `--iterations 80 --max-repairs-per-clip 3` for the later stages. `render.py --stride 4` renders 15 fps. `validate_all.py` runs every audit and returns a failure code if any strict check fails.

`reproduce_recording.py` repeats the selected phase boundaries, the low first recovery, and the revised later path. Diagnostic branches preserve their shared prefixes and actual solved tails. The selected sequence uses the narrow initial bundle grasp and the narrow final release. GPU numerical differences can change subsequent conditional selections.

To iterate directly on the final wire recovery without repeating the earlier simulation, restore its exact branch checkpoint:

```bash
python make_checkpoint.py --output checkpoint_143_5.npz
python simulate.py --resume checkpoint_143_5.npz --iterations 80 --max-repairs-per-clip 3
```

The helper reconstructs the recorded prefix and restores the saved body velocities and controller state. The collision warm start resets on resume; numerical reruns can diverge. Use `controller_wide_release.py` with `--stop-after 13` to repeat the wide-opening diagnostic from that checkpoint.
