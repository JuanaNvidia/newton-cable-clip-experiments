# Connector insertion without the top rib

The UR5 grasps the connector's existing side tabs, lowers it at 25 degrees, rotates it into the fixed black receiver, then releases and withdraws. The added T-shaped top handling rib is removed from both the physics and exported STL/USD geometry. This isolated experiment contains no cables or clips.

[Watch the selected trial](pad_direct.mp4). [Inspect all trial results](trial_summary.json).

The selected recording remains seated without a latch or grasp attachment. Over the final two seconds, its maximum seat-position error is 0.023 mm and orientation error is 0.002 degrees. Recorded validation status: **PASS**.

## Methods tried

| Method | Result |
|---|---|
| Body pinch, level or angled release, then press | Early simplified geometry seated, but omitted collisions on visible tabs and ridges. The first grasps also exceeded the pad-overlap tolerance. Preserved under `initial_surrogate_trials`, not selected. |
| Low side-tab grasp, angled release, then press | Caught a guide; the forced press produced numerical instability. Failed recording preserved as `ear_grasp`. |
| Low side-tab grasp, level release, then press | Reached the seat but exceeded collision tolerances. Preserved as `tab_flat`. |
| Level release with alignment check | Declined to press a misaligned connector, leaving it on the rim. Preserved as `tab_flat_guarded`. |
| Higher side-tab grasp with the old fingertip lips | Seats the connector but the robot contacts the socket guides; failed clearance audit, preserved as `tab_direct`. |
| Pad-only side-tab grasp with continuous angled insertion | Selected as `pad_direct`; holds the original tabs until seated. Both 5 mm and 6 mm grasp-height trials are retained. |

The final geometry enables collisions on the existing side tabs, front and rear ridges, and blue wire support. Only the tiny contact markings and fixture decorations remain visual-only. The exported geometry is a video-inspired surrogate, not measured connector CAD.

## Robot and physics

The selected trial removes the previous fingertip support lips and uses the gripper pads to pinch the existing side tabs. No new gripper or connector geometry was added. The grasp uses a 46.8 mm jaw gap and a TCP 5 mm above the connector housing center. Measured in-hand pose calibration occurs after lifting and above the socket. Gripper contact moves the connector; its pose is never assigned by the controller.

Newton `SolverVBD` uses compliant ALM, 40 iterations, 32 substeps per 60 Hz output frame (1920 Hz physics), gravity 9.81 m/s², and a 15 g connector. Contact stiffness is 1e6 N/m and damping is 100; friction coefficients are 0.5 for the objects and 1.0 for the pads. Object contact gap is 0.2 mm and pad gap is 0.1 mm. A disabled bookkeeping joint works around this runtime's zero-joint array-shape issue; it is never enabled. There is no locking constraint or hidden snap into place.

The robot follows scripted kinematic URDF poses. No motor torque limits, perception, or real locking mechanism are modeled. Cables and their loads are deliberately excluded. These results establish an insertion strategy in this surrogate scene, not robustness on hardware or with the full harness.

## Validation

The selected motion passes all recorded checks for finite states, URDF joint bounds/speeds, robot FK agreement, seating after release, and sampled contacts. Maximum sampled connector overlap is 0.102 mm; maximum sampled robot overlap with the table or socket boxes is 0.000 mm. The overlap threshold is 0.2 mm. Contact audits use connector surface points at 15 Hz and robot mesh vertices at 10 Hz, not continuous swept-volume checks. Robot self-collision and the cylindrical fixture pedestal are not fully audited.

Videos replay recorded Newton poses through OpenGL; they are not Isaac Sim physics runs. Raw poses, controller targets, joint positions and phase labels are in `pad_direct.npz` at 60 Hz.

## Reproduce

Use the repository's pinned Newton/Warp environment and a CUDA GPU. From the repository root:

```bash
python tools/run.py connector-no-rib --output runs/connector-no-rib-01
```

Or from a copy of this experiment directory:

```bash
python simulate.py --strategy direct --grasp-gap .0468 --grasp-z 0.005 --no-lips --output pad_direct
python validate.py --motion pad_direct
python render.py --strategy pad_direct
```

The earlier release trials used `controller_unguarded.py` or `controller_release.py`, as recorded in `trial_summary.json`. Their parameters appear in each trial's JSON. Initial surrogate trials have their own asset and controller snapshots; do not validate them using the final collision geometry.
