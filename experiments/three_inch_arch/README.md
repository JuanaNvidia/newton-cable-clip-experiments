# Bring 12-inch cable endpoints to 3 inches apart

This separate Newton run preserves the preceding arch comparison. Cable length remains 304.8 mm and diameter remains 2 mm. Cyan is 0.1x the original bending rigidity, yellow is the original material, and red is the selected 2x material (EI = 0.00114591559 N m²). Each color is an independent comparison cable, not part of a colliding bundle.

After one second of gravity settling, both endpoint clamps translate symmetrically inward. Their separation follows a smooth motion from 240 mm to 76.2 mm over four seconds, then remains at 76.2 mm for three more seconds. Endpoint elevation stays 15 mm and endpoint orientations stay fixed at their original values. Only the clamps are driven: the cable middle is solved by Newton. There is no imposed final cable curve or midpoint load. Results depend on these endpoint orientation constraints.

Physics is Newton 1.6 SolverVBD, compliant ALM, 40 iterations, 3840 Hz. All material, mass, damping, contact, and rod discretization settings are unchanged from the previous arch test. Nonadjacent segment self-contact and board contact are enabled; independent comparison cables cannot collide with one another. There are 61 rod segments per cable. The rods have a straight rest state and start in an elastically bent arch.

`arch_comparison.mp4` is Newton OpenGL playback of the recorded physics, not an Isaac RTX render. `arch_motion.npz` includes all recorded poses and commanded endpoint spans. `arch_results.json` reports actual final endpoint separation, arch height, joint errors, and settling motion. The three cases are qualitative material comparisons, not measurements calibrated from the user's video.

Run with the Newton environment from the preceding package:

```sh
python arch_test.py
python render_arch.py
```

Final selected-material apex: 145.14 mm above the board, or 130.14 mm (5.12 inches) above the endpoints. Actual final endpoint spacing is 76.209 mm. Apex variation over the last second is 0.00194 mm. Validation checks in `validation.json` pass. The original material also remains arched; the 0.1x material slumps onto the board.
