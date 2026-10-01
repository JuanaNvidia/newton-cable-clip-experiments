# Repository preparation checks

Verified on Linux / Python 3.12 during repository preparation:

- Artifact hashes, expected validation statuses, finite reference pose arrays, and Python syntax pass the CPU integrity checker.
- Original clip mesh copies are identical between insertion experiments.
- The independent three-inch pose validator reproduces endpoint-spacing, clamp-error, and joint-continuity checks from the stored motion.
- A fresh copied baseline experiment successfully constructs the Newton model, captures the CUDA graph, and simulates 0.05 seconds at 3840 Hz with finite recorded motion on an RTX 6000 Ada.
- The experiment runner's command-line interface parses successfully.
- Packaging scan found no personal absolute filesystem paths, credential patterns, or files over 90 MiB in the publishable tree.

The short GPU run is a packaging smoke test, not a repeat of full insertion validation. Full reference simulation/validation results are the original saved recordings and reports. Optional Isaac RTX rendering was not rerun during packaging. The GitHub Actions workflow runs these integrity checks on pushes and pull requests; see the Actions tab for its current status.

The corrected fixed-receiver experiment was fully simulated for 7 seconds at 3840 Hz and independently validated. The black receiver pose is bitwise constant over all 420 recorded frames. After hand release the maximum seat error is 0.504 mm and angular error is 1.883 degrees; maximum sampled plug/receiver penetration is below 0.004 mm. This checks seating, not positive locking. The 420-frame USD playback preserves the fixed receiver and contains no hinge geometry.

The combined 18-inch experiment was simulated for 14 seconds with six 91-segment cables. Required insertion and cable-retention checks pass for the preloaded-spring variant; full lid closure is a separate failed diagnostic (3.43 degrees open). All six cables remain inside the original closed-cavity bounds throughout the 1.8-second final release period. The original-spring comparison intentionally fails retention and settles about 31.52 degrees open. Both numerical reports are included. The compact USD matches all 549 recorded bodies exactly at four sampled frames.
