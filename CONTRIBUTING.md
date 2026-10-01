# Iterating on the experiments

Create a branch and use `runs/` for scratch work; it is ignored by Git. Preserve the reference recordings so that improvements remain comparable. Include the exact code/configuration, Newton/Warp versions, GPU, timestep, solver iterations, and seed if you introduce randomness.

## Where to edit

- Insertion length/radius/material: `experiments/long_cable_newton/cable_config.py` and `simulate_newton.py`.
- Baseline insertion material: `experiments/six_cable_newton/simulate_newton.py`.
- Arch material: `base_EI`, `factors`, and `bend_damping` in `arch_test.py`.
- Arch length/diameter: `L`, `N`, `r`; the current GPU midpoint indexing and render geometry also assume 61 segments and 1 mm radius. Update these together.
- 3-inch endpoint motion: `move_clamps` in `experiments/three_inch_arch/arch_test.py`; update validation targets when changing final span.
- Contact settings: cable and board `ShapeConfig` are independent. Do not assume the arch board has the insertion board's settings.
- Grip controller: the `grasp` kernel in each insertion simulation.

Keep EI constant when changing segment length, and set per-joint bending stiffness to EI / segment length. Maintain the mass correction and analytic inertia restoration. Decreasing the physics frequency can destabilize the explicit grip controller even if the rod material is unchanged.

## Validation

Run `python tools/check_repository.py` to verify artifact hashes, recorded arrays, report status, and Python syntax without importing Newton. For new simulations, rerun the relevant experiment validator. The 3-inch validator recomputes endpoint travel and joint continuity directly from recorded poses. Insertion validation additionally checks sampled geometry, cable ordering, connector attachments, and retention. Repeat the contact-disabled control when changing clip/contact behavior.

Review the rendered motion as well as the numbers. Report any changes to boundary conditions, friction, density, damping, collision filtering, or inertia alongside stiffness changes. Do not interpret comparisons with different constraints as a material-only study.

For a proposed improvement, include a short description, preview, parameter changes, validation JSON, and reproducible command. Use a new result folder rather than replacing reference data. Regenerate `artifact_manifest.json` with `python tools/check_repository.py --write-manifest` only after intentionally updating reference artifacts, and review the resulting diff.

## Rest curvature

Read [the modeling correction](docs/MODELING_CORRECTION.md) before interpreting or extending the arch experiments. In Newton 1.6, initially curved body poses become cached rod rest curvature unless explicitly changed. The new hinged-connector example zeroes those caches; do not assume identity anchor rotations alone make a straight-rest cable.
