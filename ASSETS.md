# Asset provenance and rights

- `clipTop.stl` and `clipBottom.stl`: user-supplied clip geometry. The copies in both insertion experiments match byte-for-byte. Original model ownership/license was not supplied.
- `connector_displayport.usd`: connector asset supplied in the original local project. Original model ownership/license was not supplied.
- `actual_clip_scene.usda` / `.usdc`: derived scenes incorporating those geometries, materials, the fitted hinge arrangement, and cable visuals. Newton physics is constructed in Python.
- Videos, preview images, NPZ motion, and playback USD files: generated during these experiments.
- Newton, Warp, and all other dependencies are installed separately and retain their respective upstream licenses. Their source/runtime packages are not vendored here.

This repository is intended for private collaboration. No new blanket open-source license or third-party asset redistribution rights are asserted. Confirm rights with the asset owners before making the repository public or redistributing their models separately.

The reference footage and customer conversation are not included. No GitHub credentials or workstation environment are part of the repository.

The hinged-connector extension uses newly generated parametric surrogate geometry from `experiments/hinged_connector_insertion/assets.py`. Its USD/STL assets do not incorporate the prior DisplayPort connector or original clip STLs. The hidden mechanism and dimensions are assumptions based on visual reference. The reference video itself remains excluded.

The corrected `fixed_receiver_insertion/assets.py` removes all receiver hinge hardware and generates `fixed_table_connector` assets. The original hinged extension is a superseded interpretation. Both are approximate surrogate geometry; neither establishes the real concealed locking mechanism.
