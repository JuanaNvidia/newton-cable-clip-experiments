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

The combined `connector_and_clip` experiment reuses the corrected connector assets and byte-identical original clip STL files. It adds no measured customer CAD. Connector retention is a force-model approximation rather than a mesh-derived locking catch.

## UR5 and Robotiq assets

The robot experiment adds ROS-Industrial UR5 CB3 meshes/default kinematics (BSD-3-Clause) and Robotiq 2F-85 meshes/kinematic dimensions from MuJoCo Menagerie (BSD-2-Clause). Original licenses and pinned upstream commits are included in `experiments/ur5_cable_clip/robot_assets`. Source files were verified against upstream Git blob hashes. The Menagerie XML is only an asset/kinematics source: the experiment runs no MuJoCo physics. See the experiment README for source links.

The interior-table UR5 variant reuses those same licensed robot assets and original clip meshes. Its second connector is another instance of the existing approximate plug geometry; the larger tabletop is procedural box geometry.

`ur5_interior_clip/robot.py` adds procedural wedge support inserts to the Robotiq fingertips. These are new experiment geometry, separate from the unchanged upstream robot mesh files; they participate in collision and rendering.

## Three-clip harness assembly

`experiments/ur5_three_clips` reuses the original unscaled clip STL files and the licensed UR5/Robotiq assets from the earlier robot experiments. Its approximate connector meshes add T-shaped handling ribs. Table sockets, tabletop and fingertip support lips are generated geometry. The customer video and correspondence are excluded.
