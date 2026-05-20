# Mechanism SDF Design Ledger

## Scope

Generate one SDF 1.12 model per downloaded Thang010146 mechanism under
`models/mechanisms/*/<mechanism>.sdf`. These files target CAD Explorer
rendering. They are not intended to be physically complete Gazebo dynamics
models.

## Source Of Truth

- `models/mechanisms/sdf_common.py` contains the shared generator.
- Each mechanism directory has `assets/gen_sdf.py`; generated top-level SDF
  files should not be hand edited.
- `assets/source_step.zip` and `assets/.source_step_files/` contain the STEP
  source package from the video description.
- `assets/source_inventor.zip` and `assets/.source_inventor_files/` contain the
  matching Inventor package from the video description.

## Geometry

- The top-level `<mechanism>.step` is the user-facing assembly STEP.
- SDF visuals reference per-occurrence STL meshes in
  `assets/.sdf_meshes/`.
- Colors are authored in the SDF material elements so CAD Explorer can visually
  separate fixed frames, driven bodies, gears, fasteners, springs, belts, and
  linkages.
- Collision geometry and inertials are intentionally omitted in this pass.

## Frames And Poses

- Each generated model has one root link named `mechanism_root`.
- Each rendered STEP occurrence becomes one SDF link.
- Occurrence links are attached to `mechanism_root` by fixed joints whose poses
  come directly from the exported assembly GLB topology.
- The generator no longer creates guessed revolute or prismatic joints for
  closed-loop mechanisms. Those guesses made many assemblies look plausible in
  XML while breaking the mechanism visually.

## Motion

These generated SDFs are static structural reviews. The generator does not add
CAD Explorer-specific motion plugins or animation metadata. Source alternate
assembly states and Inventor files remain archived as upstream evidence, but
they are not encoded into SDF animation behavior.

## Validation

- Regenerate with a Python script that imports `gen_mechanism_sdf()` from
  `models/mechanisms/sdf_common.py` and writes each
  `models/mechanisms/<mechanism>/<mechanism>.sdf`.
- Validate generated XML and mesh URI existence for every mechanism.
- Run `npm --prefix skills/cad-explorer/scripts/explorer test`.
- Smoke-test SDFs in CAD Explorer; the viewer must render the assembly as a
  static robot-description model with direct inspection controls.
