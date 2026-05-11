# Pelican Bicycle Assets

Generated CAD and URDF artifacts for a stylized pelican riding and pedaling a
bicycle.

- STEP source of truth: `pelican_riding_bicycle.py`
- Generated STEP model: `pelican_riding_bicycle.step`
- STEP Explorer sidecar: `.pelican_riding_bicycle.step.glb`
- URDF source of truth: `pelican_pedal_bike.py`
- Generated robot description: `pelican_pedal_bike.urdf`
- URDF CAD Explorer metadata: `.pelican_pedal_bike.urdf/explorer.json`
- URDF reusable visual meshes: `meshes/*.stl`

The URDF is a tree, so the pedal, wheel, and leg motion is represented with
`mimic` joints driven by `pedal_cycle_joint` rather than a true closed-loop
chain/leg mechanism. Scrub the `pedal_cycle_joint` slider in CAD Explorer to
see the cranks, wheels, hips, and knees move together. The leg offsets are tuned
so each orange foot center lands on its pedal center at the saved stroke poses.
