# Sculpted A hypercar

Concept A interpreted as a procedural, simulator-ready mid-engine hypercar:
metallic silver body, swept LED headlights, dark canopy, sculpted door coves,
deep side intakes, carbon splitter and diffuser, ten split-spoke center-lock
wheels and an active rear wing. This mesh is an interpretation of the concept,
not an exact reproduction of the generated image.

## Files

- `car_hyper.glb`: runtime model, embedded materials/textures.
- `car_hyper.rig.json`: existing simulator rig convention, ISO axes in metres.
- `car_hyper.blend`: editable Blender import with studio lighting and camera.
  The studio objects are preview helpers; the GLB is the runtime asset.
- `preview-front.png`, `preview-rear.png`: Blender studio renders of the mesh.
- `preview-rig.png`: butterfly doors open, steering and active wing posed.

## Rebuild and check

From the simulator root:

```powershell
py tools/assetgen/build.py car_hyper
py tools/assetgen/validate.py --determinism car_hyper
blender -b --factory-startup -P tools/assetgen/blender_check.py -- car_hyper out.png
```

The authoritative source is `tools/assetgen/assetgen/vehicles/car_hyper.py`.
Editing the `.blend` does not update the procedural generator.

Validation: 115,440 triangles, 3.09 MB GLB, 27 nodes, 19 joints, 5 sockets,
6 light functions. Structural and byte-identical rebuild checks pass.
Blender independently imported the model and checked/posed the rig.
The asset has not been exercised in a live simulator session.

Cockpit revision: enclosed footwells and wheel-well backs, full-height door
cards attached to butterfly doors, fitted quarter trim, roof liner, padded
dash with instruments/vents, centre screen and controls. The wraparound
windscreen corners are glass; opaque outer trim no longer fills those windows.
Verified front/side/rear eye-level renders in Blender and a driver-eye screenshot
in Godot's mobile renderer. VR head tracking itself remains untested.
Source: tools/assetgen/assetgen/vehicles/hyper_interior.py.

Live digital cluster: speed dial and km/h, RPM dial (0-9000) with redline, gear R/N/1-7, trip distance, fuel percentage/litres, instantaneous L/100 km (L/h below 5 km/h), trip average and estimated range after 100 m. Screen is fitted to the cabin and rendered in 3D for desktop and XR cameras. Fuel uses the existing core snapshot flow/tank mass, exposed through Godot's powertrain dictionary; gauge metadata supplies actual tank capacity and fuel density. Authored tank: 75 L E5, initial 60 L. Trip averages/range are session estimates, based on this fictional engine's existing crude consumption model.

Dashboard layout follows the owner's sketch: vertical fuel bar left, large speed/RPM dials, gear centre, stacked drive-power/boost auxiliaries right, and bottom trip/consumption strip. DRIVE kW is signed clutch-transmitted power (torque times engine angular speed); boost is unavailable for the current torque-map engine and displays --. Verified live in Godot mobile rendering.
