# Description
  DexArm supported 4x4 mesh bed leveling manual. Auto bed leveling whth BLTouch will be supported later.

# Steps
1. Clear the old levelling factor,`M891 X0 Y0`, repowerup DexArm.
2. Move Dexarm to HOME Position by HOME button or `M1112`.
3. Use `G29 S0` to get the current status and mesh. If there’s an existing mesh, you can send M420 S1 to use it.
4. Use `G29 S1` to move to the first point for Z adjustment.
5. Adjust Z so a piece of paper can just pass under the nozzle.
6. Use `G29 S2` to save the Z value and move to the next point.
7. Repeat steps 3-4 until completed.
8. Use `M500` to save the mesh to EEPROM, if desired.
9. `G29 S4` set global Z offset.
10. `M420 S1` Enable bed leveling
11. Adjust Z axis height，set origin according to your slicer, `G92 X0 Y0 Z0` or `G92 Z0`, then start 3D printing.

# Notes
- Probe point position:
```
  (-45， 255), (-15， 255), (15， 255), (45， 255)

  (-45， 285), (-15， 285), (15， 285), (45， 285)

  (-45， 315), (-15， 315), (15， 315), (45， 315)

  (-45， 345), (-15， 345), (15， 345), (45， 345)
```
- More about [G29](https://raw.githubusercontent.com/Rotrics-Dev/DexArm_API/master/gcode/%5BMarlin%5D%20G029%20-%20Bed%20Leveling%20Manual.md)

# Examples
Modify some mesh points and view the new mesh:
```
> G29 S3 I2 J2 Z0.042
> G29 S3 I1 J1 Z-0.666
> G29 S0
3x3 mesh. Z offset: 0
Measured points:
       0      1       2
0 +0.011 -0.020  -0.026
1 +0.017 -0.666  -0.019
2 +0.022 -0.030  +0.042
```

set global Z offset
```
G29 S4 Z101.6
```

# In Addition
I save the leveling data in gcode and send it to dexarm if needed.
Also for your reference :)
[Mesh_Bed_Leveling_Sample.gcode](Mesh_Bed_Leveling_Sample.gcode)