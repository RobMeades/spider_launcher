# Introduction
This directory contains the 3D printed parts and printing instructions for the spider launcher

The main file is `spider_launcher.blend`, the components of which are exported to a number of `spider_launcher*.stl` files at a Blender scale factor of 1000 to give real size in millimetres. `_xY` on the end of an `stl` file name means you will need to print `Y` of those parts (e.g. two of `spider_launcher_nylon_line_guide_x2.stl`).

# Printing
All parts should be printed in \[black\] ASA, 15% in-fill, fastest speed, no supports required, brim if you think you need it, except for the following:

- `spider_launcher_nylon_line_guide_x2.stl`: print at a higher resolution (e.g. 0.1&nbsp;mm layer height) if you can, for a smoother edge,
- `spider_launcher_pole_bug_eyes.stl`: best sliced in half, width-wise, in your printer's slicer program, then both halves can be placed on their cut faces, printed (at 0.1&nbsp;mm layer for best curves) with supports on the print bed and cyanoacrylated together afterwards,
- `spider_launcher_pole_top_bracket.stl`: similarly, can be chopped into two, in the middle of the thickest part, in your printer's slicer program, then the two parts can be cyanoacrylated together afterwards,
- `spider_launcher_winch_mount_motor_side.stl`: there is no way around having supports everywhere for this one,
- `spider_launcher_winch_mount_non-motor_side_upper.stl`, `spider_launcher_winch_mount_non-motor_side_lower.stl`, `spider_launcher_winch_wheel.stl`, `spider_launcher_winch_wheel_nylon_line_bead_x2.stl` and `spider_launcher_box_lid.stl` will require supports on the print bed.