This folder has examples of timing measurements - a set of batch files that can automatically render a set of images offline
and measure the BVH building and ray tracing times to give useful information about the performance of your solution. These
results are also automatically plotted in a bar graph to make comparison of different methods, scenes and implementations easy.
This is convenient for the ray tracing optimization extra credit assignment - you can measure your code against the reference
solution by running the comparison.bat. The goal is to be at least as fast as the reference, but stopping there is in no way
mandatory!

To make a new timing measurement, copy one of the existing and change the TESTNAME variable to something reasonable.
The executables arguments are as follows:

assignment1_Win32_Release.exe scenefile resultfile measurement_name -bat_render [additional options]

where scenefile is the scene to render (.dat file), resultfile is where the results will be added (.txt) and measurement_name is
the name for this particular set of options (the plotter uses this name to differentiate between sets)

The following options are available:
-bat_render: performs an offline render, without this you get the interactive view and all of the arguments are ignored.
-output_images: outputs the renderings into the images/ folder
-builder (followed by method): choose the BVH builder method (from "none", "sah", "object_median", "spatial_median")
-spp: how many anti aliasing or ambient occlusion samples per pixel
-ao and -aa: sets anti aliasing or ambient occlusion sampling type (-aa by default)
-ao_length (followed by float): sets the AO sampling length
-use_textures: enables texturing (after implemented)

These are parsed in App::process_args, you can obviously add features as you please.

You can call the executable from a measurement bat either in a loop (as in "identical set.bat" which has three sets with the same
options to demonstrate how the timing is not completely precise) to render all of the views/states in the states/ folder, or render
a single view/state as in "single state example.bat".

New state files can be created by running the executable, selecting the desired scene and view and pressing Alt+[number]. The
new file, named "state_assignment1_[number].dat", will appear in the project folder. You can then rename it
and move it to the states/ folder to keep things tidy and easily loopable.