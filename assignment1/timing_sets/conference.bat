cd ..

SET TESTNAME=Conference
SET EXENAME=bin/base_assignment1_Win32_Release.exe
SET EXENAMEEXAMPLE=reference_assignment1_32bit.exe
SET SCENE=states\standard set\conference_overview.dat

del "timing_results\%TESTNAME%.txt"

"%EXENAME%" "%SCENE%" "timing_results/%TESTNAME%.txt" submission -bat_render -output_images
"%EXENAMEEXAMPLE%" "%SCENE%" "timing_results/%TESTNAME%.txt" reference -bat_render -output_images

timing_results\plotter "%~dp0..\timing_results\%TESTNAME%.txt"
