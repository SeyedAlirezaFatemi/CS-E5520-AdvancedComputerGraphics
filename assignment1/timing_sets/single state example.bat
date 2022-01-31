cd ..

SET TESTNAME=single state
SET EXENAME=bin/base_assignment1_Win32_Release.exe

del "timing_results\%TESTNAME%.txt"

"%EXENAME%" "states\standard set\conference_overview.dat" "timing_results/%TESTNAME%.txt" conference -bat_render -builder SAH -output_images

timing_results\plotter "%~dp0..\timing_results\%TESTNAME%.txt"
