cd ..

SET TESTNAME=comparison
SET EXENAME=bin/base_assignment1_Win32_Release.exe
SET EXENAMEEXAMPLE=reference_assignment1_32bit.exe

del "timing_results\%TESTNAME%.txt"

FOR /R %%G in ("states\standard set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" submission -bat_render -ao -spp 16 -output_images
FOR /R %%G in ("states\standard set\*") do "%EXENAMEEXAMPLE%" "%%G" "timing_results/%TESTNAME%.txt" reference -ao -spp 16 -bat_render

timing_results\plotter "%~dp0..\timing_results\%TESTNAME%.txt"
