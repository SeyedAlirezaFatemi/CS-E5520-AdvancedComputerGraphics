cd ..

SET TESTNAME=builder comparison
SET EXENAME=bin/base_assignment1_Win32_Release.exe

del "timing_results\%TESTNAME%.txt"

FOR /R %%G in ("states\builder set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" SAH -bat_render -ao -spp 16 -builder SAH
FOR /R %%G in ("states\builder set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" spatial -bat_render -ao -spp 16 -builder spatial_median
FOR /R %%G in ("states\builder set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" object -bat_render -ao -spp 16 -builder object_median
FOR /R %%G in ("states\builder set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" linear -bat_render -ao -spp 16 -builder linear

timing_results\plotter "%~dp0..\timing_results\%TESTNAME%.txt"
