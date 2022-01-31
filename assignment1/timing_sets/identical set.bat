cd ..

SET TESTNAME=indentical sets
SET EXENAME=bin/base_assignment1_Win32_Release.exe

del "timing_results\%TESTNAME%.txt"

FOR /R %%G in ("states\standard set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" first -bat_render -builder SAH -output_images
FOR /R %%G in ("states\standard set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" second -bat_render -builder SAH
FOR /R %%G in ("states\standard set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" third -bat_render -builder SAH

timing_results\plotter "%~dp0..\timing_results\%TESTNAME%.txt"
