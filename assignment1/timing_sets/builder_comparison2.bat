cd ..

SET TESTNAME=builder comparison
SET EXENAME=bin/base_assignment1_Win32_Release.exe
SET EXENAMEEXAMPLE=reference_assignment1_32bit.exe

del "timing_results\%TESTNAME%.txt"

FOR /R %%G in ("states\builder set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" sub_sah -bat_render -ao -spp 16 -builder SAH
FOR /R %%G in ("states\builder set\*") do "%EXENAME%" "%%G" "timing_results/%TESTNAME%.txt" sub_morton -bat_render -ao -spp 16 -builder morton

FOR /R %%G in ("states\builder set\*") do "%EXENAMEEXAMPLE%" "%%G" "timing_results/%TESTNAME%.txt" ref_sah -bat_render -ao -spp 16 -builder SAH
FOR /R %%G in ("states\builder set\*") do "%EXENAMEEXAMPLE%" "%%G" "timing_results/%TESTNAME%.txt" ref_linear -bat_render -ao -spp 16 -builder linear

timing_results\plotter "%~dp0..\timing_results\%TESTNAME%.txt"
