set CNT_ANALYSIS_SETTINGS=C:\github\wisco_opensim\source\examples\cmd\visualizeContact\ContactAnalysis_settings.xml
set PLUGIN=C:\github\wisco_opensim\install\plugin\WISCO_Plugin.dll

opensim-cmd -L %PLUGIN% run-tool %CNT_ANALYSIS_SETTINGS%

pause
