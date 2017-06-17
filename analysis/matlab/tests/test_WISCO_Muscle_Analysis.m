%% Test WISCO_motion
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%==========================================================================
close all;
sto_file = 'C:\github\wisco_opensim\build\examples\exampleFBKneePassiveFlexion\results\_MuscleAnalysis_TendonForce.sto';

msl = WISCO_Muscle_Analysis(sto_file);

msl.plot_muscle_param({'all'},'force')
for i = 1:msl.nMuscles
    msl.plot_muscle_param({msl.names{i}},'force');
end