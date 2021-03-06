%% Test WISCO_motion
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%==========================================================================
close all;
sto_file = 'C:\github\wisco_opensim\build\examples\exampleFBKneePassiveFlexion\results\_ForceReporter_forces.sto';

lig = WISCO_Ligament_Analysis(sto_file);

lig.plot_ligament_bundles({'all'},'force_total')
lig.plot_ligament_bundles({'mcl'},'force_total')
lig.plot_ligament_bundles({'acl'},'force_total')
lig.plot_ligament_bundles({'pcl'},'force_total')
lig.plot_ligament_bundles({'lcl'},'force_total')
lig.plot_ligament_bundles({'pt'},'force_total')
lig.plot_ligament_bundles({'lpfl'},'force_total')
lig.plot_ligament_bundles({'mpfl'},'force_total')
lig.plot_ligament_bundles({'MCLp'},'force_total')
lig.plot_ligament_bundles({'PFL'},'force_total')
lig.plot_ligament_bundles({'pCAP'},'force_total')
lig.plot_ligament_bundles({'itb'},'force_total')
