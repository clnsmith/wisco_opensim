%% Test WISCO_motion
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%==========================================================================
%close all;
sto_file = 'C:\github\wisco_opensim\build\examples\exampleFBKneePassiveFlexion\results\PF_contact_ReporterVec3.sto';

frc = WISCO_ElasticFoundationForce_Analysis(sto_file);

%fid = fopen(sto_file);
%x=textscan(fid,'%s','Delimiter',{'\t',','},'MultipleDelimsAsOne',1,'HeaderLines',4)%,'EndOfLine','\r\n');
%a = reshape(x{1,1},19,[]);
%textscan(s,'%s','Delimiter',{' ',','},'MultipleDelimsAsOne',1)

frc.plot_contact_forces('casting','total')
frc.plot_contact_forces('casting','medial')
frc.plot_contact_forces('casting','lateral')