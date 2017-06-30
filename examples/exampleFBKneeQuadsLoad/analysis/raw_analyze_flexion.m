%% test analyze passive

file = 'C:\github\wisco_opensim\build\examples\exampleFBKneePassiveFlexion\results\fbknee_passive_flex.mot';

[data lab hdr] = read_opensim_mot(file);
 time = data(:,1);
 
%Plot Knee
figure('name','TF Kinematics')
subplot(3,2,1); hold on
title('Flexion')
ind = find(strcmp(lab,'knee_r/knee_flex_r/value'));
plot(time, data(:,ind)*180/pi)

subplot(3,2,3); hold on
title('Adduction')
ind = find(strcmp(lab,'knee_r/knee_add_r/value'));
plot(time, data(:,ind)*180/pi)

subplot(3,2,5); hold on
title('Rotation')
ind = find(strcmp(lab,'knee_r/knee_rot_r/value'));
plot(time, data(:,ind)*180/pi)

subplot(3,2,2); hold on
title('Tx')
ind = find(strcmp(lab,'knee_r/knee_tx_r/value'));
plot(time, data(:,ind)*10^3)

subplot(3,2,4); hold on
title('Ty')
ind = find(strcmp(lab,'knee_r/knee_ty_r/value'));
plot(time, data(:,ind)*10^3)

subplot(3,2,6); hold on
title('Tz')
ind = find(strcmp(lab,'knee_r/knee_tz_r/value'));
plot(time, data(:,ind)*10^3)

%Plot Patella
figure('name','pF Kinematics')
subplot(3,2,1); hold on
title('Flexion')
ind = find(strcmp(lab,'pf_r/pf_flex_r/value'));
plot(time, data(:,ind)*180/pi)

subplot(3,2,3); hold on
title('Rotation')
ind = find(strcmp(lab,'pf_r/pf_rot_r/value'));
plot(time, data(:,ind)*180/pi)

subplot(3,2,5); hold on
title('Tilt')
ind = find(strcmp(lab,'pf_r/pf_tilt_r/value'));
plot(time, data(:,ind)*180/pi)

subplot(3,2,2); hold on
title('Tx')
ind = find(strcmp(lab,'pf_r/pf_tx_r/value'));
plot(time, data(:,ind)*10^3)

subplot(3,2,4); hold on
title('Ty')
ind = find(strcmp(lab,'pf_r/pf_ty_r/value'));
plot(time, data(:,ind)*10^3)

subplot(3,2,6); hold on
title('Tz')
ind = find(strcmp(lab,'pf_r/pf_tz_r/value'));
plot(time, data(:,ind)*10^3)