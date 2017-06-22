%% Analyze Passive Knee Simulation
%==================================
close all;

h5_file = 'C:\github\wisco_opensim\build\examples\exampleFBKneePassiveFlexion\results\ball_and_floor.h5';

h5 = WISCO_h5_Analysis(h5_file);

if(1)
    h5.lig.plot_ligament_bundles({'all'},'force_total')
    h5.lig.plot_ligament_bundles({'mcl'},'force_total')
    h5.lig.plot_ligament_bundles({'aclam'},'force_total')
    h5.lig.plot_ligament_bundles({'aclpl'},'force_total')
    h5.lig.plot_ligament_bundles({'pcl'},'force_total')
    h5.lig.plot_ligament_bundles({'lcl'},'force_total')
    h5.lig.plot_ligament_bundles({'pt'},'force_total')
    h5.lig.plot_ligament_bundles({'lpfl'},'force_total')
    h5.lig.plot_ligament_bundles({'mpfl'},'force_total')
    h5.lig.plot_ligament_bundles({'mclp'},'force_total')
    h5.lig.plot_ligament_bundles({'pfl'},'force_total')
    h5.lig.plot_ligament_bundles({'pcap'},'force_total')
    h5.lig.plot_ligament_bundles({'itb'},'force_total')
end

if (1)
    h5.msl.plot_muscle_param({'all'},'force')
    
    msl_names = h5.msl.names;
    nMsl = h5.msl.nMuscles;
    
    %for i = 1:nMsl
    %    h5.msl.plot_muscle_param({msl_names{i}},'force')
    %end
    
    %h5.msl.plot_muscle_param({'addbrev_r'},'force')
end

if (1)
    figure('name','TF Total Contact Force')
    hold on;
    plot(h5.cnt.data.TF_contact.tibiaMesh.total.contact_force(1,:),'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.total.contact_force(2,:),'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.total.contact_force(3,:),'LineWidth',2)
    legend('x','y','z')
    
    figure('name','TF Medial Contact Force')
    hold on;
    plot(h5.cnt.data.TF_contact.tibiaMesh.medial.contact_force(1,:),'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.medial.contact_force(2,:),'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.medial.contact_force(3,:),'LineWidth',2)
    legend('x','y','z')
    
    figure('name','TF Lateral Contact Force')
    hold on;
    plot(h5.cnt.data.TF_contact.tibiaMesh.lateral.contact_force(1,:),'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.lateral.contact_force(2,:),'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.lateral.contact_force(3,:),'LineWidth',2)
    legend('x','y','z')
    
    figure('name','PF Total Contact Force')
    hold on;
    plot(h5.cnt.data.PF_contact.patellaMesh.total.contact_force(1,:),'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.total.contact_force(2,:),'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.total.contact_force(3,:),'LineWidth',2)
    legend('x','y','z')
    
    figure('name','PF Medial Contact Force')
    hold on;
    plot(h5.cnt.data.PF_contact.patellaMesh.medial.contact_force(1,:),'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.medial.contact_force(2,:),'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.medial.contact_force(3,:),'LineWidth',2)
    legend('x','y','z')
    
    figure('name','PF Lateral Contact Force')
    hold on;
    plot(h5.cnt.data.PF_contact.patellaMesh.lateral.contact_force(1,:),'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.lateral.contact_force(2,:),'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.lateral.contact_force(3,:),'LineWidth',2)
    legend('x','y','z')
    
    figure('name','TF Contact Pressure')
    subplot(2,1,1);hold on
    title('max pressure')
    plot(h5.cnt.data.TF_contact.tibiaMesh.total.max_pressure,'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.medial.max_pressure,'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.lateral.max_pressure,'LineWidth',2)
    legend('total','med','lat')
    
    subplot(2,1,2);hold on
    title('mean pressure')
    plot(h5.cnt.data.TF_contact.tibiaMesh.total.mean_pressure,'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.medial.mean_pressure,'LineWidth',2)
    plot(h5.cnt.data.TF_contact.tibiaMesh.lateral.mean_pressure,'LineWidth',2)
    legend('total','med','lat')
    
    figure('name','PF Contact Pressure')
    subplot(2,1,1);hold on
    title('max pressure')
    plot(h5.cnt.data.PF_contact.patellaMesh.total.max_pressure,'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.medial.max_pressure,'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.lateral.max_pressure,'LineWidth',2)
    legend('total','med','lat')
    
    subplot(2,1,2);hold on
    title('mean pressure')
    plot(h5.cnt.data.PF_contact.patellaMesh.total.mean_pressure,'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.medial.mean_pressure,'LineWidth',2)
    plot(h5.cnt.data.PF_contact.patellaMesh.lateral.mean_pressure,'LineWidth',2)
    legend('total','med','lat')
    
end
