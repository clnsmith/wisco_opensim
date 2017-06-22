%% WISCO_ElasticFoundationForce_Analysis class 
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%
%==========================================================================
classdef WISCO_ElasticFoundationForce_Analysis
    properties
        sto_file
        raw
        
        time
        frame_number
        nFrames
        data
        nForces
        force_names
        
    end
    methods
        function obj = WISCO_ElasticFoundationForce_Analysis(file)
        %==================================================================
        %CLASS CONSTRUCTER
        %------------------------------------------------------------------
        %file : full path to .h5 or .sto file to read and analyze 
        %(include extension)
        %==================================================================
        
            %Read ForceReporter.sto file
            %---------------------------
            if (exist(file,'file') ~=2)
                fprintf('File: %s',file);
                error('File does not exist!')                
            end
            [pathstr,name,ext] = fileparts(file);
            
            if (strcmp(ext,'.h5'))
                obj.time = h5read(file,'/time');
                
                info = h5info(file,'/WISCO_ElasticFoundationForce');
                
                obj.nForces = length(info.Groups);
                
                obj.force_names = cell(obj.nForces,1);
                
                for i = 1:obj.nForces
                    label = strsplit(info.Groups(i).Name,'/');
                    obj.force_names{i} = label{3};
                    
                    nMesh = length(info.Groups(i).Groups);
                    
                    for j = 1:nMesh
                        label = strsplit(info.Groups(i).Groups(j).Name,'/');
                        mesh_name = label{4};
                        
                        nType = length(info.Groups(i).Groups(j).Groups);
                        
                        for k = 1:nType
                            label = strsplit(info.Groups(i).Groups(j).Groups(k).Name,'/');
                            type_name = label{5};
                            
                            nParam = length(info.Groups(i).Groups(j).Groups(k).Datasets);
                                                       
                            for l = 1:nParam
                                param_name = info.Groups(i).Groups(j).Groups(k).Datasets(l).Name;
                                                                
                                data_set_name = ['/WISCO_ElasticFoundationForce/' obj.force_names{i} '/' mesh_name '/' type_name '/' param_name];
                                obj.data.(obj.force_names{i}).(mesh_name).(type_name).(param_name) = h5read(file,data_set_name);
                            end
                        end
                    end
                end
                        
                        
            elseif (strcmp(ext,'.sto'))
                obj.sto_file = file;

 

                [data,labels,header] = read_opensim_sto(file);
                obj.raw.data = data;
                obj.raw.labels = labels;
                obj.raw.header = header;
                obj.raw.nLabels = length(labels);
                obj.raw.component_names = cell(obj.raw.nLabels,1);
                obj.raw.param_names = cell(obj.raw.nLabels,1);

                for i = 1:obj.raw.nLabels
                    [ss, match] = strsplit(labels{i},{'/','|'});
                    if(length(match)>0)
                        obj.raw.model_names{i} = ss{2};
                        obj.raw.force_names{i} = ss{3};
                        obj.raw.param_names{i} = ss{4};
                    else
                        obj.raw.model_names{i} = '';
                        obj.raw.force_names{i} = '';
                        obj.raw.param_names{i} = '';
                    end
                end

                % Time and Frame
                %---------------
                timeI = strcmpi('time',labels);
                obj.time = data(:,timeI);
                obj.frame_number = 1:length(obj.time);
                obj.nFrames = length(obj.time);

                % Contact Force
                %--------------
                ind = strfind(obj.raw.param_names,'total_contact_force');
                frc_tot_i = find(not(cellfun('isempty', ind)));
                frc_tot_col = 2+(frc_tot_i-2)*3;

                ind = strfind(obj.raw.param_names,'medial_contact_force');
                frc_med_i = find(not(cellfun('isempty', ind)));
                frc_med_col = 2+(frc_med_i-2)*3;

                ind = strfind(obj.raw.param_names,'lateral_contact_force');
                frc_lat_i = find(not(cellfun('isempty', ind)));
                frc_lat_col = 2+(frc_lat_i-2)*3;

                obj.force.casting.total = data(:,frc_tot_col:frc_tot_col+2);
                obj.force.casting.medial = data(:,frc_med_col:frc_med_col+2);
                obj.force.casting.lateral = data(:,frc_lat_col:frc_lat_col+2);
            end
        end
        
        function plot_contact_forces(obj,mesh_type,stat_type)
        %==================================================================
        %mesh_type : str
        %   'casting' or 'target'
        %stat_type : str
        % 'total' 'medial' or 'lateral'
        %==================================================================
        figure('name',[mesh_type ':' stat_type ':contact_force'])
        hold on;
        
        plot(obj.time,obj.force.(mesh_type).(stat_type)(:,1))
        plot(obj.time,obj.force.(mesh_type).(stat_type)(:,2))
        plot(obj.time,obj.force.(mesh_type).(stat_type)(:,3))
        legend('x','y','z')
        end
    end
end
    