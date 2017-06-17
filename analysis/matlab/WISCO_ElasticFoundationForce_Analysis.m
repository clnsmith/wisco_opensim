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
        sto_file;
        raw;
        
        time;
        frame_number;
        nFrames;
        force;
        cop;
        
    end
    methods
        function obj = WISCO_ElasticFoundationForce_Analysis(file)
            [pathstr,name,ext] = fileparts(file);
            
            if (strcmp(ext,'h5'))
                
            end
            
            
            if (strcmp(ext,'.sto'))
                obj.sto_file = file;

                %Read ForceReporter.sto file
                %---------------------------
                if (exist(file,'file') ~=2)
                    fprintf('File: %s',file);
                    error('File does not exist!')                
                end

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
    