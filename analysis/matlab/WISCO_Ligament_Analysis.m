%% WISCO_Ligament_Analysis class 
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%
%==========================================================================
classdef WISCO_Ligament_Analysis
    properties
        sto_file;
        raw;
        
        time;
        frame_number;
        nFrames;
    end
    methods
        function obj = WISCO_Ligament_Analysis(sto_file)
            obj.sto_file = sto_file;
            
            %Read ForceReporter.sto file
            %---------------------------
            if (exist(sto_file,'file') ~=2)
                fprintf('File: %s',sto_file);
                error('File does not exist!')                
            end
            
            [data,labels,header] = read_opensim_mot(sto_file);
            obj.raw.data = data;
            obj.raw.labels = labels;
            obj.raw.header = header;
            obj.raw.nLabels = length(labels);
            obj.raw.component_names = cell(obj.raw.nLabels,1);
            obj.raw.param_names = cell(obj.raw.nLabels,1);
            
            for i = 1:obj.raw.nLabels
                [ss, match] = strsplit(labels{i},'.');
                if(length(match)>0)
                    obj.raw.component_names{i} = ss{1};
                    obj.raw.param_names{i} = ss{2};
                end
            end
            
            % Time and Frame
            %---------------
            timeI = strcmpi('time',labels);
            obj.time = data(:,timeI);
            obj.frame_number = 1:length(obj.time);
            obj.nFrames = length(obj.time);

            
        end
        function plot_ligament_bundles(obj,bundle_names,parameter)
        %==================================================================
        %bundle_names : cell of str
        %   'all' or basename of ligaments ('MCL' or 'MCLd' or 'MCLd1' etc)
        %parameter : str
        % 'force_spring','force_damping','force_total','length','lengthening_speed'
        %==================================================================
        nBundle = length(bundle_names);
        
        for i = 1:nBundle
            if (strcmp(bundle_names{1}, 'all'))
                bundle_ind = true(obj.raw.nLabels,1);
            else
                bundle_ind = strncmpi(bundle_names{i},obj.raw.component_names ,length(bundle_names{i}));
            end
                param_ind = strcmpi(parameter,obj.raw.param_names);
            ind = and(bundle_ind,param_ind);
            
            data(i).sto = obj.raw.data(:,ind);
            data(i).names = obj.raw.component_names(ind);
        end
        
        figure('name',[bundle_names{:} ' : ' parameter])
        hold on;
        for i = 1:nBundle
            [r, c] = size(data(i).sto);
            for j = 1:c
                plot(obj.time,data(i).sto(:,j))
            end
        end
        
        
        end
    end
end
    