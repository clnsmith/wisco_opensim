%% WISCO_Muscle_Analysis class 
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%
%==========================================================================
classdef WISCO_Muscle_Analysis
    properties
        file
        raw
        time
        nMuscles
        names
        force
    end
    methods
        function obj = WISCO_Muscle_Analysis(file)
            [pathstr,name,ext] = fileparts(file);
        
            if(strcmp('.sto',ext))
                            
            
                obj.file = file;

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
                
                obj.names = labels(2:end);
                obj.nMuscles = length(labels)-1;
                
                obj.time = obj.raw.data(:,1);
                
                [nr nc] = size(obj.raw.data);
                
                obj.force = zeros(nr,nc-1);
                
                for i = 1:obj.raw.nLabels-1
                    obj.force(:,i) = obj.raw.data(:,i+1);
                end
            end
            
           
        end
        
        function plot_muscle_param(obj,muscle_names,param_name)
            if strcmp(muscle_names{1},'all')
                muscle_names = obj.names;
            end
            
            nMsl = length(muscle_names);
            
            fig_name = ['Muscle ' param_name ': '];
            for i = 1:nMsl
                fig_name = [fig_name muscle_names{i} ' '];
            end
            
            figure('name',fig_name)
            hold on;
            for i = 1:nMsl
                ind = strcmp(obj.names,muscle_names{i});
                frc = obj.(param_name)(:,ind);                       
                plot(obj.time,frc,'LineWidth',2);                
            end
            xlabel('time [s]')
            ylabel(param_name)
            legend(muscle_names)
        end
    end
end