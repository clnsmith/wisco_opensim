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
        data
    end
    methods
        function obj = WISCO_Muscle_Analysis(file)
        %==================================================================
        %CLASS CONSTRUCTER
        %------------------------------------------------------------------
        %file : full path to .h5 or .sto file to read and analyze 
        %(include extension)
        %==================================================================
            if (exist(file,'file') ~=2)
                fprintf('File: %s',file);
                error('File does not exist!')                
            end
        
            [pathstr,name,ext] = fileparts(file);
            obj.file = file;
            
            if(strcmp('.h5',ext))
                obj.time = h5read(file,'/time');
                
                info = h5info(file,'/WISCO_IdealMuscle');
                
                obj.nMuscles = length(info.Groups);
                
                obj.names = cell(obj.nMuscles,1);
                
                for i = 1:obj.nMuscles
                    label = strsplit(info.Groups(i).Name,'/');
                    obj.names{i} = label{3};
                    
                    nDataSet = length(info.Groups(i).Datasets);
                    
                    for j = 1:nDataSet 
                        param = info.Groups(i).Datasets(j).Name;
                        data_set_name = ['/WISCO_IdealMuscle/' obj.names{i} '/' param];
                        obj.data.(obj.names{i}).(param) = h5read(file,data_set_name);
                    end
                end
                
            elseif(strcmp('.sto',ext))
                            
            
            

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
        %==================================================================
        % PLOT_MUSCLE_PARAM
        %------------------------------------------------------------------
        %
        %
        %==================================================================
            figure('name',[muscle_names{:} ' : ' param_name])
            
            if strcmp(muscle_names{1},'all')
                muscle_names = obj.names;
            end
            
            nMsl = length(muscle_names);
            
            hold on;
            for i = 1:nMsl                   
                plot(obj.time,obj.data.(muscle_names{i}).(param_name),'LineWidth',2);                
            end
            xlabel('time [s]')
            ylabel(param_name)
            legend(muscle_names)
        end
    end
end