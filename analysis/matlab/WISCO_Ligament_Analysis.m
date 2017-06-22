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
        file
        raw
        
        time
        frame_number
        nFrames
        nLigaments
        names
        data
    end
    methods
        function obj = WISCO_Ligament_Analysis(file)
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
                
                info = h5info(file,'/WISCO_Ligament');
                
                obj.nLigaments = length(info.Groups);
                
                obj.names = cell(obj.nLigaments,1);
                
                for i = 1:obj.nLigaments
                    label = strsplit(info.Groups(i).Name,'/');
                    obj.names{i} = label{3};
                    
                    nDataSet = length(info.Groups(i).Datasets);
                    
                    for j = 1:nDataSet 
                        param = info.Groups(i).Datasets(j).Name;
                        data_set_name = ['/WISCO_Ligament/' obj.names{i} '/' param];
                        obj.data.(obj.names{i}).(param) = h5read(file,data_set_name);
                    end
                end
                
            elseif(strcmp('.sto',ext))
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
        end
        function plot_ligament_bundles(obj,bundle_names,param)
        %==================================================================
        %bundle_names : cell of str
        %   'all' or basename of ligaments ('MCL' or 'MCLd' or 'MCLd1' etc)
        %param : str
        % 'force_spring','force_damping','force_total','length','lengthening_speed'
        %==================================================================
        figure('name',[bundle_names{:} ' : ' param])
        
        if (strcmp(bundle_names{1}, 'all'))
            bundle_names = obj.names;
        end
        nBundle = length(bundle_names);

    %         for i = 1:nBundle
    %             if (strcmp(bundle_names{1}, 'all'))
    %                 bundle_ind = true(obj.raw.nLabels,1);
    %             else
    %                 bundle_ind = strncmpi(bundle_names{i},obj.raw.component_names ,length(bundle_names{i}));
    %             end
    %                 param_ind = strcmpi(parameter,obj.raw.param_names);
    %             ind = and(bundle_ind,param_ind);
    %             
    %             data(i).sto = obj.raw.data(:,ind);
    %             data(i).names = obj.raw.component_names(ind);
    %         end

            
            hold on;
            legend_names = cell(1,1);
            nLegendNames = 0;
    
            for i = 1:nBundle
                lig_ind = strncmpi(bundle_names{i},obj.names ,length(bundle_names{i}));
                lig_names = obj.names(lig_ind);
                nLig = length(lig_names);

                for j = 1:nLig
                    if(isfield(obj.data.(lig_names{j}),param))
                        %plot(obj.time,obj.data.(lig_names{j}).(param))
                        plot(obj.data.(lig_names{j}).(param),'LineWidth',2)
                    else
                        printf('Parameter: %s nor found for ligament: %s',param,lig_names{i})
                    end
                end
                
                legend_names(nLegendNames+1:nLegendNames+nLig) = lig_names;
                nLegendNames = nLegendNames+nLig;
            end
            xlabel('time [s]')
            ylabel(param)
            legend(legend_names{:})

    %         figure('name',[bundle_names{:} ' : ' parameter])
    %         hold on;
    %         for i = 1:nBundle
    %             [r, c] = size(data(i).sto);
    %             for j = 1:c
    %                 plot(obj.time,data(i).sto(:,j))
    %             end
    %         end


        end
    end
end
    