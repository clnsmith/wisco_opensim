%% WISCO_Kinematics_Analysis class 
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%
%==========================================================================
classdef WISCO_Coordinate_Analysis
    properties
        file
        raw
        
        time
        frame_number
        nFrames
        names
        data
        nCoords
    end
    methods
        function obj = WISCO_Coordinate_Analysis(file)
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
                
                info = h5info(file,'/Coordinates');
                
                obj.nCoords = length(info.Groups);
                
                obj.names = cell(obj.nCoords,1);
                
                for i = 1:obj.nCoords
                    label = strsplit(info.Groups(i).Name,'/');
                    obj.names{i} = label{3};
                    
                    nDataSet = length(info.Groups(i).Datasets);
                    
                    for j = 1:nDataSet 
                        param = info.Groups(i).Datasets(j).Name;
                        data_set_name = ['/Coordinates/' obj.names{i} '/' param];
                        obj.data.(obj.names{i}).(param) = h5read(file,data_set_name);
                    end
                end
                
            elseif(strcmp('.sto',ext))
                
            end
        end
    end
end