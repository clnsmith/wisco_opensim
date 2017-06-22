%WISCO_h5_Analysis
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%
%==========================================================================
classdef WISCO_h5_Analysis
    properties
        h5_file
        lig
        cnt
        coord
        msl
        %states
    end
    methods
        function obj = WISCO_h5_Analysis(h5_file)
            if (exist(h5_file,'file') ~=2)
                fprintf('File: %s',file);
                error('File does not exist!')                
            end
            obj.h5_file = h5_file;
            
            h5_info = h5info(h5_file);
            h5_grp = {h5_info.Groups.Name};
            n_h5_grp = length(h5_grp);
            
            for i = 1:n_h5_grp
                if strcmp(h5_grp{i},'/WISCO_Ligament')
                    obj.lig = WISCO_Ligament_Analysis(h5_file);
                elseif strcmp(h5_grp{i},'/WISCO_ElasticFoundationForce')
                    obj.cnt = WISCO_ElasticFoundationForce_Analysis(h5_file);
                elseif strcmp(h5_grp{i},'/Coordinates')
                    obj.coord = WISCO_Coordinate_Analysis(h5_file);
                elseif strcmp(h5_grp{i},'/Muscles')
                    obj.msl = WISCO_Muscle_Analysis(h5_file);
                elseif strcmp(h5_grp{i},'/States')
                    %obj.states = WISCO_States_Analysis(h5_file);
                end
            end
        end
    end   
end