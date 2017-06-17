%% WISCO_Motion class 
%==========================================================================
%Author: Colin Smith
%--------------------------------------------------------------------------
%
%
%
%==========================================================================

classdef WISCO_motion
    properties
        mot_file;
        time;
        raw;
    end
    
    methods
        function obj = WISCO_motion(mot_file)
            %==============================================================
            %CLASS CONSTRUCTER
            %--------------------------------------------------------------
            %mot_file : full path to .mot file to read and analyze 
            %(include .mot extension)
            %==============================================================
            
            %Read .mot file
            %--------------
            if (exist(mot_file,'file') ~=2)
                fprintf('Motion File: %s',mot_file);
                error('Motion file does not exist!')                
            end
            
            [data,labels,header] = read_opensim_mot(mot_file);
            obj.raw.data = data;
            obj.raw.labels = labels;
            
            % Time and Frame
            %---------------
            timeI = strcmpi('time',labels);
            obj.time = data(:,timeI);
            
            
        end
    end
end