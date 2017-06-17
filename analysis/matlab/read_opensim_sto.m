function [data, labels, header] = read_opensim_mot(file)
%%=========================================================================
%READ_OPENSIM_MOT
%--------------------------------------------------------------------------
%Author: Colin Smith
%Date: 10/24/2016
%--------------------------------------------------------------------------
%file : str
%
%
%data : [nFrames x nLabels] matrix
%
%
%labels : {1 x nLabels} cell of strings 
%
%
%header : struct
%
%
%==========================================================================

if ~exist('file','var')
    [infile, inpath]=uigetfile('*.sto','Select input file');
    file=[inpath infile];
end
        
disp(file);
fid=fopen(file,'r');
nLine = 0;

if fid <0
    mot=[];labels=[];
    disp('File Not Found:\n');
    disp([file '\n']);
    return
end

disp(['Loading file...' file] );

% Read Header
line = fgetl(fid);
nLine = nLine+1;

while ~strcmpi(line,'endheader')

    
    if (line == -1)
        disp('ERROR: Reached EOF before "endheader"')
        return
    end
    
    split_line = strsplit(line,'=');
    
    if (length(split_line)==2)
        var = split_line{1};
        value = split_line{2};

        if strcmpi(var,'version')
            header.version = str2double(value);
        elseif strcmpi(var,'nRows')
            nr = str2double(value);
            header.nRows = nr;
        elseif strcmpi(var,'nColumns')
            nc = str2double(value);
            header.nColumns = nc;
        elseif strcmpi(var,'indegrees')
            header.indegrees = value;
        elseif strcmpi(var,'DataType')
            header.dataType = value;
        end
    end
    
    line = fgetl(fid);
    nLine = nLine+1;
end


%Load Column Headers
line=fgetl(fid);
nLine = nLine+1;

labels = strsplit(line,'\t');
nLabel = length(labels);

%% Read Data
if(exist('header.dataType','var') == 1)
    if (strcmp(header.dataType,'Vec3'))
        nCol = 1+(nLabel-1)*3;
    end
else
    nCol = nLabel;
end

raw=textscan(fid,'%f','Delimiter',{'\t',',','(ind)'},'MultipleDelimsAsOne',1);%,'HeaderLines',nLine);
data = reshape(raw{1,1},nCol,[])';
 
fclose(fid);



if (nargout>1)
    % return all data in a single array
    mot=data;
elseif (nargout==1)
    % return all information in a single structure
    mot.data=data;
    mot.hdr=labels;
end


function [t,q]=load_exp(file);
global NQ NM;
rawdata=load_motionfile(file);
[t,data]=extractcolumns(rawdata,1);
[q,data]=extractcolumns(data,NQ);


function [x,outdata]=extractcolumns(data,nc);
x=data(:,1:nc);
[m,n]=size(data);
outdata=data(:,(nc+1):n);