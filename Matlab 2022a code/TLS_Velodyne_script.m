%%   Dense  point  cloud  acquisition  with  a  low-cost  Velodyne  Puck
%    Author : Jason Bula
%    Modified to work with the VLP-32C Ultra Puck and as a stand alone App with point cloud visualization by Donny Mott

%% Directory Management

load('C:\TLS_Velodyne\application\settings.mat')

%browse to file
input_file_name = input_file_name;
disp(input_file_name)

file = input_file_name;
[filepath,~,~] = fileparts(file);
input = filepath;
disp(input)


% Export folder
resultsDir = fullfile(input, 'results');
disp(resultsDir)
try
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end
catch ME
    disp(['Error creating directory: ', ME.message])
end

output =resultsDir;
file = input_file_name;
[~,output_file_name,ext] = fileparts(file);
disp(output_file_name)


%% Initialization of parameters 

times = times; % Scan duration in seconds 
angle = angle; % Rotation in degrees around motor axis. Ouster TLS X+ is up, Velodyne is Y+ up

if isempty(angle) || ~isnumeric(angle) || angle < 45 || angle > 360
    disp('Input for angle is invalid or outside the valid range (45-360). Setting angle to 360.');
    angle = 360;
end

first = first; % Time at the first frame (delay)
puck = puck; %version of lidar puck 'VLP16' or 'VLP32C'
% Reset the list of files to merge
filesToMerge = {};

%% Export parameters
% Set to 0 to keep all the points
% Set to 1 to keep the x-positives values (velodyne)
% Set to 2 to keep the X-negatives values (velodyne)
pos = pos; % set to 1 normally / Set to 0 for calibration of a2

pos2 = pos2; % set to 1 normally / Set to 0 to keep save First and last band/ Set to 1 to keep all bands/ Set to 2 to keep middle band

gridStep = gridStep; %Set the grid resolution in [m] / Set to 0 to keep all the values (subsampling)

%% Calibration parameters
% Correction angle alpha_1 and alpha_2 [degrees]
alpha_1 = alpha_1; %incorrect value cause blur
alpha_2 = alpha_2; %Incorrect value causes domeing
R =     R; % Arm length [m] distance rotation center of Lidar off from Motor axis
theta3 = 0; %adjust for irregularities and discrepancies in the speed of the motor

%% Point cloud correction to be applied during rotation
% 
%  In this part of the code, the scan will be extracted frame by frame 
%  in order to to apply the necessary correction to realign the point cloud.
%  First of all, only images containing positive X will be kept, then each
%  image will be processed separately.
%  Two transformation matrices will be applied to each image. One
%  containing rotation according to the LiDAR angle (varies over time) and
%  the other, an applied translation corresponding to the distance between 
%  the arm and the optical center of the LiDAR. This distance was measured
%  manually and corresponds to a displacement over z of R = 0.0945m (can
%  be improved). Finally, each image is saved separately in a Cell.
% 

% Initialisation
if puck == 1

veloReader = velodyneFileReader(input_file_name,'VLP16');

else

 veloReader = velodyneFileReader(input_file_name,"VLP32C");
end

%count frames of scan
totalScanFrames = veloReader.NumberOfFrames;
fprintf('Total Number of Scan Frames (totalScanFrames): %d\n', totalScanFrames);

usableFrames = totalScanFrames - first;  % frames-start delay
fprintf('Total Number of Scan Frames minus Delay (usableFrames): %d\n', usableFrames);

% Check if 'times' exceeds the usable times
if times > usableFrames
    warning('Entered time value is too large. It has been replaced with the maximum allowed Time of %d.', usableFrames);
    times = usableFrames;  % replace times with maximum allowed value
    
end

fprintf('Input Value in Time (times): %d\n', times);

settings_file = ('C:\TLS_Velodyne\application\settings.mat');
save(settings_file, 'times', 'angle', 'first', 'gridStep', 'alpha_1', 'alpha_2', 'R', 'pos', 'pos2', 'input_file_name', 'puck', 'usableFrames', 'totalScanFrames')

last = first +times; % Time at the last frame
angle_deg=(0:angle/times:angle); % Angle after each frame
angle = deg2rad(angle_deg); % Angle in radian
s = 0; 


%% Initialize Cloud
for i = 2 : length(angle) % Runs as many times as there are frames
     NF = first + s;
     ptCloudIn = readFrame(veloReader,NF); % Selecting frames separately

% Apply export parameters 
% pos 1 filters (velodyne) the point cloud data to only include points that are
% located in the positive X half-space (X > 0). Any point cloud data with
% X-coordinate values less than or equal to zero are removed.

if pos == 1   
    
    ptCloudIn3 =  ptCloudIn.Location(:,:,1); % extract the x-coordinate values from the point cloud (velodyne)
    ptCloudIn3(isnan(ptCloudIn3))=0; % replacing all the NaN values in the x-coordinates with zeros
    
    ptCloudIntensity = ptCloudIn.Intensity(:,:); % intensity values of the point cloud are being extracted
    ptCloudIntensity(isnan(ptCloudIntensity))=0; % any NaN values are being replaced with zeros

    ptCloudIn3(ptCloudIn3<0)=0; % intensity values are being multiplied by the binary mask
    ptCloudIn3(ptCloudIn3>0)=1; % zeroes out the intensity values for all points with an x-coordinate less than or equal to zero
    
    ptCloudIntensity2 = single(ptCloudIntensity(:,:)).*ptCloudIn3; % intensity values are being multiplied by the binary mask
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3,'Intensity',ptCloudIntensity2);

    % the pos variable determines which half-space of the relevant axis
    % (Velodyne) X-axis or (Ouster) Y-axis is preserved in the output point cloud. This
	% filtering process is accomplished by creating a binary mask based on the
	% sign of the coordinate values and applying this mask to the point cloud
	% coordinates and intensity values. The output point cloud only includes
	% points from the specified half-space and the rest of the points are
	% filtered out.
end

% pos 2 (velodyne) filters the point cloud data to only include points that are
% located in the negative X half-space (X < 0). Any point cloud data with
% X-coordinate values greater than or equal to zero are removed.

if pos == 2   
    
    ptCloudIn3 =  ptCloudIn.Location(:,:,1);
    ptCloudIn3(isnan(ptCloudIn3))=0; % replacing all the NaN values in the y-coordinates with zeros
    
    ptCloudIntensity = ptCloudIn.Intensity(:,:);
    ptCloudIntensity(isnan(ptCloudIntensity))=0;
    
    ptCloudIn3(ptCloudIn3>0)=0;
    ptCloudIn3(ptCloudIn3<0)=1;
    
    ptCloudIntensity2 = single(ptCloudIntensity(:,:)).*ptCloudIn3;
    ptCloudIn = pointCloud(ptCloudIn.Location(:,:,:).*ptCloudIn3,'Intensity',ptCloudIntensity2);
end
  
clear ptCloudIn2 ptCloudIn3 

%% Transformation of each band
% Transformation matrix

% Alignment correction as a function of motor speed (y-axis) (velodyne)
VM = [cos(angle(i)) 0 sin(angle(i)) 0; 0 1 0 0; -sin( angle(i)) 0 ...
      cos(angle(i)) 0; 0 0 0 1];

  % Alpha_1 correction (Y-axis)
A1 = [1 0 0 0; 0 cosd(alpha_1) sind(alpha_1) 0 ; ...
      0 -sind(alpha_1) cosd(alpha_1) 0 ; 0 0 0 1];  
  
% Alpha_2 correction (z-axis)
A2 =  [cosd(alpha_2) sind(alpha_2) 0 0; ...
     -sind(alpha_2) cosd(alpha_2) 0 0; 0 0 1 0; 0 0 0 1];
  
% R correction to translate along the z-axis (velodyne and ouster)
T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 R 1];

% Speed adjustement
T4 = [cosd(theta3) sind(theta3) 0 0; ...
     -sind(theta3) cosd(theta3) 0 0; 0 0 1 0; 0 0 0 1];
 
% Apply transformation
tform_T = affine3d(T);
ptCloudIn = pctransform(ptCloudIn,tform_T);

tform_A1 = affine3d(A1);
ptCloudIn = pctransform(ptCloudIn,tform_A1);

tform_A2 = affine3d(A2);
ptCloudIn = pctransform(ptCloudIn,tform_A2);

tform_T = affine3d(T4);
curr_img = pctransform(ptCloudIn,tform_T); % configuration 2

tform_R = affine3d(VM);
Cloud{i-1} = pctransform(curr_img,tform_R); % Save in a cell

s = s + 1; % count update

end


%% Initialisation of export parameters
if pos2 == 1 && puck == 1
    bandNumber = 16; %selects all VLP16 segments
end

if pos2 == 1 && puck == 2
    bandNumber = 32; %selects all VLP32C segments
end
if pos2 == 0
    bandNumber = 2; %selects first and last segment
end
if pos2 == 2
    bandNumber = 1; %selects middle segment
end

%% Point cloud merging 

for bande_sep = 1 : bandNumber
    
if pos2 == 1 
    iiii=bande_sep;
end

if pos2 == 0  && puck == 1  
    Bande_calibration = [1 16];
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 0  && puck == 2  
    Bande_calibration = [1 32];
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 2  && puck == 1  
    Bande_calibration = 8;
    iiii = Bande_calibration(bande_sep);
end

if pos2 == 2  && puck == 2  
    Bande_calibration = 16;
    iiii = Bande_calibration(bande_sep);
end

%% Variable initialization
x = [];
y = [];
z = [];
int = [];

X_ref = [];
Y_ref = [];
Z_ref = [];
int_ref = [];

X_ref_final = [];
Y_ref_final = [];
Z_ref_final = [];
int_ref_final = [];

% Acceleration of the process by combining several loops
for iii = 1 : 10 
    for ii = round((times/10*iii)-((times/10)-1)) : round(iii*times/10)    
   

     for i = iiii:iiii % save the band separately

% Deleting old values
        x1 = [];
        y1 = [];
        z1 = [];
        int1 = [];
% Selection of points in the correct matrix locations
% Point clouds are recorded as follows: 16*1800*3
% 16 corresponds to the band, 1800 corresponds to the number of points recorded
% per band, 3 corresponds to the x, y and z values.

x1(i,:) = Cloud{1,ii}.Location(i,:,1);
y1(i,:) = Cloud{1,ii}.Location(i,:,2);
z1(i,:) = Cloud{1,ii}.Location(i,:,3);
int1(i,:) = Cloud{1,ii}.Intensity(i,:);


x = [x x1(i,:)];
y = [y y1(i,:)];
z = [z z1(i,:)];
int = [int int1(i,:)];


    end 
    X_ref = [X_ref x];
    Y_ref = [Y_ref y];
    Z_ref = [Z_ref z];
    int_ref = [int_ref int];
    
    x = 0;
    y = 0;
    z = 0;
    int = 0;
    
    end
    X_ref_final = [X_ref_final X_ref];
    Y_ref_final = [Y_ref_final Y_ref];
    Z_ref_final = [Z_ref_final Z_ref];
    int_ref_final = [int_ref_final int_ref];
    
% disp(iii) % extraction progress meter 
    X_ref = 0;
    Y_ref = 0;
    Z_ref = 0;
    int_ref = 0;
end
        

%% Reconstruction of the point cloud

% Reset the list of files to merge
%filesToMerge = {};

ref = [X_ref_final; Y_ref_final; Z_ref_final]'; 

PC_corr1 = pointCloud(ref,'Intensity',int_ref_final');

if gridStep == 0
    PC_downsampled_1 = PC_corr1;
else
    PC_downsampled_1 = pcdownsample(PC_corr1,'gridAverage',gridStep);
end

% Remove invalid valuesbande_sep
[PC_Final1,indices]= removeInvalidPoints(PC_downsampled_1);

%Create the rotation matrix (velodyne)
RotX = [1 0 0 0; 0 cosd(-90) -sind(-90) 0; 0 sind(-90) cosd(-90) 0; 0 0 0 1];

% Apply the rotation to point cloud to align Z axis upwards (velodyne)
PC_Final1 = pctransform(PC_Final1, affine3d(RotX));

%% Point cloud export with Seperate Filesand Merged Files (velodyne)
% In this part of the code, the scatterplot is exported. 

% Defining output document names
%filename = sprintf([output_file_name,'_', num2str(iiii)]);
filename = sprintf('%s_%d.ply', output_file_name, iiii);

% Change directory to output
cd(output)

% Write file
pcloud{1,ii} = PC_Final1;
pcwrite(PC_Final1,filename,'PLYFormat','binary');

% Store filenames for later merging
filesToMerge{bande_sep} = fullfile(output, filename);
disp(['File stored for merging: ', filesToMerge{bande_sep}]);

% Return to input directory
cd(input)

ref = []; % suppression of the loaded point cloud
if puck == 1
f = msgbox((["Processed File:";output_file_name,'_', num2str(iiii) ' of 16\n']),"Status");
else
f = msgbox((["Processed File:";output_file_name,'_', num2str(iiii) ' of 32\n']),"Status");
end
pause(1)
if isvalid(f); delete(f); end
end


%% Setup for Merge of Files
% Define merge tolerance
%mergeTolerance = gridStep unless gridStep is 0 then subsample at 0.005
%meters. (for pcmerge gridstep has to be a positive value)

if gridStep ~= 0
    mergeTolerance = gridStep; 
else
    mergeTolerance = 0.005;  
end
 
% Check if there are multiple files to merge Add
if length(filesToMerge) > 1

%disp('Files to merge:');
%disp(filesToMerge);

% Read the first file and use it to initialize the merged point cloud
tempPC = pcread(filesToMerge{1});
mergedPointCloud = tempPC;

% Read each subsequent file back in and merge
for i = 1:length(filesToMerge) % was 2 now 1
    %disp(filesToMerge{i}); % Add this line to print each file name
    if exist(filesToMerge{i}, 'file')
        tempPC = pcread(filesToMerge{i});
        mergedPointCloud = pcmerge(mergedPointCloud, tempPC, mergeTolerance);
    else
        disp(['File does not exist: ', filesToMerge{i}]);
    end
end


% Define the output filename
merged_filename = [output_file_name,'_merged.ply'];

% Save the merged point cloud
pcwrite(mergedPointCloud, fullfile(output, merged_filename), 'PLYFormat', 'binary');

% Display the merged point cloud
disp(['Merged File Saved: ', merged_filename]);


% Set the maximum limit of points for visualization
    maxNumPoints = 5e5; % 100,000 points would be 1e5, 500,000 would be 5e5 and 1,000,000 is to many points.
    if mergedPointCloud.Count > maxNumPoints
        % Downsample the point cloud if it contains more than the maximum limit
        mergedPointCloud = pcdownsample(mergedPointCloud, 'random', maxNumPoints / mergedPointCloud.Count);
    end

    % Normalize the Z values to the range [0, 1]
    z = mergedPointCloud.Location(:, 3); % Z values
    z = (z - min(z)) / (max(z) - min(z)); % normalization to [0, 1]

    % Apply the colormap
    cmap = jet(256); % colormap
    c = round(z * (size(cmap, 1) - 1)) + 1; % Match color indices to z values
    color = uint8(cmap(c, :) * 255); % Convert to 8-bit RGB color

    % Create a new point cloud with color
    mergedPointCloudColor = pointCloud(mergedPointCloud.Location, 'Color', color);

    % Apply the colormap
    pcshow(mergedPointCloudColor);
    title('Merged Point Cloud');
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
else
    % Specify action for the case of one file
    disp('Only one file, skipping merging process.')

    % Read the single point cloud file
    singlePointCloud = pcread(filesToMerge{1});

    % Normalize the Z values to the range [0, 1]
    z = singlePointCloud.Location(:, 3); % Z values
    z = (z - min(z)) / (max(z) - min(z)); % normalization to [0, 1]

    % Apply the colormap
    cmap = jet(256); % colormap
    c = round(z * (size(cmap, 1) - 1)) + 1; % Match color indices to z values
    color = uint8(cmap(c, :) * 255); % Convert to 8-bit RGB color

    % Create a new point cloud with color
    singlePointCloudColor = pointCloud(singlePointCloud.Location, 'Color', color);

    % Display the single point cloud
    pcshow(singlePointCloudColor);
    title('Single Point Cloud');
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
end
% Return to input directory
cd(input)
