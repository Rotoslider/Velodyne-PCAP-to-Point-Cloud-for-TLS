# VelodynePCAPtoPointCloudConverter

This  study  develops  a  method  to  acquire  densepoint  clouds  with  a  low-cost  Velodyne  VLP-16 or VLP32C  lidar  Puck, without using expensive GNSS positioning or IMU. Our setting consists  in  mounting  the  lidar  on  a  motor  to  continuously change the scan direction, which leads to a significant increase in  the point  cloud  density.  A  post-treatment  reconstructs  the position of each point accounting for the motor angle at the timeof acquisition, and a calibration step accounts for inaccuracies in  the  hardware  assemblage.  The  system  is  tested  in  indoors settings  such  as  buildings  and  abandoned  mines,  but  is  also expected  to  give  good  results  outdoors. 



Instructions to run the codes



Setup lidar unit on mount so the cable is pointed down.
Measure from center of axis motor to center of lidar window to get R value. (example: 0.2095) value is in meters
Idealy it should be 0 since Center of Lidar should be directly inline with motor axis.

So there are several points that could improve your system calibration:

1) Ideally, the calibration should be done in a large indoor area in a more structured environment (wall/floor/angle etc.), and if you don't disassemble the system for the next measurements, the parameters should be the same.

2) The system has to run clockwise.

3) The Matlab calibration_demo file is to be used twice (once for alpha 1 and once for alpha 2). I calibrated alpha2 first (line 9 : calibration = 2 ;) in calibration_TLS. You must also set alpha1 = 0 in the alpha2_calibration file. (have not had to good of luck with this) You can do manual calibration by trial and error if you do not have Matlab

Once the calculation is done, the value of alpha2 is saved in x. Enter this value in TLS_main for alpha2

Then you have to calculate alpha1 (line 9 : calibration = 1 ;) in calibration_demo. This time you have to set alpha2= x (your alpha2 result) in the alpha1_calibration file. And restart calibration_demo

4) Set grid value in TLS_main to 0.005 ( = 0.197") to reduce point cloud size output when doing 2 deg per second. otherwise set to 0

5) I advise you to leave the process speed at mult = 2 (not an option in the app)

6) You can also adjust the Calibration Range value (line 45) which will allow you to set the distance on which the calibration is based (for example "Range = [10 20]" if you are indoors) (4 30 default outdoor) (not avalible in the app)

Run the app or the from where you installed it or run the TLS_PCAP_Converter.mlapp file in Matlab
After running there will be 16 or 32 .ply files if you slected all. . Open Cloudcompare and drag these files in
Select all the files and click edit then merge
Click yes to the scaler field question
Select the merged cloud and sthen click edit subsample. I use 0.002 (0.0787") for this
Select the new subsampled cloud press edit and Apply transformation. Select Axis,Angle tab  X1.0 Y0.0 Z0.0 Rotation 90deg click ok
Select subbed rotated cloud and Save in E57 format

Calibration Notes:
The Y axis of the scan runs parallel with the axis of the motor. Which means Y+ is down and Y- is up Z and X run horizontal to the ground.
A1 and A2 values are in Degrees
R value is in meters

A1 causes blur
A2 causes a doming effect. If you set line 39 of velodyne_tls_intensity to 0 you keep all points. Now you can see how well they line up. If A2 is off the bottom of the scan will be off like this  >< . It should look like this --
Change A2 to bring front and rear scans together.
The farther A2 is out the larger the R offset. 
With original frame and csf-14-50 hamonic drive a1=0 a2=-1.1 R=0 t=3630 keep -y values
If keeping the -Y values you need to apply a transform in Cloud Comapre to rotate it so Z is up.
X1,Y0,Z0 90degrees
