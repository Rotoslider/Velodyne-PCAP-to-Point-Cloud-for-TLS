# VelodynePCAPtoPointCloudConverter

This uses a method to acquire dense point clouds with a low-cost Velodyne VLP-16 or VLP32C lidar Puck, without using expensive GNSS positioning or IMU. This consists in  mounting  the  lidar  on  a  motor  to  continuously change the scan direction, which leads to a significant increase in  the point  cloud  density.  A  post-treatment  reconstructs  the position of each point accounting for the motor angle at the time of acquisition. By adjusting the Calibration Settings you can adjust for misalignment in the motor to lidar axis.  The system is tested in indoors settings such as buildings and abandoned mines and caves. it gives good results outdoors enviorments also. 


Instructions to run the codes

Setup lidar unit on mount so the cable is pointed down.
Measure from center of axis motor to center of lidar window to get R value. (example: 0.2095) value is in meters
Idealy it should be 0 since Center of Lidar should be directly inline with motor axis.

So there are several points that could improve your system calibration:

1) Ideally, the calibration should be done in a large indoor area in a more structured environment (wall/floor/angle etc.), and if you don't disassemble the system for the next measurements, the parameters should be the same.

2) The system has to run clockwise.

3) Set Grid step to 0.005 ( = 0.197") to reduce point cloud size output when doing 2 deg per second. otherwise set to 0 = no subsampling


Run the app or the from where you installed it or run the TLS_PCAP_Converter.mlapp file in Matlab

After running there will be series of .ply files. One file if you selected Middle or two if First and Last, 16 if you are using a VLP16 and selected All. 32 if your using a VLP32
There will also be a merged file generated. It will be subsampled at 0.005m or whatever value besides 0 you set in GridStep

Calibration Notes:
The Y axis of the scan runs parallel with the axis of the motor. Which means Y+ is down and Y- is up Z and X run horizontal to the ground.
A1 and A2 values are in Degrees
R value is in meters

alpha1 misalignment  causes blur
alpha2 misalignment causes a doming effect. If you set Points to Keep to all you can see how well they line up. If alpha2 is off, the bottom of the scan will be off like this  >< . It should look like this --
Change alpha2 to bring front and rear scans together.
The farther alpha2 is out the larger the R offset. 


