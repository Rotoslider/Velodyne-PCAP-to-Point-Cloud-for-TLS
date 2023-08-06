# Velodyne PCAP-to-Point-Cloud-for-TLS

Converts PCAP files generated using a Velodyne VLP16 or VLP32 lidar puck using MATLAB or the included stand alone MATLAB App. The lidar needs to be rotated around its Y axis with the connector pointing down. The closer the Y axis is aligned to the motor axis the better your results will be. The rotation needs to be very smooth and consistant. I have found the planetary gearsets not as smooth as what is required. A harmonic gearbox in the 30 to 50 to 1 ratio works well with either a 0.9 or 1.8 degree stepper motor. A rotation time of 2 to 4 minutes produces a dense cloud.

This is based on the work by Jason Bula and his velodyne_tls Matlab script. https://github.com/jason-bula/velodyne_tls
I have created a gui and a stand alone application for it so you do not need Matlab if you use the installer. You will need to download the Matlab 2022a Runtime which is freely avalible at Mathworks: https://www.mathworks.com/products/compiler/matlab-runtime.html
The Stand alone was written to run on a Windows 64 machine and must be installed to C:\TLS_Velodyne or it will not work. You can change this diretory in the MATLAB files and recompile if you would like to to be installed somewhere else. 

Detailed instructions on the settings and calibration of the unit are at https://github.com/jason-bula/velodyne_tls

![TLS_Velodyne_screen](https://github.com/Rotoslider/Velodyne-PCAP-to-Point-Cloud-for-TLS/assets/15005663/1d393355-ffa7-4ac3-b915-57883a173575)


The software for the scanner can be found here: https://github.com/Rotoslider/TLS_Pie
The Lidar unit should be rotated slowly and precisely along the Y axis. A rotation time of 4 minutes will produce a very dense pointcloud. The connector should be pointing down.

If the angle is below 45 degrees or above 360 it will now throw a warning and adjust it to 360.


![TLS_Velodyne_Angle_error_window](https://github.com/Rotoslider/Velodyne-PCAP-to-Point-Cloud-for-TLS/assets/15005663/d89098bf-5c6b-4eaa-ba54-728da042257d)

If the duration is set longer than the scan time in seconds, minus the delay time it will now automatically replace it with the max usable time and issue a warning.



![TLS_Velodyne_Times_error_window](https://github.com/Rotoslider/Velodyne-PCAP-to-Point-Cloud-for-TLS/assets/15005663/1a4560e5-97f6-446c-86c6-fed32e97c2dc)

At the end of proccessing a window will open to display the final point cloud. If its the middle band it will show it. If its the first and last or all bands it will show you the merged cloud. It this window you can zoom pan and rotate the cloud for inspection. This is a subsampled preview of the full cloud saved in the results folder.


![TLS_Velodyne_output_screen](https://github.com/Rotoslider/Velodyne-PCAP-to-Point-Cloud-for-TLS/assets/15005663/5f024536-08a9-4655-8c86-4e69480a7df9)




