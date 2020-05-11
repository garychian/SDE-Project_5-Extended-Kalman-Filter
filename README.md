# SDE-Project_5-Extended-Kalman-Filter
C++, EKF, Lidar, Radar 

## Overview
In this project, we are going to implement the extended Kalman Filter in C++. The data from both LIDAR and RADAR measurements for object are given. 

### Data file
The data file screenshot is shown down below. Each row represents a sensor measurement where first column tells us the measurement coms from radar(R) and Lidar(L). The following columns are **sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groudtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth**

![](https://github.com/garychian/SDE-Project_5-Extended-Kalman-Filter/blob/master/Image/data%20file.jpg)

### File Structure
`main.cpp`: communicates with the Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RSME

`FusionEKF.cpp`: initializers the filter, calls the predit function, calls the update function

`kalman_filter.cpp`: define the predict function, the update function for lidar, and the update function for radar

`tools.cpp`: function to calculate RMSE and the Jacobian matrix

####Relation between each files
`main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`

`FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The kalman filter equations are not in this file. It has a variable called `ekf_`, which is an instance of a `kalmanFilter` class. The `ekf_` will hold the matrix and vector values. We will use the `ekf_` instance to call the predict and update equations

`KalamnFilter` class is define in `kalman_filter.cpp` and `kalman_filter.h`. 
