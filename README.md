# SDE-Project_5-Extended-Kalman-Filter
C++, EKF, Lidar, Radar 

## Overview
In this project, we are going to implement the extended Kalman Filter in C++. The data from both LIDAR and RADAR measurements for object are given. 
![](https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter/blob/master/data/both_lidar_radar.gif)

## Dependencies

Please meet the minimum project dependency versions:

* cmake:3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

`CMakeLists.txt`: file that will be used when compiling your code

### Basic steps:
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
5. Test your code using the [simulator](https://github.com/udacity/self-driving-car-sim/releases)  

Note: This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.


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
	
* The Term 2 simulator is a client, and the C++ program is a web server.
* main function is made up of several functions within `main()`, these all handle the uWebsocketIO communication between the simulator and it's self.  
* Give the estimation position and call calculated RMSE method


`FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The kalman filter equations are not in this file. It has a variable called `ekf_`, which is an instance of a `kalmanFilter` class. The `ekf_` will hold the matrix and vector values. We will use the `ekf_` instance to call the predict and update equations

```c++
// TODO: Finish initialiizing the FusionEKF
// set the process and measurement noises

H_laser_ << 1,0,0,0,
			0,1,0,0;

// initialize the kalman filter variables
ekf_.P_ = MatrixXd(4,4);
ekf_.P_ << 1,0,1,0,
			0,1,0,1,
			0,0,1000,0,
			0,0,0,100;
ekf_.F_ = MatrixXd(4,4);
ekf_.F_ << 1,0,1,0,
		   0,1,0,1,
		   0,0,1,0
		   0,0,0,1,
		   0,0,0,1;
// set measurement noises
noise_ax = 9;
noise_ay = 9;
```
```c++
if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      // and initialize state.
	float rho = measurement_pack.raw_measurements_[0];
	float phi = measurement_pack.raw_measurements_[1];
	float rho_dot = measurement_pack.raw_measurements_[2];

	ekf_.x_ << rho * cos(phi), rho * sin(phi),rho_dot * cos(phi), rho_dot * sin(phi)

    }
```
```c++
else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      // TODO: Initialize state.
      
      
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
```

```c++

/*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

```
`KalamnFilter` class is define in `kalman_filter.cpp` and `kalman_filter.h`. 

```c++
// kalman_filter.h

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
};
```

kalman_filter.cpp

```c++
void KalmanFilter::Predict(){
	// TODO: predict the state

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = (F_ * P_ * Ft) + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
	VectorXd z_pred = H_ * x_;
  	VectorXd y = z - z_pred;
  	MatrixXd Ht = H_.transpose();
  	MatrixXd S = H_ * P_ * Ht + R_;
  	MatrixXd Si = S.inverse();
  	MatrixXd PHt = P_ * Ht;
  	MatrixXd K = PHt * Si;

  	//new estimate
  	x_ = x_ + (K * y);
  	long x_size = x_.size();
  	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
	float px = x_[0];
  	float py = x_[1];
  	float vx = x_[2];
  	float vy = x_[3];

  	float rho = sqrt(pow(px,2) + pow(py,2));
  	float phi;
  	float rho_dot;
  
  	if (rho > 0.001) {
    	phi = atan(py/px);
    	rho_dot = (px*vx + py*vy)/rho;
  } else {
    phi = 0;
    rho_dot = 0;
  }

  	VectorXd z_pred(3);
  	z_pred << rho, phi, rho_dot;

  	VectorXd y = z - z_pred;
  	MatrixXd Ht = H_.transpose();
  	MatrixXd S = H_ * P_ * Ht + R_;
  	MatrixXd Si = S.inverse();
  	MatrixXd PHt = P_ * Ht;
  	MatrixXd K = PHt * Si;

  	//new estimate
  	x_ = x_ + (K * y);
  	long x_size = x_.size();
  	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  	P_ = (I - K * H_) * P_;


}

```

`tools.cpp` fill in the function that calcuate root mean squared error(RMSE) and the Jacobian matrix. 

## Conclusion

Compiling is the process of translating the code that you've written into machine code that processors understand. Every program, regardless of the source language, needs to be compiled in order to execute. This is true even for scripting languages like Python or JavaScript. In these cases, the interpreter (or a similar system) is responsible for compiling code on the fly in a process known as just-in-time compiling. To the user, compiling and execution are effectively a single action. (Of course, the actual process of compiling code at run-time is much more complicated than what was described here in one sentence. It's also very much dependent on the exact language and runtime in question.)

Unlike scripted languages, compiled languages treat compilation and execution as two distinct steps. Compiling a program leaves you with an executable (often called a "binary"), a non-human readable file with machine code that the processor can run.

The nice thing about binaries is that they're generally distributable. So long as it was built with the right architecture in mind, you can copy an executable and run it immediately on other machines (like downloading a .exe file on Windows) without any need to share your source code or have the user perform any intermediate tasks before execution.

The problem with compiling is that it can be a massive pain, to put it lightly. I've argued that half the battle of learning a language like C++ is just getting your code to compile for the first time. There are many tools available to help you compile, ranging from barebones tools, such as g++ on Unix, to complex build systems that are integrated into IDEs like Visual Studio and Eclipse.

For this project, we decided to use a high-level build tool called CMake for the fact that it's fairly popular and cross-platform. CMake in and of itself, however, does not compile code. CMake results in compilation configurations. It depends on a lower-level build tool called Make to manage compiling from source. And then Make depends on a compiler to do the actual compiling.

Technically, you only need a compiler to compile C++ source code to a binary. A compiler does the dirty work of writing machine code for a given processor architecture. There are many compilers available. For this project, we picked the open source GNU Compiler Collection, more commonly called G++ or GCC. gcc is a command line tool.

There are two challenges with using gcc alone, both of which relate to the fact that most C++ projects are large. For one thing, you need to pass the paths for all of the project's source header files and cpp files to gcc. This is in addition to any compiler flags or options. You can easily end up with single call to gcc that spans multiple lines on a terminal, which is unruly and error-prone.

Secondly, large projects will usually contain multiple linked binaries, each of which is compiled individually. If you're working in large project and only change one .cpp file, you generally only need to recompile that one binary - the rest of your project does not need to be compiled again! Compiling an entire project can take up to hours for large projects, and as such being intelligent about only compiling binaries that need to be compiled can save lots of time. GCC in and of itself is not smart enough to recognize what files in your project have changed and which haven't, and as such will recompile binaries needlessly - you'd need to manually change your gcc calls for the same optimizations. Luckily, there are tools that solve both of these problems!