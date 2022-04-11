# ncu_mdog

## About The Project
This is ncu_the drivers and sample code of ncu_mdog.
<p align="right">(<a href="#top">back to top</a>)</p>

### Hardware List

+ jetson nano 4GB
+ Ydlidar X4
+ Intel Realsence D435i
+ MPU9250
+ SG996*12
+ PCA9685


### Built With

+ jetson nano 4GB
+ Ubuntu 18.04
+ Ros melodic
+ [RTIMULIB](https://github.com/jetsonhacks/RTIMULib)
+ [ros-i2cpwmboard](https://github.com/mentor-dyun/ros-i2cpwmboard)

<p align="right">(<a href="#top">back to top</a>)</p>

## Getting Started
### Prerequisites
+ apt part
 ```sh
sudo apt updat
sudo apt-get install i2c-tools libi2c-dev build-essential python3-catkin-tools
```
+ cmake part
 ```sh
cd ~
git clone https://github.com/jetsonhacks/RTIMULib.git

cd RTIMULib/Linux
mkdir build
cd build
cmake .. -DBUILD_DEMO=0 -DBUILD_GL=0
make -j4
sudo make install
sudo ldconfig
```

### Installation

get to your workspace/src then clone and build the package:
```sh
cd ~/catkin_ws/src
git clone https://github.com/mentor-dyun/ros-i2cpwmboard.git
cd ..
catkin_make
source devel/setup.sh (source ~/.bashrc)

cd ~/catkin_ws/src
git clone https://github.com/neko7055/ncu_mdog.git
cd ..
catkin_make
source devel/setup.sh (source ~/.bashrc)
roscd ncu_mdog/startup
sudo chmod 777 ./*
sudo sh initenv.sh
```
<p align="right">(<a href="#top">back to top</a>)</p>

## Usage

```sh
roslaunch ncu_mdog project_sample.launch
```

## Acknowledgments

+ [RTIMULIB](https://github.com/jetsonhacks/RTIMULib.git)
+ [ros_i2cbread](https://github.com/mentor-dyun/ros-i2cpwmboard.git)

<p align="right">(<a href="#top">back to top</a>)</p>

