# DumplingBot

## apriltagslam package
Install:  
gtsam from https://bitbucket.org/gtborg/gtsam/downloads/  
Download apriltagslam source code from
```
git clone https://github.com/ProjectArtemis/aprilslam
```
Change mapper.cpp:  
```C++
Mapper::Mapper(double relinearize_thresh, int relinearize_skip)
    : init_(false),
      params_(ISAM2GaussNewtonParams(), relinearize_thresh, relinearize_skip),
      isam2_(params_){
	Vector tag_noise_vector(6);
	tag_noise_vector << 0.2,0.2,0.2,0.1,0.1,0.1;
	Vector small_noise_vector(6);
	small_noise_vector << 0.1,0.1,0.1,0.05,0.05,0.05;
	tag_noise_=noiseModel::Diagonal::Sigmas(tag_noise_vector);
	small_noise_=noiseModel::Diagonal::Sigmas(small_noise_vector);
}
```

## Mecanum drive need to measure:
* `base_width`
* `base_length`
* `wheel_gap`
* `wheel_setback`
* `wheel_radius`

To send command to `mecanum_command`  
```
Float32MultiArray: fl,fr,bl,br
```

## localization 
`map.csv` format:  
`id,px,py,pz,ox,oy,oz,ow`  
need to change equation for dist 
dependes on position and orientation of camera

# DumplingBot Web User Interface

## Dependencies
* `ros-kinetic-rosbridge-suite`

## To run
Ensure roscore is running on the local machine.
```
$ source /opt/ros/kinetic/setup.bash
$ roscore
```
Run all nodes needed to operate the robot.
Start the rosbridge server. By default this runs on port 9090.
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
Now open `index.html` with a web browser.
