# DumplingBot

# apriltagslam package
install:
gtsam
https://bitbucket.org/gtborg/gtsam/downloads/
download source code from
git clone https://github.com/ProjectArtemis/aprilslam
change mapper.cpp:
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

#Mecanum drive need to measure:
base_width;
base_length;
wheel_gap;
wheel_setback;
wheel_radius;

mecanum_command
Float32MultiArray: fl,fr,bl,br


catkin_make
