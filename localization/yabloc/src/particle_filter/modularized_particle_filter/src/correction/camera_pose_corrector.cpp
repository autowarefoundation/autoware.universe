#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <iostream>

CameraPoseCorrector::CameraPoseCorrector() : Node("camera_pose_corrector") {}