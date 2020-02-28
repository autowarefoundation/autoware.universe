# Complex32.msg
# Can be used to represent yaw angle for trajectories
# To convert back to a yaw angle, see
# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
float32 real 1.0 # cos(yaw / 2)
float32 imag 0.0 # sin(yaw / 2)
