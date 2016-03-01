Header header

ArmState arm_vel

# for future
# 1: wheel_velocity_mode(lwheel, rwheel)
# 2: body_velocity_mode(v, w)
int64 base_vel_input_mode

float64 wheel_left_vel
float64 wheel_right_vel

# for future
#  It is possible that yozakura will be controlled with v, w
#  base_vel is just for such case
geometry_msgs/Twist base_vel
