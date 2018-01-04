import rospy
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_controller = YawController(
		kwargs['wheel_base'], 
		kwargs['steer_ratio'], 
		kwargs['min_speed'], 
		kwargs['max_lat_accel'], 
		kwargs['max_steer_angle'])
	self.throttle_pid = PID(
		kwargs['kp'], 
		kwargs['ki'], 
		kwargs['kd'], 
		kwargs['mn'], 
		kwargs['mx'])

	self.prev_time = None
        pass

    def control(self, twist_cmd, current_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
	if( self.prev_time is None):
	    self.prev_time = rospy.get_time()
	    return 0., 0., 0.
	
	# update the delta
	current_time = rospy.get_time()
	time_delta = current_time - self.prev_time
	self.prev_time = current_time

	lin_vel = abs(twist_cmd.twist.linear.x)
	ang_vel = twist_cmd.twist.angular.z
        vel_err = lin_vel - current_velocity.twist.linear.x

	steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_velocity.twist.linear.x)
	throttle = self.throttle_pid.step(vel_err, time_delta)
        
	# Return throttle, brake, steer
        return throttle, 0., steer
