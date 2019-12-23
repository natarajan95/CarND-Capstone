import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        # pass
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio

        
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        #kp = 0.3
        #ki = 0.1
        #kd = 0.
        #mn = 0. # Minimum throttle value
        #mx = 0.2 # Max throttle value
        self.throttle_controller = PID(kp=0.3, ki=0.1, kd=0.0, mn=0.0, mx=0.2)
        # mn & mx - Min & Max throttles
        
        # Filterout high frequency noise in velocity (vel_lpf)
        tau = 0.5 # tau - Cutoff frequency
        ts = .02  # ts - Sample time
        self.vel_lpf = LowPassFilter(tau, ts)        
        

        self.last_time = rospy.get_time()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # If drive by wire enabled, human driver is driving - So reset controller (to avoid error accum)
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.
        
        # Else get current velocity (after passing through LPF) & steer
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        ##Video - DBW Walkthrough 9:00
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 # N*m - to hold the car in place if we are stopped at a light.  Accel ~ 1m/s**2

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # ##Video - DBW Walkthrough 2:12 & 9:00

        return throttle, brake, steering