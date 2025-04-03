#!/usr/bin/env python3

"""
auto_exposure_control.py subscribes to a ROS image topic and performs
histogram based automatic exposure control based on the paper
"Automatic Camera Exposure Control", N. Nourani-Vatani, J. Roberts
"""
#import roslib
import math
import numpy as np
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from usb_cam_auto_exposure.cfg import ControllerSettingsConfig


# PID Controller: https://github.com/m-lundberg/simple-pid/blob/master/simple_pid/pid.py

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value


class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
        differential_on_measurement=True,
        error_map=None,
        time_fn=None,
        starting_output=0.0,
    ):
        """
        Initialize a new PID controller.

        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param differential_on_measurement: Whether the differential term should be calculated on
            the input directly rather than on the error (which is the traditional way).
        :param error_map: Function to transform the error value in another constrained value.
        :param time_fn: The function to use for getting the current time, or None to use the
            default. This should be a function taking no arguments and returning a number
            representing the current time. The default is to use time.monotonic() if available,
            otherwise time.time().
        :param starting_output: The starting point for the PID's output. If you start controlling
            a system that is already at the setpoint, you can set this to your best guess at what
            output the PID should give when first calling it to avoid the PID outputting zero and
            moving the system away from the setpoint.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.differential_on_measurement = differential_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_error = None
        self._last_input = None

        if time_fn is not None:
            # Use the user supplied time function
            self.time_fn = time_fn
        else:
            import time

            try:
                # Get monotonic time to ensure that time deltas are always positive
                self.time_fn = time.monotonic
            except AttributeError:
                # time.monotonic() not available (using python < 3.3), fallback to time.time()
                self.time_fn = time.time

        self.output_limits = output_limits
        self.reset()

        # Set initial state of the controller
        self._integral = _clamp(starting_output, output_limits)

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.

        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).

        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        now = self.time_fn()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            return self._last_output

        # Compute error terms
        error = self.setpoint - input_
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)
        d_error = error - (self._last_error if (self._last_error is not None) else error)

        # Check if must map the error
        if self.error_map is not None:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # Avoid integral windup

        if self.differential_on_measurement:
            self._derivative = -self.Kd * d_input / dt
        else:
            self._derivative = self.Kd * d_error / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_error = error
        self._last_time = now

        return output

    def __repr__(self):
        return (
            '{self.__class__.__name__}('
            'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
            'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
            'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
            'proportional_on_measurement={self.proportional_on_measurement!r}, '
            'differential_on_measurement={self.differential_on_measurement!r}, '
            'error_map={self.error_map!r}'
            ')'
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.

        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, disable the PID by setting auto mode to False and later when the PID should
        be turned back on, pass the last output variable (the control variable) and it will be set
        as the starting I-term when the PID is set to auto mode.

        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode. Has no effect if the PID is already in
            auto mode.
        """
        if enabled and not self._auto_mode:
            # Switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if (last_output is not None) else 0
            self._integral = _clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).

        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.

        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = _clamp(self._integral, self.output_limits)

        self._last_time = self.time_fn()
        self._last_output = None
        self._last_input = None
        self._last_error = None



# I term for PI controller
err_i = 0


pid = PID(0.1, 0.1, 0.0, setpoint=2.5, output_limits=(0, 1), sample_time=1/30)


def pid_reconfigure_callback(config, level):
    global pid
    
    pid.Kp = config.k_p
    pid.Ki = config.k_i
    pid.Kd = config.k_d
    pid.setpoint = config.setpoint
    return config



# TODO: make this more generic
# This is very specific to the Logitech BIRO
def normalize_exposure(shutter_speed):
    """
    Normalize shutter_speed to a exposure between 0 and 1
    
    The shutter_speed range is between 3 and 2000
    and has a exponential curve
    """
    # Ensure shutter_speed is within bounds
    shutter_speed = max(3, min(shutter_speed, 2000))
    
    # Apply logarithmic transformation to account for non-linear relationship
    # where small changes at low values have larger effect
    log_min = math.log(3)
    log_max = math.log(2000)
    log_value = math.log(shutter_speed)
    
    # Normalize to range [0, 1]
    normalized = (log_value - log_min) / (log_max - log_min)
    
    return normalized

def denormalize_exposure(exposure):
    """
    Convert a normalized exposure (0 to 1) back to shutter_speed (3 to 2000)
    
    This is the inverse of normalize_exposure
    """
    # Ensure exposure is within bounds
    exposure = max(0, min(exposure, 1))
    
    # Apply inverse logarithmic transformation
    log_min = math.log(3)
    log_max = math.log(2000)
    log_value = exposure * (log_max - log_min) + log_min
    
    # Convert back to original scale
    shutter_speed = math.exp(log_value)
    
    # Round to integer and ensure within bounds
    shutter_speed = int(round(shutter_speed))
    shutter_speed = max(3, min(shutter_speed, 2000))
    
    return shutter_speed


def get_exposure(dyn_client):
    values = dyn_client.get_configuration()
    return normalize_exposure(values['exposure'])

def set_exposure(dyn_client, exposure):
    params = {'exposure' : denormalize_exposure(exposure)} 
    config = dyn_client.update_configuration(params)

def image_callback(image, args):
    global err_i
    bridge = args['cv_bridge']
    dyn_client = args['dyn_client']
    cv_image = bridge.imgmsg_to_cv2(image,
                                    desired_encoding = "bgr8")
    
    (rows, cols, channels) = cv_image.shape
    if (channels == 3):
        brightness_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[:,:,2]
    else:
        brightness_image = cv_image

    #crop_size = 10
    #brightness_image = brightness_image[rows-crop_size:rows+crop_size, cols-crop_size:cols+crop_size]

    #(rows, cols) = brightness_image.shape
    
    hist = cv2.calcHist([brightness_image],[0],None,[5],[0,256])
    
    mean_sample_value = 0
    for i in range(len(hist)):
        mean_sample_value += hist[i]*(i+1)
        
    mean_sample_value /= (rows*cols)
    
    #focus_region = brightness_image[rows/2-10:rows/2+10, cols/2-10:cols/2+10]
    #brightness_value = numpy.mean(focus_region)

    # Middle value MSV is 2.5, range is 0-5
    # Note: You may need to retune the PI gains if you change this
    desired_msv = 2.5

    err_p = desired_msv-mean_sample_value



    control = pid(mean_sample_value)
    if (abs(err_p) > 0.5):
        set_exposure(dyn_client, control)
    
        
def main(args):
    rospy.init_node('auto_exposure_control')

    bridge = CvBridge()
    camera_name = rospy.get_param('~camera_name')
    dyn_client = dynamic_reconfigure.client.Client(camera_name)

    # params = {'auto_exposure' : False}
    # config = dyn_client.update_configuration(params)
    
    args = {}
    args['cv_bridge'] = bridge
    args['dyn_client'] = dyn_client
    img_sub=rospy.Subscriber('image_raw', Image, image_callback, args)

    srv = Server(ControllerSettingsConfig, pid_reconfigure_callback)

    rospy.spin()
    
if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
