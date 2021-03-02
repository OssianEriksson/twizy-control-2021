import rospy

from pynput import keyboard
from math import radians

from twizy_msgs.msg import CarControl


class Control:
    """
    Class for handling key control.

    Instead of only checking at the time of update if a key is pressed, it
    might be useful to instead use a key callback to store key actions until
    the next update comes around. This class holds two booleans:

    self.pressed is True if the key is currently beeing held down

    self.tapped is True if the key has been pressed down at least once since
    the control was last processed

    self.tapped should be included for smoother key control, since the
    keypresses are then not dropped if they happen to occur between update
    cycles. This inclusion is automatically handeled by using the active
    property
    """

    def __init__(self):
        self.pressed = False
        self.tapped = False

    def set_key_down(self):
        """
        Call this function to mark the control key as beeing pressed down -
        intended to be a callback function for a key listener
        """

        self.pressed = True
        self.tapped = True

    def set_key_up(self):
        """
        Call this function to mark the control key as beeing released down -
        intended to be a callback function for a key listener
        """

        self.pressed = False

    def mark_processed(self):
        """
        Call this function to mark the control key as having been processed.
        This function should be called at the end of the update cycle and
        resets the value of self.tapped to False
        """

        self.tapped = False

    @property
    def active(self):
        """
        Active is 1 if the key is currently beeing held down or was pressed
        down atleast once since the control was last processed, 0 otherwise
        """

        return 1 if self.pressed or self.tapped else 0


def main():
    # Keyboard keys to use for different actions
    keys = {"up": keyboard.Key.up, "down": keyboard.Key.down,
            "left": keyboard.Key.left, "right": keyboard.Key.right}

    # Initialize dict of Control instances to keep track of which keys have
    # been or are currently beeing pressed down
    controls = {key: Control() for key in keys.values()}

    def on_press(key):
        """
        Keyboard callback function. Should be called once a key has been
        pressed down
        """

        # Register the key press if the key is relevant to us (it is in
        # controls)
        if key in controls:
            controls[key].set_key_down()

    def on_release(key):
        """
        Keyboard callback function. Should be called once a key has been
        released down
        """

        # Register the key release if the key is relevant to us (it is in
        # controls)
        if key in controls:
            controls[key].set_key_up()

    # Stat listening for key presses
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Initialize ROS node
    rospy.init_node('local_keyboard')

    # Read parameters from ROS parameter server
    max_forward_speed = rospy.get_param('~max_forward_speed', 1.38)    # 5 km/h
    max_backward_speed = rospy.get_param('~max_backward_speed', 1.38)  # 5 km/h
    max_steering_angle = rospy.get_param('~max_steering_angle', 0.52)  # 30 deg

    # Initialize publisher for car control messages
    pub_control = rospy.Publisher('car_control', CarControl, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    control = CarControl()

    # Main loop
    while not rospy.is_shutdown():
        # Calculate if angle and speed should be positive, negative or zero.
        # Pressing opposing keys cancels out the action
        angle = controls[keys["right"]].active - controls[keys["left"]].active
        speed = controls[keys["up"]].active - controls[keys["down"]].active

        # Mark all keys as having been processed. We will not read the
        # Control.active property any more this loop iteration
        for c in controls.values():
            c.mark_processed()

        # Amplify the reference angle and speed values to their correct
        # magnitudes
        angle *= max_steering_angle
        speed *= max_forward_speed if speed > 0 else max_backward_speed

        # Set the corresponing values in the message to be published
        control.angle = angle
        control.speed = speed

        # Publish CarControl message containing reference angle and speed
        pub_control.publish(control)

        rate.sleep()

    # Close listener
    listener.stop()
