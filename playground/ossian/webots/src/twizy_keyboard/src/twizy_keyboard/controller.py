import rospy

from pynput import keyboard

from std_msgs.msg import Float64MultiArray

MAX_FORWARD_SPEED = 5 / 3.6
MAX_BACKWARD_SPEED = 5 / 3.6

MAX_STEERING_ANGLE = 40


class Control:
    def __init__(self):
        self.pressed = False
        self.tapped = False

    def set_key_down(self):
        self.pressed = True
        self.tapped = True

    def set_key_up(self):
        self.pressed = False

    def mark_processed(self):
        self.tapped = False

    @property
    def active(self):
        return 1 if self.pressed or self.tapped else 0


def main():
    keys = {"up": keyboard.Key.up, "down": keyboard.Key.down,
            "left": keyboard.Key.left, "right": keyboard.Key.right}

    controls = {key: Control() for key in keys.values()}

    def on_press(key):
        if key in controls:
            controls[key].set_key_down()

    def on_release(key):
        if key in controls:
            controls[key].set_key_up()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    rospy.init_node('keyboard_controller')

    pub_control = rospy.Publisher(
        'car_control', Float64MultiArray, queue_size=1)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        control_axle = controls[keys["up"]].active - \
            controls[keys["down"]].active
        control_steering = controls[keys["left"]].active - \
            controls[keys["right"]].active

        for control in controls.values():
            control.mark_processed()

        control_axle *= MAX_FORWARD_SPEED if control_axle > 0 else \
            MAX_BACKWARD_SPEED
        control_steering *= MAX_STEERING_ANGLE

        control = Float64MultiArray()
        control.data = [control_axle, control_steering]

        pub_control.publish(control)

        rate.sleep()

    listener.stop()
