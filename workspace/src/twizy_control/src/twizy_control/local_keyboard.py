import rospy

from pynput import keyboard
from math import radians

from twizy_msgs.msg import CarControl

MAX_FORWARD_SPEED = 5  # (km/h)
MAX_BACKWARD_SPEED = 5  # (km/h)

MAX_STEERING_ANGLE = 30  # (degrees)


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

    rospy.init_node('local_keyboard')

    pub_control = rospy.Publisher('car_control', CarControl, queue_size=1)

    rate = rospy.Rate(10)  # 10 Hz

    control = CarControl()

    while not rospy.is_shutdown():
        angle = controls[keys["right"]].active - controls[keys["left"]].active
        speed = controls[keys["up"]].active - controls[keys["down"]].active

        for c in controls.values():
            c.mark_processed()

        speed *= MAX_FORWARD_SPEED if speed > 0 else MAX_BACKWARD_SPEED
        angle *= MAX_STEERING_ANGLE

        control.angle = radians(angle)  # degrees to radians
        control.speed = speed / 3.6     # km/h to m/s

        pub_control.publish(control)

        rate.sleep()

    listener.stop()
