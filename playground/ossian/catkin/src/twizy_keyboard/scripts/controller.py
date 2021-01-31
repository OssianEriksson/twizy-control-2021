import rospy

from pynput import keyboard

from std_msgs.msg import Float64


MAX_FORWARD_SPEED = 10
MAX_BACKWARD_SPEED = 10
ACCELLERATION = 10

MAX_STEERING_ANGLE = 0.7
STEERING_ACCELERATION = 1


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


def controller():
    pub_axle = rospy.Publisher(
        '/twizy/rear_axle_controller/command', Float64, queue_size=1)
    pub_ackermann = rospy.Publisher(
        '/twizy/ackermann_controller/command', Float64, queue_size=1)

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

    rospy.init_node('twizy_keyboard_controller')

    rate = rospy.Rate(30)
    last_time = rospy.rostime.get_rostime()

    axle = 0
    steering = 0

    def update_value(current, reference, speed):
        last = current

        current += speed if current < reference else -speed

        return current if (current < reference) == (last < reference) else reference

    while not rospy.is_shutdown():
        control_axle = controls[keys["up"]].active - \
            controls[keys["down"]].active
        control_steering = controls[keys["left"]
                                    ].active - controls[keys["right"]].active

        for control in controls.values():
            control.mark_processed()

        control_axle *= MAX_FORWARD_SPEED if control_axle > 0 else MAX_BACKWARD_SPEED
        control_steering *= MAX_STEERING_ANGLE

        curr_time = rospy.rostime.get_rostime()
        dt = curr_time.to_sec() - last_time.to_sec()
        last_time = curr_time

        axle = update_value(axle, control_axle, ACCELLERATION * dt)
        steering = update_value(
            steering, control_steering, STEERING_ACCELERATION * dt)

        pub_axle.publish(axle)
        pub_ackermann.publish(steering)

        rate.sleep()

    listener.stop()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
