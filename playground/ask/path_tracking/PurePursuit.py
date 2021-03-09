import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

# Parameters
k = 0.1    # look forward gain
Lfc = 1.5  # [m] look-ahead distance (Must be over 1)
Kp = 2     # speed proportional gain
dt = 0.1   # [s] time tick
WB = 1.686  # [m] wheel base of vehicle
target_speed = 10/3.6  # [m/s] Target speed
max_time = 250  # [s] max simulation time

show_animation = True


class State:
    """
    A class to represent the current state of the vehicle, that is, position, velocity and yaw(angle to the x-axis).
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x  # Current x-coordinate
        self.y = y  # Current y-Coordinate
        self.yaw = yaw  # Current yaw of the vehicle
        self.v = v  # Current velocity
        self.front_x = self.x + ((WB / 2) * math.cos(self.yaw))  # x position of the front axle
        self.front_y = self.y + ((WB / 2) * math.sin(self.yaw))  # y position of the front axle
        self.target_speed = target_speed

    def update_from_gps(self, gps_data, v):
        """
        A method to update the vehicles position using GPS data
        :param gps_data: A 4*n(?) matrix where gps_data[0] is the x position, gps_data[1] is the y position and
        gps_data[3] is the yaw of the vehicle
        :param v: Integer with the GPSs external calculation of the vehicles velocity
        """
        self.x = gps_data[0] - 0.36 * math.cos(self.yaw)  # No clue what 0.36 and 0.34 is, maybe use tf here?
        self.y = gps_data[1] - 0.34 * math.sin(self.yaw)
        self.yaw = gps_data[3]
        self.v = v
        self.front_x = self.x + ((WB / 2) * math.cos(self.yaw))
        self.front_y = self.y + ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        """
        Update method used when no GPS is available
        :param a: Float with the internal calculation of the vehicles acceleration
        :param delta: Float with the current steering angle (in radians?)
        """
        # Limiting the vehicles steering angle to a maximum of 0.7 radians (= 40 degrees)
        slow = False
        if delta > 0.7:
            self.target_speed = target_speed * pow(abs(0.7/delta), 3)
            delta = 0.7
            slow = True
        else:
            self.target_speed = target_speed
        if delta < -0.7:
            self.target_speed = target_speed * pow(abs(0.7/delta), 3)
            delta = -0.7
        elif not slow:
            self.target_speed = target_speed

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.front_x = self.x + ((WB / 2) * math.cos(self.yaw))
        self.front_y = self.y + ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        """
        Calculates the distance from the vehicles front axle to a given point
        :param point_x: Float with the x-coordinate of the desired point
        :param point_y: Float with the y-coordinate of the desired point
        :return: Float with the euclidean norm from the vehicles front axle and the specified point
        """
        dx = self.front_x - point_x
        dy = self.front_y - point_y
        return math.hypot(dx, dy)


class States:
    """
    A class to remember both the planned path and the actual path of the vehicle
    """

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.front_x = []
        self.front_y = []
        self.v = []
        self.t = []
        self.e = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.e.append(np.sin(state.yaw)*Lfc)
        self.front_x.append(state.front_x)
        self.front_y.append(state.front_y)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    """
    A simple method that calculates the desired acceleration for a target velocity, that is, a negative value if the
    vehicle is moving to quickly and vice versa.
    :param target: Float with the target velocity
    :param current: Float with the current velocity of the vehicle
    :return: Float with the new acceleration to achieve target velocity
    """
    a = Kp * (target - current)

    return a


class TargetCourse:
    """
    A class to represent the target course or the planned path.
    """

    def __init__(self, cx=None, cy=None):
        self.path_generated = False     # False
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def is_path_generated(self):
        """
        Quick method to se if the path has been generated
        :return: Boolean if a path exist
        """
        if self.path_generated:
            gen = True
        else:
            gen = False

        return gen

    def set_path(self, gps_x, gps_y, yaw):
        """
        The method that makes the path, here its made from scratch and resembles a tan^-1 curve. This is simply for
        testing purposes and to implement a real path one would simply just have to import a Path file and assign
        x and y values to self.cx and self.cy respectively
        :return: Two lists, self.cx and self.cy. cx contains the courses x-coordinate and cy the y-coordinates
        """
        front_x = gps_x - ((WB / 2) * math.cos(yaw))
        front_y = gps_y - ((WB / 2) * math.sin(yaw))
        str_coordinates = []
        with open('XY_path.txt', 'r') as coordinates_file:
            for coordinates_line in coordinates_file:
                # Line for line extracting the numerical coordinates from unwanted characters
                str_coordinates.append(coordinates_line.strip("{}\n").split(","))
        str_coordinates.pop(0)
        str_coordinates = str_coordinates[:len(str_coordinates) - 300]
        a = []
        b = []
        for element in str_coordinates:
            if len(element) == 4:
                b.append(front_y + float(element[3].strip('"'))/2)
            else:
                b.append((front_x + int(element[3].strip('"')) + float(element[4].strip('"')) / 100000)/2)
            a.append((front_y + int(element[1].strip('"')) + float(element[2].strip('"')) / 100000)/2)

        self.cx = a
        self.cy = b

        # Old code for tan curve
        # self.cx = np.arange(front_y, front_x + 20 - 1, 0.1)
        # self.cy = [2 for x in self.cx]
        # self.cy = [front_y + a * np.arctan((c/b + 3)) + a*np.arctan((1 / b)*((x-front_x) - 3 * b - c)) for x in self.cx]

        return self.cx, self.cy

    def search_target_index(self, state):
        """
        A method that finds the index of the lookahead point and makes it possible to vary the lookahead distance
        depending on current velocity
        :param state: State object containing the current stare of the vehicle
        :return: Index of the lookahead point and the distance to that point
        """
        # search nearest point index
        dx = [state.front_x - icx for icx in self.cx]
        dy = [state.front_y - icy for icy in self.cy]
        d = np.hypot(dx, dy)
        ind = np.argmin(d)
        self.old_nearest_point_index = ind

        look_forward = Lfc  # Update look ahead distance, (+ k*state.v to make it proportional to speed)

        # search look ahead target point index
        while look_forward > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, look_forward


def pure_pursuit_steer_control(state, trajectory, pind):
    """
    The Pure Pursuit method. Using the current state of the vehicle, the planed trajectory and the lookahead point index
    :param state: State object containing the current stare of the vehicle
    :param trajectory: TargetCourse object describing the planned path
    :param pind: Index to the lookahead point
    :return: delta, a float containing the steering angle and ind, an index of the lookahead point guaranteed to be
    within the indices of the planned trajectory
    """
    ind, look_forward = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.front_y, tx - state.front_x) - state.yaw
    delta = math.atan2(2.0 * WB * math.sin(alpha) / look_forward, 1.0)
    return delta, ind


def plot_arrow(x, y, yaw, length=1.6, width=0.5, fc="black", ec="k"):
    """
    Plots an arrow from the vehicles middle and pointing in its current yaw
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    """
    The main function to be run
    """
    #  Target course
    path = TargetCourse()

    cx, cy = path.set_path(6, 5, 3.14)  # Here 6 and 5 decide the paths start location, x = 6, y = 5

    # Initial state of the vehicle
    state = State(x=path.cx[0], y=path.cy[0], yaw=0.0, v=0.0)

    last_index = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_index, _ = target_course.search_target_index(state)

    while max_time >= time and last_index > target_index:

        # Calculate control input

        ai = proportional_control(state.target_speed, state.v)
        di, target_index = pure_pursuit_steer_control(
            state, target_course, target_index)

        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # For stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.front_x, states.front_y, "-b", label="trajectory")
            plt.plot(cx[target_index], cy[target_index], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_index >= target_index, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.front_x, states.front_y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
