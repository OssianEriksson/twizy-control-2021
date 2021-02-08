from matplotlib import pyplot as plt
import numpy as np


class Coordinate:
    def __init__(self, x, y):
        """first Argument is the x coordinate and second is the y coordinate"""
        self.x = x
        self.y = y

class Map:
    def __init__(self,coordstart, coordbreak1, coordbreak2, coordend, twizydist, offset):
        self.coordstart = coordstart
        self.coordbreak1 = coordbreak1
        self.coordbreak2 = coordbreak2
        self.coordend = coordend
        self.twizydist = twizydist
        self.offset = offset

    # all coordinates come from the GPS, twizydist from rear ultrasonic sensor
    def generateMap(self):
        offset_length = np.linspace(self.coordstart, self.coordbreak1 - 0.001, 20)
        extradots = np.linspace(self.coordbreak1, self.coordbreak1 + 0.1, 20)
        parkingspot_length = np.linspace(self.coordbreak1 + 0.1, self.coordbreak2, 20)
        end_length = np.linspace(self.coordbreak2, self.coordend, 20)

        parkingspotdepth = 2.5
        #  y = 25x + (distance . 25*offset)
        parkmap = {}

        for x in offset_length:
            parkmap[x] = self.twizydist
        for x in extradots:
            parkmap[x] = 25 * x + (self.twizydist - 25 * self.offset)
        for x in parkingspot_length:
            parkmap[x] = parkingspotdepth + self.twizydist
        for x in end_length:
            parkmap[x] = self.twizydist

        return parkmap


def filter_collision(x_0, y_0, deriv, parkingmap):
    circleRadius = 0.69
    #parkingmap = generateMap(coord_start, coord_p1, coord_p2, coord_end, distance)
    carlength = 2.32
    angle = np.arctan(deriv)
    counter = 0

    p1 = Coordinate(x_0 - carlength * np.cos(angle), y_0 - carlength * np.sin(angle))
    p2 = Coordinate(x_0, y_0)

    tang_linspace = np.linspace(p1.x, p2.x, 20)
    # tangent = deriv * (tang_linspace - x_0) + y_0
    # plt.plot(tang_linspace, tangent)
    for x in tang_linspace:
        counter = counter + 1
        if counter > 6 or counter < 16:
            for key in parkingmap:
                tangent = deriv * (x - x_0) + y_0
                dist = distance1(x, tangent, key, parkingmap.get(key))
                if dist < circleRadius:
                    return True
    return False


def distance1(x1, y1, x2, y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def f_arctan(a, b, c, x):
    return a * np.arctan(c / b + 3) + a * np.arctan((1 / b) * (x - 3 * b - c))


def f_arctan_d1(a, b, c, x):  # derivative
    return a / (b * (np.power(x - 3 * b - c, 2) / np.power(b, 2) + 1))


def f_arctan_d2(a, b, c, x):  # second derivative
    return (2 * a * (-3 * b - c + x)) / (np.power(b, 3) * np.power(np.power(-3 * b - c + x, 2) / np.power(b, 2) + 1, 2))


def path(current, goal, parkingmap):
    """first argument takes in the current coordinate of the car and the second is the
     coordinate of the goal position this method will return the optimal
     trajectory"""
    depth = np.abs(goal.y - current.y)  # 2
    length = np.abs(goal.x - current.x)  # 10
    period = 2
    phase = 3
    deptharray = np.linspace(0.5, depth / 1.15, 30)
    periodarray = np.linspace(0, period, 20)
    lengtharray = np.linspace(current.x, current.x + length, 20)
    phasearray = np.linspace(0, phase, 20)

    radius = []
    for a in deptharray:
        for b in periodarray:
            for c in phasearray:
                if a != 0 and b != 0:
                    del radius[:]
                    collision1 = False
                    counter = 0
                    function = f_arctan(a, b, c, lengtharray)
                    # plt.plot(lengtharray,function)
                    if abs((f_arctan(a, b, c,
                                     goal.x) - goal.y)) > 0.2:  # filter out every function of which distance to
                        break  # to the goal position at goal.x is to big

                    if f_arctan_d1(a, b, c, goal.x) > 0.2:  # filter out every function that ends with a slope
                        break  # larger than 0.1 rad (to ensure that the car is parked horizontally)

                    for x in lengtharray:  # filter out every function with a radius of curvature smaller
                        if x != 0:  # than that of renault twizy
                            f_deriv = f_arctan_d1(a, b, c, x)  # first derivative
                            f_deriv2 = f_arctan_d2(a, b, c, x)  # second derivative

                            if f_deriv2 != 0:
                                radius.insert(counter, np.absolute(np.power(1 + np.power(f_deriv, 2),
                                                                            3 / 2) / f_deriv2))  # formula for radius of curvature
                                counter = counter + 1
                    #if min(radius) < 1.3:  # filter out every function with a radius of curvature smaller		
                    #    break  # than that of renault twizy

                    for x in lengtharray:
                        if x != 0:
                            f_deriv = f_arctan_d1(a, b, c, x)  # first derivative
                            collision = filter_collision(x, f_arctan(a, b, c, x), f_deriv, parkingmap)

                            if collision == True:
                                collision1 = True

                    if collision1 == True:
                        break

                    plt.plot(lengtharray, function)
                    print([a, b, c])
                    return a, b, c

                     

def main():
    offset = 1.5
    parkingLength = 7
    distance = 1
    current = Coordinate(0, 0)

    goal = Coordinate(offset + parkingLength - 1.25, distance + 1.25)


    start = 0
    p1 = offset
    p2 = offset + parkingLength
    end = offset + parkingLength + 3

    karta = Map(start, p1, p2, end, distance, offset).generateMap()
    plt.plot(list(karta.keys()), list(karta.values()), 'black')
    # plt.show()

    path(current, goal,karta)
    plt.show()

if __name__ == '__main__':
    main()
