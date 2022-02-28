turn_speed = 5.144
# speeds_dict = {"cruise": 15.4333, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
speeds_dict_model1 = {"cruise": 15.4333, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
speeds_dict_model2 = {"cruise": 10.288886666666667, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
angle_intervals = [25, 100, 150]
accel_max = 3 # m/s**2
vertical_speed = 5
vertical_accel = 3.5

class Drone:

    def __init__(self, flightNumber, dep, arr, hDep, droneType):
        self.flight_number = flightNumber
        self.accel_max = accel_max
        self.cruise_speed = 15.4333  # in m/s
        self.turn_speed = turn_speed  # m/s
        self.vertical_speed = vertical_speed  # m/s
        self.vertical_accel = vertical_accel  # m/s**2
        self.braking_distance = 30  # m
        self.deposit_time = None
        self.is_loitering_mission = None
        self.dep = dep  # Departure node in the path_dict
        self.arr = arr  # Arrival node in the path_dict
        self.drone_type = None
        # TODO edge depart et edge arr != NONE SI ON EST DANS UNCONSTRAINED
        self.dep_edge = None
        self.arr_edge = None
        self.dep_time = hDep
        self.type = droneType
        if droneType == 'MP30':
            self.speeds_dict = speeds_dict_model1
        elif droneType == 'MP20':
            self.speeds_dict = speeds_dict_model2
        else:
            raise Exception

        self.departure_vertiport = None
        self.is_unconstrained_departure = None
        self.arrival_vertiport = None
        self.is_unconstrained_arrival = None
        # A path object that will be used to store the drone current path
        self.path_object = None

    def find_current_edge(self, current_t, graph):
        """Finds the edge of the drone at given time t in seconds."""
        from_node = ''
        for tt in self.path_object.path_dict:
            if tt <= current_t:
                from_node = self.path_object.path_dict[tt]
            if tt > current_t:
                if from_node != '':
                    return from_node, self.path_object.path_dict[tt]
        # if conflict time is before take off time or beetween first and second node
        return self.path_object.path[0], self.path_object.path[1]


def return_speed_from_angle(angle, drone):
    if angle <= angle_intervals[0]:
        return drone.speeds_dict["cruise"]
    elif angle_intervals[0] < angle <= angle_intervals[1]:
        return drone.speeds_dict["turn1"]
    elif angle_intervals[1] < angle <= angle_intervals[2]:
        return drone.speeds_dict["turn2"]
    elif angle_intervals[2] < angle:
        return drone.speeds_dict["turn3"]


def return_braking_distance(v1, v2):
    avg_speed = (v1 + v2)/2
    return avg_speed * abs(v1 - v2)/accel_max


def return_accel_time(v1, v2):
    accel_time = abs(v1 - v2) / accel_max
    return accel_time


