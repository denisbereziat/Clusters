import math

drone_speed_factor = 1
drone_accel_factor = 0.7

turn_speed = 5.144
# speeds_dict = {"cruise": 15.4333, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
speeds_dict_model1 = {"cruise": 15.4333 * drone_speed_factor, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
speeds_dict_model2 = {"cruise": 10.288886666666667 * drone_speed_factor, "turn1": 5.144, "turn2": 2.5722, "turn3": 1.02889}
angle_intervals = [25, 100, 150]
accel_max = 3 * drone_accel_factor  # m/s**2
vertical_speed = 5
vertical_accel = 3.5


class Drone:

    def __init__(self, flightNumber, dep, arr, hDep, drone_type):
        self.flight_number = flightNumber
        self.accel_max = accel_max
        self.vertical_speed = vertical_speed  # m/s
        self.vertical_accel = vertical_accel  # m/s**2
        self.deposit_time = None
        self.dep = dep  # Departure nodes list in the path_dict
        self.arr = arr  # Arrival nodes list in the path_dict
        self.dep_edge = None
        self.arr_edge = None
        self.dep_time = hDep

        self.drone_type = drone_type
        if self.drone_type == 'MP30':
            self.speeds_dict = speeds_dict_model1
        elif self.drone_type == 'MP20':
            self.speeds_dict = speeds_dict_model2
        else:
            raise Exception

        # Stores if this is a mission that creates a geofence
        self.is_loitering_mission = None
        self.loitering_geofence = None  # [duration, x1, y2, x2, y2]

        self.departure_vertiport = None
        self.is_unconstrained_departure = None
        self.arrival_vertiport = None
        self.is_unconstrained_arrival = None

    '''
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
    '''

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

def return_vertical_accel_dist(v1, v2):
    avg_speed = (v1 + v2) / 2
    return avg_speed * abs(v1 - v2) / vertical_accel


def return_vertical_accel_time(v1, v2):
    accel_time = abs(v1 - v2) / vertical_accel
    return accel_time

#horizontal kinematics
def accelerated_speed(v1, distance):
    return math.sqrt(v1**2 + 2*accel_max*distance)

def decelerated_speed(v1, distance):
	#due to float representation inprecission (e.g. 0 that is -0.0000001) we 'round' the calculation
    return math.sqrt(max(0, v1**2 - 2*accel_max*distance))
    
def acceleration_distance(v1, v2):
    return (v2**2 - v1**2)/(2*accel_max)

def deceleration_distance(v1, v2):
    return acceleration_distance(v2, v1)
    
def acceleration_time(v1, v2):
    return (v2 - v1)/accel_max
    
def deceleration_time(v1, v2):
    return acceleration_time(v2, v1)

#vertical kinematics
def vertical_accelerated_speed(v1, distance):
    return math.sqrt(v1**2 + 2*vertical_accel*distance)

def vertical_decelerated_speed(v1, distance):
    return math.sqrt(v1**2 - 2*vertical_accel*distance)

def vertical_acceleration_distance(v1, v2):
    return (v2**2 - v1**2)/(2*vertical_accel)

def vertical_deceleration_distance(v1, v2):
    return vertical_acceleration_distance(v2, v1)
    
def vertical_acceleration_time(v1, v2):
    return (v2 - v1)/vertical_accel
    
def vertical_deceleration_time(v1, v2):
    return vertical_acceleration_time(v2, v1)
    
class Integrator:
    '''
    It is an class that describe drone's motion on the given segment 
    between two point of it's trajectory, given starting speed, segment distance,
    and maximum possible end speed, that minimize travel time.
    Maximum possible end speed is generally determined based on the turning angle 
    speed restriction at the end point but also at the next point in the traj. 
    '''
    def __init__(self, p0, p1, d, v0, v1, d_acc, v_acc, t_acc, d_cruise, v_cruise, t_cruise, d_dec, v_dec, t_dec):
        self.p0 = p0
        self.p1 = p1
        self.d = d
        self.v0 = v0
        self.v1 = v1
        self.d_acc = d_acc
        self.v_acc = v_acc
        self.t_acc = t_acc
        self.d_cruise = d_cruise
        self.v_cruise = v_cruise
        self.t_cruise = t_cruise
        self.d_dec = d_dec
        self.v_dec = v_dec
        self.t_dec = t_dec
        
        self.t_middle = self.time_at_distance(d/2)
    
    def __str__(self):
        res = "Integrator for: (" + str(self.p0) + "," + str(self.v0) + ")--(" + str(self.p1) + "," + str(self.v1) + ")\n"
        res += "Acceleration: d=" + str(self.d_acc) + ", v=" + str(self.v_acc) + ", t=" + str(self.t_acc) + "\n"
        res += "Cruisse: d=" + str(self.d_cruise) + ", v=" + str(self.v_cruise) + ", t=" + str(self.t_cruise) + "\n"
        res += "Deceleration: d=" + str(self.d_dec) + ", v=" + str(self.v_dec) + ", t=" + str(self.t_dec) + "\n"
        return res
    
    
    def end_time(self):
        return self.t_dec
    
    def middle_time(self):
        return self.t_middle
        
    def time_at_distance_after_begin(self, distance):
        if distance >= self.d:
            #distance is longer than segment, hence we approximate with average speed over segment
            return distance*self.t_dec/self.d
        else:
            return self.time_at_distance(distance)
    
    def time_at_distance_before_end(self, distance):
        if distance >= self.d:
            #distance is longer than segment, hence we approximate with average speed over segment
            return distance*self.t_dec/self.d
        else:
            return self.t_dec - self.time_at_distance(self.d - distance)
        
    def time_at_distance_after_middle(self, distance):
        if distance >= self.d/2:
            #distance is longer than half-segment, hence we approximate with average speed over second half of segment
            return distance*2*(self.t_dec - self.t_middle)/self.d
        else:
            return self.time_at_distance(self.d/2 + distance) - self.t_middle
    
    def time_at_distance_before_middle(self, distance):
        if distance >= self.d/2:
            #distance is longer than half-segment, hence we approximate with average speed over first half of segment
            return distance*2*self.t_middle/self.d
        else:
            return self.t_middle - self.time_at_distance(self.d/2 - distance)
        
    def time_at_distance(self, distance): #returns relative time from begining of the segment till required distnce
        if distance < self.d_acc:
            return acceleration_time(self.v0, accelerated_speed(self.v0, distance))
        elif distance < self.d_acc + self.d_cruise:
            return self.t_acc + (distance - self.d_acc)/self.v_cruise
        else:
            return self.t_cruise + deceleration_time(self.v_cruise, decelerated_speed(self.v_cruise, distance - self.d_acc - self.d_cruise))
    '''
    The following method solves directly optimal control problem and creates/return
    Integrator object.
    It takes two consecutive segment portion of trajectory (three points) with known
    distances, initial speed and maximum posible speed at middle and end point.
    The drone motion is coverned by the kinematic law with constant acceleration/deceleration,
    maximum cruising speed that are all defined in Drone class. 
    '''
    def integrate(p, v, d1, d2, drone):
        if len(p) == 3:
            if not(v[2] >= v[1] or d2 >= deceleration_distance(v[1], v[2])):
                #second segment is restrictive for v[1] that need be reduced
                v[1] = accelerated_speed(v[2], d2)
        
        d_accMax = acceleration_distance(max(v[0], v[1]), drone.speeds_dict["cruise"])
        d_acc01 = acceleration_distance(v[0], v[1])
        d_dec01 = -d_acc01
        delta_d = max(0, d1 - max(d_dec01, d_acc01))
        d_add = min(delta_d/2, d_accMax)
        
        #following 3 sections summarize drone motion in the first segment:
        #drone first accelerate, then cruising with max speed, then decelerate
        #ACCELERATION
        d_acc = min(d1, max(0, d_acc01) + d_add)
        v_acc = accelerated_speed(v[0], d_acc)
        t_acc = acceleration_time(v[0], v_acc)
        #CONSTANT SPEED
        d_cruise = max(0, delta_d - 2*d_accMax)
        t_cruise = t_acc + d_cruise/v_acc
        #DECELERATION
        d_dec = min(d1, max(0, d_dec01) + d_add)
        v_dec = decelerated_speed(v_acc, d_dec)
        t_dec = t_cruise + deceleration_time(v_acc, v_dec)
    
        return Integrator(p[0], p[1], d1, v[0], v[1], d_acc, v_acc, t_acc, d_cruise, v_acc, t_cruise, d_dec, v_dec, t_dec)

class VerticalIntegrator:
    '''
    It is an class that describe drone's vertical motion on the given segment
    given starting speed, segment distance,
    and maximum possible end speed, that minimize travel time.
    Maximum possible end speed is generally determined based on the turning angle 
    speed restriction at the end point but also at the next point in the traj. 
    '''
    def __init__(self, d, v0, v1, d_acc, v_acc, t_acc, d_cruise, v_cruise, t_cruise, d_dec, v_dec, t_dec):
        self.d = d
        self.v0 = v0
        self.v1 = v1
        self.d_acc = d_acc
        self.v_acc = v_acc
        self.t_acc = t_acc
        self.d_cruise = d_cruise
        self.v_cruise = v_cruise
        self.t_cruise = t_cruise
        self.d_dec = d_dec
        self.v_dec = v_dec
        self.t_dec = t_dec
    
    def __str__(self):
        res = "Integrator for: " + str(self.v0) + "--" + str(self.v1) + "\n"
        res += "Acceleration: d=" + str(self.d_acc) + ", v=" + str(self.v_acc) + ", t=" + str(self.t_acc) + "\n"
        res += "Cruisse: d=" + str(self.d_cruise) + ", v=" + str(self.v_cruise) + ", t=" + str(self.t_cruise) + "\n"
        res += "Deceleration: d=" + str(self.d_dec) + ", v=" + str(self.v_dec) + ", t=" + str(self.t_dec) + "\n"
        return res
    
    
    def end_time(self):
        return self.t_dec
        
    def time_at_distance_after_begin(self, distance):
        if distance >= self.d:
            #distance is longer than segment, hence we approximate with average speed over segment
            return distance*self.t_dec/self.d
        else:
            return self.time_at_distance(distance)
    
    def time_at_distance_before_end(self, distance):
        if distance >= self.d:
            #distance is longer than segment, hence we approximate with average speed over segment
            return distance*self.t_dec/self.d
        else:
            return self.t_dec - self.time_at_distance(self.d - distance)
        
    def time_at_distance(self, distance): #returns relative time from begining of the segment till required distnce
        if distance < self.d_acc:
            return vertical_acceleration_time(self.v0, vertical_accelerated_speed(self.v0, distance))
        elif distance < self.d_acc + self.d_cruise:
            return self.t_acc + (distance - self.d_acc)/self.v_cruise
        else:
            return self.t_cruise + vertical_deceleration_time(self.v_cruise, vertical_decelerated_speed(self.v_cruise, distance - self.d_acc - self.d_cruise))
    '''
    The following method solves directly optimal control problem and creates/return
    Integrator object.
    The drone vertical motion is coverned by the kinematic law with constant acceleration/deceleration,
    maximum vertical speed that are all defined in Drone class. 
    '''
    def integrate(d, v0=0, v1=0):
        d_accMax = vertical_acceleration_distance(max(v0, v1), vertical_speed)
        d_acc01 = vertical_acceleration_distance(v0, v1)
        d_dec01 = -d_acc01
        delta_d = max(0, d - max(d_dec01, d_acc01))
        d_add = min(delta_d/2, d_accMax)
        
        #following 3 sections summarize drone vertical motion
        #drone first accelerate, then cruising with max vert speed, then decelerate
        #ACCELERATION
        d_acc = min(d, max(0, d_acc01) + d_add)
        v_acc = vertical_accelerated_speed(v0, d_acc)
        t_acc = vertical_acceleration_time(v0, v_acc)
        #CONSTANT SPEED
        d_cruise = max(0, delta_d - 2*d_accMax)
        t_cruise = t_acc + d_cruise/v_acc
        #DECELERATION
        d_dec = min(d, max(0, d_dec01) + d_add)
        v_dec = vertical_decelerated_speed(v_acc, d_dec)
        t_dec = t_cruise + vertical_deceleration_time(v_acc, v_dec)
    
        return VerticalIntegrator(d, v0, v1, d_acc, v_acc, t_acc, d_cruise, v_acc, t_cruise, d_dec, v_dec, t_dec)

'''
drone = Drone("MyFlight", None, None, 0, "MP30")
integrator = Integrator.integrate(["A", "B", "C"], [0, 10, 20], 50, 200, drone)

print(integrator)
print("end_time: ", integrator.end_time())
print("middle_time: ", integrator.middle_time())
print("time at 50 after begin: ", integrator.time_at_distance_after_begin(50))
print("time at 50 before end: ", integrator.time_at_distance_before_end(50))
print("time at 50 after middle: ", integrator.time_at_distance_after_middle(50))
print("time at 50 before midde: ", integrator.time_at_distance_before_middle(50))

norm = 30
print("time at norm after begin: ", integrator.time_at_distance_after_begin(norm))
print("time at norm before end: ", integrator.time_at_distance_before_end(norm))
print("time at norm after middle: ", integrator.time_at_distance_after_middle(norm))
print("time at norm before midde: ", integrator.time_at_distance_before_middle(norm))
'''
'''
integrator = VerticalIntegrator.integrate(9.1447)

print(integrator)
print("end_time: ", integrator.end_time())
norm = 7.62
print("time at norm after begin: ", integrator.time_at_distance_after_begin(norm))
print("time at norm before end: ", integrator.time_at_distance_before_end(norm))
'''
