from math import sqrt, inf
import json

class TrackMap():

    class Region():
        def __init__(self, max_speed, pwm_speed, radius, lenght, is_curve):
            self.max_speed = max_speed
            self.pwm_speed = pwm_speed
            self.radius = radius
            self.lenght = lenght
            self.is_curve = is_curve
        
        def to_json(self):
            return {
                "max_speed": self.max_speed,
                "pwm_speed": self.pwm_speed,
                "radius": self.radius,
                "lenght": self.lenght,
                "is_curve": self.is_curve
            }

    # TO BE CALCULATED
    GRAVITY = 980

    def __init__(self, encoder_right, encoder_left, A, B, min_speed=60, speed_limit=100, static_coeficient=sqrt(3), axis_distance=10):
        self.static_coeficient=static_coeficient
        self.axis_distance = axis_distance

        # 2B (CONTINUED) CALCULATED
        self.A = A
        self.B = B
        self.speed_limit = speed_limit
        self.min_speed = min_speed

        self.right_total_distance = 0 
        self.left_total_distance = 0

        self.encoder_right = encoder_right
        self.encoder_left = encoder_left

        self.track_map=[]
        self.mark_count = 0

    def curve_radius(self):
        Sr = self.encoder_right.distance - self.right_total_distance
        Sl = self.encoder_left.distance - self.left_total_distance

        # if abs(Sr - Sl) <= 1e-9:
        if abs(Sr - Sl) <= 4:
            return inf
        
        return (self.axis_distance/2)*((Sr + Sl)/abs(Sr - Sl))

    def theoretical_max_speed(self, radius):
        if radius <= 0:
            return 0
        return sqrt(self.static_coeficient * radius * self.GRAVITY)

    def meters2PWM(self, speed):
        if speed == inf:
            return self.speed_limit
        if speed < self.min_speed:
            return self.min_speed

        return self.A*speed + self.B
    
    def append_map(self):
        radius = self.curve_radius()
        max_speed = self.theoretical_max_speed(radius)
        pwm_speed = self.meters2PWM(max_speed)

        left_dist = self.encoder_left.distance - self.left_total_distance
        right_dist = self.encoder_right.distance - self.right_total_distance

        lenght = (left_dist + right_dist)/2
        is_curve = (radius != inf)

        region = self.Region(max_speed, pwm_speed, radius, lenght, is_curve)
        self.track_map.append(region)
        
        self.right_total_distance = self.encoder_right.distance
        self.left_total_distance = self.encoder_left.distance

        self.mark_count += 1

    def get_region(self):
        return self.track_map[self.mark_count]

    def to_json(self):
        map_dict = self.__dict__
        map_dict["encoder_right"] = self.encoder_right.to_json()
        map_dict["encoder_left"] = self.encoder_left.to_json()
        map_dict["mark_count"] = self.mark_count
        map_dict["track_map"] = [region.to_json() for region in self.track_map]
        return map_dict

    def to_file(self, fname):
        json_str = json.dumps(self.to_json(), indent=4)
        with open(fname, "w") as f:
            f.write(json_str)

    def from_file(self, fname):
        with open(fname, "r") as f:
            json_str = json.load(f)

        self.track_map = json_str["track_map"]
        self.A = json_str["A"]
        self.B = json_str["B"]

