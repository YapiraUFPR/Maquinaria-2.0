from libs.track_map import TrackMap

class Encoder():
    def __init__(self, encoder_a, encoder_b, steps, radius_wheel):
        # init pins
        self.encoder_a = encoder_a
        self.encoder_b = encoder_b

        self.steps = steps
        self.radius_wheel = radius_wheel

        self.distance = 0
        
    def to_json(self):
        return {
        "pin_a": self.encoder_a,
        "pin_b": self.encoder_b,
        "steps": self.steps,
        "radius_wheel": self.radius_wheel,
    }   

encoder_right = Encoder(1, 2, 7*20, 1.6)
encoder_left = Encoder(3, 4, 7*20, 1.6)

encoder_left.distance = 1
encoder_right.distance = 1

map = TrackMap(encoder_left, encoder_right, 0.526, 2.789)

# map.from_file("map.json")
# print(map.track_map, type(map.track_map))

map.append_map()

encoder_left.distance += 5
encoder_right.distance += 5

map.append_map()

encoder_left.distance += 10
encoder_right.distance += 10

map.append_map()

encoder_left.distance += 3
encoder_right.distance += 7

map.append_map()

encoder_left.distance += 10
encoder_right.distance += 8

map.append_map()

map.to_file("map.json")