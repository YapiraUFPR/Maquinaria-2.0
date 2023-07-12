import math

class TrackMap():

    # TO BE CALCULATED
    STATIC_COEFICIENT = 1
    AXIS_DISTANCE = 1
    WHEEL_DIAMETER = 3.2
    REV_PULSES = 7 * 20

    GRAVITY = 980

    def __init__(self, encoder_right, encoder_left, A, B):
        self.encoder_right = encoder_right
        self.encoder_left = encoder_left

        self.track_map=[]

        # 2B (CONTINUED) CALCULATED
        self.A = A
        self.B = B

    def curveRadius(self):
        Sr = self.encoder_right.distance
        Sl = self.encoder_left.distance
        
        if abs(Sr - Sl) <= 1e-9:
            return math.inf
        
        return (self.AXIS_DISTANCE/2)*((Sr + Sl)/abs(Sr - Sl))

    def maxSpeed(self):
        r = self.curveRadius()

        # if r == math.inf:
        #     return math.inf
        
        return math.sqrt(self.STATIC_COEFICIENT * r * self.GRAVITY)

    def metersToPWM(self, speed):
        return self.A*speed + self.B
    
    def append_map(self):
        self.track_map.append(self.metersToPWM(self.maxSpeed()))

        self.encoder_left.distance = 0
        self.encoder_right.distance = 0