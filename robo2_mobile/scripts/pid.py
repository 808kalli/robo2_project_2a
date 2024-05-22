class PID:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D
        self.oldError = 0
        self.newError = 0
        self.P_factor = 0
        self.D_factor = 0
        self.I_factor = 0
        self.value = 0
    
    def PID_calc(self, Target, Current, dt):
        self.newError = Current - Target
        self.P_factor = self.P * self.newError
        self.D_factor = (self.D * (self.newError - self.oldError))/dt
        self.I_factor += self.I_factor + (self.I * dt * self.newError)
        self.value = (self.P_factor + self.I_factor + self.D_factor)
        self.oldError = self.newError
        #print("Value: ", self.value)
        # if self.value > 1047.19754:
        #     self.value = 1047.19754
        return self.value

    # def PID_get(self):
    #     P_I_D = [self.P, self.I, self.D]
    #     return P_I_D
    
    # def PID_tune(self, P , I, D):
    #     self.P = P
    #     self.I = I
    #     self.D = D
    #     return