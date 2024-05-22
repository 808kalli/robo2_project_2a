from controller import Robot, Motor, Supervisor
import math
import time
import numpy as np

# =============== WEBOTS SETTINGS AND CONSTANTS =============== #

robot = Supervisor()
robot_node = robot.getFromDef("ICARUS")
mass = 2.5
G = 9.81
PI = 3.1415926
t1 = 3.0969e-05 #Updated
t2 = 0.001518   #Updated
q1 = 2.88424619989677e-06 #Updated                    # q1 = D/3 * t1
q2 = 0.00014137639999494  #Updated
timestep = int(robot.getBasicTimeStep())
dt = timestep/1000  # timestep is in milliseconds, so dt is seconds
# =============== CREATING CLASSES =============== #

class LandingTrajectory:
    def __init__(self, function):
        self.traj = function
    
    def calc(self, x):
        if(self.traj == 'sigma'):
            return -(10.44/(1+math.exp(-(.3*x-3))))+10.5
        '''if(self.traj == 'opt'):
            a = 2*t1
            beta = 2e-6
            gamma = beta/a
            psi = math.sqrt(1/gamma)
            return (hstart-0.08)*math.exp(-psi*x)*(math.cos(gamma*x)+(psi/gamma)*math.sin(gamma*x))+0.08'''


class Status:
    def __init__(self, value):
        self.value = value
droneStatus = Status('GROUNDED')

class PID:
    def __init__(self, P, I, D, N_filter):
        self.P = P
        self.I = I
        self.D = D
        self.N = N_filter
        self.oldError = 0
        self.newError = 0
        self.P_factor = 0
        self.D_factor = 0
        self.I_factor = 0
        self.value = 0

    def PID_calc(self, Target, Current):
        self.newError = Target - Current
        self.P_factor = self.P * self.newError
        #print("P_factor", self.P_factor)
        self.D_factor = (self.D * self.N * (self.newError - self.oldError) + self.D_factor)
        self.D_factor = self.D_factor / (dt * self.N + 1)
        #print("D_factor", self.D_factor)
        #print("I_factor", self.I_factor)
        self.value = (self.P_factor + self.I_factor + self.D_factor)
        self.I_factor = self.I_factor + (self.I * dt * self.newError)
        self.oldError = self.newError
        return self.value
    
    def simplePID_calc(self, Target, Current):
        self.newError = Target - Current
        self.P_factor = self.P * self.newError
        # print("P_factor", self.P_factor)
        self.D_factor = (self.D * (self.newError - self.oldError))/dt
        # print("D_factor", self.D_factor)
        # print("I_factor", self.I_factor)
        #print("New Error: ", (self.I * dt * self.newError))
        self.I_factor += self.I_factor + (self.I * dt * self.newError)
        self.value = (self.P_factor + self.I_factor + self.D_factor)
        self.oldError = self.newError
        #print("Value: ", self.value)
        if self.value > 1047.19754:
            self.value = 1047.19754
        return self.value

    def PID_get(self):
        P_I_D = [self.P, self.I, self.D]
        return P_I_D
    
    def PID_tune(self, P , I, D):
        self.P = P
        self.I = I
        self.D = D
        return

yPID = PID(.95, 0, .9, 10000)
xPID = PID(.95, 0, .9, 10000)
# Combination 1
zPID = PID(60034.262245669, 211041.54323994, 5532.7804384950, 10000)

yawPID = PID(250, 0, 97, 10000)
rollPID = PID(310, 0, 50, 10000)
pitchPID = PID(310, 0, 50, 10000)

class Coordinate_Matrix:
    def __init__(self):
        self.Rz = np.identity(3)
        self.Tr = np.zeros(3)

    def update(self, imu):
        yaw = math.radians(imu[2])
        self.Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        self.Tr = np.array(in_position)

    def getPoint(self, q):
        R = self.Rz
        r = self.Tr
        R = np.transpose(R)
        p = np.array(q)
        d = np.matmul(R, np.subtract(p,r))
        return d.flatten().tolist()

    def getSpeed(self, s):
        R = self.Rz
        R = np.transpose(R)
        p = np.array(s)
        d = np.matmul(R, p)
        return d.flatten().tolist()
    
    
body_fixed = Coordinate_Matrix()
land_traj = LandingTrajectory('sigma')

# =============== NAMES OF MOTORS =============== #

motors = []
motorsNames = ["motorRF",  "motorLF", "motorRB", "motorLB"]

# =============== SETTING INITIAL TARGETS FOR TAKE OFF =============== #

posTarget = 3*[0]
pitchTarget = 0
rollTarget = 0
yawTarget = 0

Destination = [50, 50, 1]
imuData = 3*[0]

# =============== SETTING MOTORS LIST =============== #

for name in motorsNames:
    motors.append(robot.getDevice(name))
    motors[-1].setPosition(float('+inf'))
    motors[-1].setVelocity(0.0)
    #motors[-1].enableForceFeedback(20)
    #motors[-1].enableTorqueFeedback(20)

rpms = 4*[0]

# =============== INITIALIZING SENSORS =============== #

gps = robot.getDevice("gps")
gps.enable(20)
imu = robot.getDevice("imu")
imu.enable(10)
accMeter = robot.getDevice("accelerometer")
accMeter.enable(10)


# =============== INITIALIZING SPEEDS AND POSITIONS IN INITIAL COORDINATE SYSTEM =============== #

in_position = 3*[0]
in_speed = 3*[0]

# =============== INITIAZING SPEEDS AND POSITIONS RELATIVE IN BODY FIXED COORDINATE SYSTEM =============== #

position = 3*[0]
speed = 3*[0]
angularSpeed = 3*[0]

# =============== PRINT STARTING MESSAGES =============== #

print("WELCOME!\n···")
print("ICARUS v3 Test Initiated.")
print("posTarget Altitude = {:.1f} meters.".format(posTarget[2]))
print("Destination = ({:.1f}, {:.1f}).".format(Destination[0], Destination[1]))

# =============== GET STARTING TIME =============== #

startTime = time.time()
landing_time = 0

# ===============  WAIT FOR DELIVERY =============== #

def CheckTelemetry():
    global posTarget, yawTarget
    if ((time.time() - startTime >= 2) and (droneStatus.value == 'GROUNDED')):
        droneStatus.value = 'TAKE_OFF'
        posTarget[2] = Destination[2]
        yawTarget = math.degrees(math.atan2(destination[1], destination[0]))
        yawPID.oldError = yawTarget
        print("VALID DELIVERY REQUEST")

# =============== GET ROLL-PITCH-YAW AND X-Y-Z VALUES =============== #
def Sys_Tr():
    global position, speed, destination, yawTarget
    body_fixed.update(imuData)
    position = body_fixed.getPoint(in_position)
    destination = body_fixed.getPoint(Destination)
    speed = body_fixed.getSpeed(in_speed)
    return

def SensorReadings():
    global imuData, in_position
    in_position_old = in_position
    imuDataOld = imuData
    imuData = [math.degrees(angle) for angle in imu.getRollPitchYaw()]
    if(abs(imuData[2]-imuDataOld[2]) > 180):
        if (imuData[2] > 0):
            imuData[2] -= 360
        elif(imuData[2] < 0):
            imuData[2] += 360
    in_position = gps.getValues()
    in_SpeedData(in_position_old, imuDataOld)
    Sys_Tr()
    return

def in_SpeedData(in_pos_old, imuDataOld):
    global in_speed, angularSpeed
    for i in range(3):
        in_speed[i] = (in_position[i] - in_pos_old[i])/dt
        angularSpeed[i] = (imuData[i] - imuDataOld[i])/dt
    return

# =============== GET ANGLE OF OUR POSITION RELATIVE TO INERTIAL COORDINATE SYSTEM =============== #

destination = 3*[0]


def NextStep():
    global droneStatus, posTarget, landing_time, Destination, yawTarget
    if (droneStatus.value == 'GROUNDED'):
        CheckTelemetry()
    elif (droneStatus.value == 'TAKE_OFF'):
        if(abs(imuData[2]-yawTarget) < 0.1 and abs(angularSpeed[2]) < 0.01 and abs(in_position[2] - posTarget[2]) < 0.5 and abs(speed[2]) < 0.1):
            droneStatus.value = 'TRAVEL'
        return
    elif(droneStatus.value == 'TRAVEL'):
        yawTarget = imuData[2] + math.degrees(math.atan2(destination[1], destination[0]))
        if(math.sqrt(abs(position[0] - destination[0])**2 + abs(position[1] - destination[1])) < 2):
            droneStatus.value = 'ADJUST'
            yawTarget = imuData[2]
        return
    elif(droneStatus.value == 'ADJUST'):
        if(abs(speed[0]) < 0.5 and abs(in_position[0]-Destination[0]) < 0.1 and abs(in_position[1]-Destination[1]) < 0.1):
            droneStatus.value = 'LAND'
            landing_time = time.time()
        return
    elif(droneStatus.value == 'LAND'):
        t = time.time() - landing_time
        posTarget[2] = land_traj.calc(t)
        if(posTarget[2] < 0.07):
            droneStatus.value = 'LANDED'
        return

def Z_PID(PID_object, Target, Current):
    #print("Z PID")
    value = PID_object.simplePID_calc(Target, Current)
    #print("Value: ", value)
    if value < 0:
        value = 0
    return [value, value, value, value]

def PitchPID(PID_object, Target, Current):
    value = PID_object.PID_calc(Target, Current)
    T = 4*[0]
    if(value < 0):
        T[1] = abs(value)
        T[0] = abs(value)
    else:
        T[3] = value
        T[2] = value
    return T

def RollPID(PID_object, Target, Current):
    value = PID_object.PID_calc(Target, Current)
    T = 4*[0]
    if(value < 0):
        T[0] = abs(value)
        T[2] = abs(value)
    else:
        T[1] = value
        T[3] = value
    return T

def YawPID(PID_object, Target, Current):
    value = PID_object.PID_calc(Target, Current)
    T = 4*[0]
    if(value < 0):
        T[1] = abs(value)
        T[2] = abs(value)
    else:
        T[0] = value
        T[3] = value
    return T

def X_PID(PID_object, Target, Current):
    pitchTarget = PID_object.PID_calc(Target, Current)
    if pitchTarget > 5:
        pitchTarget = 5
    elif pitchTarget < -5:
        pitchTarget = -5
    return pitchTarget

def Y_PID(PID_object, Target, Current):
    rollTarget = PID_object.PID_calc(Target, Current)

    if rollTarget > 5:
        rollTarget = 5
    elif rollTarget < -5:
        rollTarget = -5
    return -rollTarget

def TorqueAndThrust():
 
    global pitchTarget, rollTarget, posTarget
    w = 4*[0]
    dw_pitch = 4*[0]
    dw_roll = 4*[0]
    dw_yaw = 4*[0]
    if(droneStatus.value != 'GROUNDED'):
        
        if(droneStatus.value != 'TAKE_OFF'):
            posTarget[0], posTarget[1], _ = destination

        w = Z_PID(zPID, posTarget[2], in_position[2])

        pitchTarget = X_PID(xPID, posTarget[0], position[0])

        rollTarget = Y_PID(yPID, posTarget[1], position[1])

        # =============== PITCH ERROR CORRECTION =============== #
        dw_pitch = PitchPID(pitchPID, pitchTarget, imuData[1])

        # =============== ROLL ERROR CORRECTION =============== # 
        dw_roll = RollPID(rollPID, rollTarget, imuData[0])

        # =============== YAW ERROR CORRECTION =============== #
        dw_yaw = YawPID(yawPID, yawTarget, imuData[2])
        #dw_yaw = 0
    return w, dw_pitch, dw_roll, dw_yaw

def setMotors():
    global rpms, motors
    w, dw_pitch, dw_roll, dw_yaw = TorqueAndThrust()
    #print(w)
    #print(dw_pitch)
    #print(dw_roll)
    #print(dw_yaw)
    for i in range(4):
        rpms[i] = w[i]
        #ssrpms[i] += dw_pitch[i]  # Add Pitch PID
        #rpms[i] += dw_roll[i]  # Add Roll PID
        #rpms[i] += dw_yaw[i]  # Add Yaw PID
        #rpms[i] = 500
    for n in range(4):
        if(droneStatus.value != 'LANDED'):
            motors[n].setVelocity(rpms[n])
        else:
            motors[n].setVelocity(0)
    

def ShowData():
    if (droneStatus.value != 'GROUNDED'):       
        # =============== PRINT INFO =============== #
        #print("yawTarget= {:.1f}°, pitchTarget= {:.1f}°, rollTarget= {:.1f}°".format(yawTarget, pitchTarget, rollTarget))
        #print("Calculated Vertical Speed: {:.2f} m/s".format(speed[2]))
        #print("Real Vertical Speed: {:.2f} m/s".format(robot_node.getVelocity()[2]))
        #print("Calculated Horizontal Speed: {:.2f} m/s".format(speed[0]))
        print("Real Horizontal Speed: {:.2f} m/s".format(math.sqrt(robot_node.getVelocity()[0]**2+robot_node.getVelocity()[1]**2)))
        #print("PosTarget[2]: ", posTarget[2])
        #print("in_position (X, Y, Z): {:.3f}m {:.3f}m {:.3f}m".format(in_position[0], in_position[1], in_position[2]))
        #print("Position Relative to Icarus: {:.3f}m {:.3f}m {:.3f}m".format(position[0], position[1], position[2]))
        print("PosTarget (X, Y, Z): {:.3f}m {:.3f}m {:.3f}m".format(posTarget[0], posTarget[1], posTarget[2]))
        print("Orientation (X, Y, Z): {:.3f}° {:.3f}° {:.3f}°".format(imuData[0], imuData[1], imuData[2]))
        #print("Orientation (X, Y, Z): ", imuData)
        #print("MotorRF Force {:.3f} r/s, MotorLF Force {:.3f}r/s, MotorRB Force {:.3f}r/s, MotorLB Force {:.3f}r/s".format(motors[0].getForceFeedback(), motors[1].getForceFeedback(), motors[2].getForceFeedback(), motors[3].getForceFeedback()))
        #print("MotorRF Torque {:.3f} r/s, MotorLF Torque {:.3f}r/s, MotorRB Torque {:.3f}r/s, MotorLB Torque {:.3f}r/s".format(motors[0].getTorqueFeedback(), motors[1].getTorqueFeedback(), motors[2].getTorqueFeedback(), motors[3].getTorqueFeedback()))
        print("MotorRF {:.3f} r/s, MotorLF {:.3f}r/s, MotorRB {:.3f}r/s, MotorLB {:.3f}r/s".format(rpms[0], rpms[1],rpms[2], rpms[3]))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

while robot.step(timestep) != -1:

    # =============== SENSOR READINGS =============== #
    SensorReadings()

 # =============== DECIDE WHAT TO DO  =============== #
    NextStep()

    # =============== TURN PID VALUES TO RPM =============== #
    setMotors()

    # =============== PRINT OUT DATA =============== #
    ShowData() 

    if(droneStatus.value == 'LANDED'):
        break

print("END OF FLIGHT")

'''
OLD:

      thrustConstants 1.96503e-05 0
      torqueConstants 3.22606e-07 0

NEW:
      thrustConstants 3.069e-05 0.001518
      torqueConstants 2.858e-06 0.00014376

TRY:
      thrustConstants -3.069e-05 -0.001518
      torqueConstants 2.858e-06 0.00014376
'''      

