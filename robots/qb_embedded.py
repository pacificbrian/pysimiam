#!/usr/bin/python
"""
@brief QuickBot class for Beaglebone Black

@author Rowland O'Flaherty (rowlandoflaherty.com)
@date 02/07/2014
@version: 1.0
@copyright: Copyright (C) 2014, Georgia Tech Research Corporation see the LICENSE file included with this software (see LINENSE file)
"""

import os
import sys
import time
import math
import re
import threading
import numpy as np

import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC

# Tic Toc Variables
TICTOC_START = 0
TICTOC_COUNT = 0
TICTOC_MEAN = 0
TICTOC_MAX = -float('inf')
TICTOC_MIN = float('inf')

# Constants
LEFT = 0
RIGHT = 1
MIN = 0
MAX = 1

ADCTIME = 0.001

ENC_BUF_SIZE = 2**9

ENC_IND = [0, 0]
ENC_TIME = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]
ENC_VAL = [[0]*ENC_BUF_SIZE, [0]*ENC_BUF_SIZE]

ADC_LOCK = threading.Lock()

RUN_FLAG = True
RUN_FLAG_LOCK = threading.Lock()

DEBUG = False

class QBEmbedded():
    """The QuickBot Class"""

    # === Class Properties ===
    # Parameters
    sampleTime = 20.0 / 1000.0

    # Pins
    ledPin = 'USR1'

    # Motor Pins -- (LEFT, RIGHT)
    dir1Pin = ('P8_12', 'P8_14')
    dir2Pin = ('P8_10', 'P8_16')
    pwmPin = ('P9_14', 'P9_16')

    # ADC Pins
    irPin = ('P9_38', 'P9_40', 'P9_36', 'P9_35', 'P9_33')
    encoderPin = ('P9_39','P9_37')

    # Encoder counting parameter and variables
    winSize = 2**5 # Should be power of 2
    ticksPerTurn = 16 # Number of ticks on encoder disc
#     minTickVelThreshold = 0.54 # Threshold on the slowest tick velocity
    minPWMThreshold = [45, 45] # Threshold on the minimum magnitude of a PWM input value
#     vel95PrctRiseTime = 1.0 # Time it takes tick velocity to get to 95% of steady state value
    encTPrev = [0.0, 0.0]
    encThreshold = [1200.0, 1200.0]
    encTickState = [0, 0]
    encTickStateVec = np.zeros((2,winSize))

    # State -- (LEFT, RIGHT)
    pwm = [0, 0]

    irVal = [0.0, 0.0, 0.0, 0.0, 0.0]
    ithIR = 0

    encTime = [0.0, 0.0]
    encPos = [0.0, 0.0]
    encVel = [0.0, 0.0]
    encVelVar = [0.1, 0.1]

    encSumN = [0, 0]
    encBufInd0 = [0, 0]
    encBufInd1 = [0, 0]
    encTimeWin = np.zeros((2,winSize))
    encValWin = np.zeros((2,winSize))
    encPWMWin = np.zeros((2,winSize))
    encTau = [0.0, 0.0]
    encCnt = 0;

    # Record variables
    encRecSize = 2**13
    encRecInd = [0, 0]
    encTimeRec = np.zeros((2,encRecSize))
    encValRec = np.zeros((2,encRecSize))
    encPWMRec = np.zeros((2,encRecSize))
    encNNewRec = np.zeros((2,encRecSize))
    encPosRec = np.zeros((2,encRecSize))
    encVelRec = np.zeros((2,encRecSize))

    # Constraints
    pwmLimits = [-100, 100] # [min, max]

    # Variables
    ledFlag = True
    cmdBuffer = ''

    # === Class Methods ===
    # Constructor
    def __init__(self):

        # Initialize GPIO pins
        GPIO.setup(self.dir1Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[LEFT], GPIO.OUT)
        GPIO.setup(self.dir1Pin[RIGHT], GPIO.OUT)
        GPIO.setup(self.dir2Pin[RIGHT], GPIO.OUT)

        GPIO.setup(self.ledPin, GPIO.OUT)

        # Initialize PWM pins: PWM.start(channel, duty, freq=2000, polarity=0)
        PWM.start(self.pwmPin[LEFT], 0)
        PWM.start(self.pwmPin[RIGHT], 0)

        # Set motor speed to 0
        self.pwmcache = [0,0]
        self.setPWM([0, 0])

        # Initialize ADC
        ADC.setup()
        self.encoderRead = encoderRead(self.encoderPin)

        self.info = Struct()
        self.info.wheels = Struct()

        self.info.wheels.radius = 0.0325
        self.info.wheels.base_length = 0.09925
        self.info.wheels.ticks_per_rev = 16
        self.info.wheels.left_ticks = 0
        self.info.wheels.right_ticks = 0
        
        self.info.wheels.max_velocity = 2*pi*130/60 # 130 RPM
        self.info.wheels.min_velocity = 2*pi*30/60  #  30 RPM

        #self.info.wheels.vel_left = 0
        #self.info.wheels.vel_right = 0

        self.info.ir_sensors = Struct()
        self.info.ir_sensors.poses = [
                          Pose(-0.0474, 0.0534, np.radians(90)),
                          Pose( 0.0613, 0.0244, np.radians(45)),
                          Pose( 0.0636, 0.0, np.radians(0)),
                          Pose( 0.0461,-0.0396, np.radians(-45)),
                          Pose(-0.0690,-0.0534, np.radians(-90))
                          ]
        self.info.ir_sensors.rmax = 0.3
        self.info.ir_sensors.rmin = 0.04
        self.info.ir_sensors.readings = self.irVal      
        
    def get_info(self):
        return self.info

    # Getters and Setters
    def setPWM(self, pwm):
        # [leftSpeed, rightSpeed]: 0 is off, caps at min and max values

        self.pwm[LEFT] = min(max(pwm[LEFT], self.pwmLimits[MIN]), self.pwmLimits[MAX])
        self.pwm[RIGHT] = min(max(pwm[RIGHT], self.pwmLimits[MIN]), self.pwmLimits[MAX])

        # Left motor
        if self.pwm[LEFT] > 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        elif self.pwm[LEFT] < 0:
            GPIO.output(self.dir1Pin[LEFT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], abs(self.pwm[LEFT]))
        else:
            GPIO.output(self.dir1Pin[LEFT], GPIO.LOW)
            GPIO.output(self.dir2Pin[LEFT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[LEFT], 0)

        # Right motor
        if self.pwm[RIGHT] > 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.HIGH)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        elif self.pwm[RIGHT] < 0:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.HIGH)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], abs(self.pwm[RIGHT]))
        else:
            GPIO.output(self.dir1Pin[RIGHT], GPIO.LOW)
            GPIO.output(self.dir2Pin[RIGHT], GPIO.LOW)
            PWM.set_duty_cycle(self.pwmPin[RIGHT], 0)

    # Methods
    def run(self):
        global RUN_FLAG
        self.encoderRead.start()

        if DEBUG:
                while RUN_FLAG == True:
#                     tic()
                    self.update()
    
                    # Flash BBB LED
                    if self.ledFlag == True:
                        self.ledFlag = False
                        GPIO.output(self.ledPin, GPIO.HIGH)
                    else:
                        self.ledFlag = True
                        GPIO.output(self.ledPin, GPIO.LOW)
                    time.sleep(self.sampleTime)
#                     toc("Run loop")
        else:
            try:
                while RUN_FLAG == True:
                    self.update()
    
                    # Flash BBB LED
                    if self.ledFlag == True:
                        self.ledFlag = False
                        GPIO.output(self.ledPin, GPIO.HIGH)
                    else:
                        self.ledFlag = True
                        GPIO.output(self.ledPin, GPIO.LOW)
                    time.sleep(self.sampleTime)
            except:
                RUN_FLAG = False

        self.cleanup()
        return

    def v2pwm(self,vl,vr):
        """Convert from angular velocity to PWM"""

        pwm_l = sign(vl)*(abs(vl) - self.beta[1])/self.beta[0];
        pwm_r = sign(vr)*(abs(vr) - self.beta[1])/self.beta[0];        

        return pwm_l, pwm_r

    def pwm2v(self,pwm_l,pwm_r):
        """Convert from PWM to angular velocity"""
        vl = sign(pwm_l)*(self.beta[0]*abs(pwm_l) + self.beta[1])
        vr = sign(pwm_r)*(self.beta[0]*abs(pwm_r) + self.beta[1])

        return vl, vr 
    
    def set_inputs(self, inputs):
        self.pwmcache = self.v2pwm(*inputs)
        self.setPWM(self.pwmcache)

    def pause(self):
        self.pwmcache = self.pwm
        self.setPWM([0,0])
    
    def resume(self):
        self.setPWM(self.pwmcache)
    
    def update_external_info(self):
        self.readIRValues()
#        self.info.ir_sensors.readings = self.irVal
        self.readEncoderValues()
        self.info.wheels.left_ticks = self.encPos[LEFT]
        self.info.wheels.right_ticks = self.encPos[RIGHT]

    def cleanup(self):
        print("Clean up")
        self.setPWM([0, 0])
        self.robotSocket.close()
        GPIO.cleanup()
        PWM.cleanup()
#         tictocPrint()
#         self.writeBufferToFile()

    def readIRValues(self):
        prevVal = self.irVal[self.ithIR]
        ADC_LOCK.acquire()
        self.irVal[self.ithIR] = ADC.read_raw(self.irPin[self.ithIR])
        time.sleep(ADCTIME)
        ADC_LOCK.release()

        if self.irVal[self.ithIR] >= 1100:
                self.irVal[self.ithIR] = prevVal

        self.ithIR = ((self.ithIR+1) % 5)


    def readEncoderValues(self):
        self.encCnt = self.encCnt + 1;
        # Fill window
        for side in range(0,2):
            self.encTime[side] = self.encTimeWin[side][-1]

            self.encBufInd0[side] = self.encBufInd1[side]
            self.encBufInd1[side] = ENC_IND[side]
            ind0 = self.encBufInd0[side] # starting index
            ind1 = self.encBufInd1[side] # ending index (this element is not included until the next update)

            if ind0 < ind1:
                N = ind1 - ind0 # number of elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:ind1]
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encValWin[side, -N:] = ENC_VAL[side][ind0:ind1]
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]]*N

            elif ind0 > ind1:
                N = ENC_BUF_SIZE - ind0 + ind1 # number of elements
                self.encSumN[side] = self.encSumN[side] + N
                self.encTimeWin[side] = np.roll(self.encTimeWin[side], -N)
                self.encValWin[side] = np.roll(self.encValWin[side], -N)
                self.encPWMWin[side] = np.roll(self.encPWMWin[side], -N)
                self.encPWMWin[side, -N:] = [self.pwm[side]]*N
                if ind1 == 0:
                    self.encTimeWin[side, -N:] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:] = ENC_VAL[side][ind0:]
                else:
                    self.encTimeWin[side, -N:-ind1] = ENC_TIME[side][ind0:]
                    self.encValWin[side, -N:-ind1] = ENC_VAL[side][ind0:]
                    self.encTimeWin[side, -ind1:] = ENC_TIME[side][0:ind1]
                    self.encValWin[side, -ind1:] = ENC_VAL[side][0:ind1]

            if ind0 != ind1:
                tauNew = self.encTimeWin[side,-1] - self.encTimeWin[side,-N]
                self.encTau[side] = tauNew / self.encCnt + self.encTau[side] * (self.encCnt-1)/self.encCnt # Running average
                if self.encSumN[side] > self.winSize:
                    self.countEncoderTicks(side)

                # Fill records
                ind = self.encRecInd[side]
                if ind+N < self.encRecSize:
                    self.encTimeRec[side, ind:ind+N] = self.encTimeWin[side, -N:]
                    self.encValRec[side, ind:ind+N] = self.encValWin[side, -N:]
                    self.encPWMRec[side, ind:ind+N] = self.encPWMWin[side, -N:]
                    self.encNNewRec[side, ind:ind+N] = [N]*N
                    self.encPosRec[side, ind:ind+N] = [self.encPos[side]]*N
                    self.encVelRec[side, ind:ind+N] = [self.encVel[side]]*N
                self.encRecInd[side] = ind+N

    def countEncoderTicks(self,side):
        # Set parameters
        zeroInputHoldCnt = 2**4
        highLowHighHoldCnt = 2**5
        
        # Set variables
        t = self.encTimeWin[side] # Time vector of data (not consistent sampling time)
        tPrev = self.encTPrev[side] # Previous update time
        pwm = self.encPWMWin[side] # PWM vector of data
        input = pwm[-1] # Last pwm value applied
        tickStatePrev = self.encTickState[side]  # Last state of tick (high (1), low (-1), or unsure(0))
        tickCnt = self.encPos[side] # Current tick count
        tickVel = self.encVel[side] # Current tick velocity
        encValWin = self.encValWin[side] # Encoder raw value buffer
        threshold = self.encThreshold[side] # Tick threshold
        minPWMThreshold = self.minPWMThreshold[side] # Minimum PWM to move wheel
        
        N = sum(t > tPrev) # Number of new updates
        
        tickStateVec = np.roll(self.encTickStateVec[side], -N)
        
        # Determine wheel direction
        if tickVel != 0:
            wheelDir = np.sign(tickVel)
        else:
            wheelDir = np.sign(input)
            
        # Count ticks and record tick state
        indTuple = np.where(t == tPrev)
        if len(indTuple[0] > 0):
            ind = indTuple[0][0]
            newInds = ind + np.arange(1,N+1)
            for i in newInds:
                if encValWin[i] > threshold:
                    tickState = 1
                    if tickStatePrev == -1:
                        tickCnt = tickCnt + wheelDir
                        
                else:
                    tickState = -1
                tickStatePrev = tickState
                tickStateVec[i] = tickState
                
            # Measure tick speed
            diffTickStateVec = np.diff(tickStateVec) # Tick state transition differences
            fallingTimes = t[np.hstack((False,diffTickStateVec == -2))] # Times when tick state goes from high to low
            risingTimes = t[np.hstack((False,diffTickStateVec == 2))] # Times when tick state goes from low to high
            fallingPeriods = np.diff(fallingTimes) # Period times between falling edges
            risingPeriods = np.diff(risingTimes) # Period times between rising edges
            tickPeriods = np.hstack((fallingPeriods, risingPeriods))
            if len(tickPeriods) == 0:
                if all(pwm[newInds] < minPWMThreshold): # If all inputs are less than min set velocity to 0
                    tickVel = 0
            else:
                tickVel = wheelDir * 1/np.mean(tickPeriods) # Average tick frequency in wheel direction
            
            # Update variables
            self.encPos[side] = tickCnt # New tick count
            self.encVel[side] = tickVel # New tick velocity
            self.encTickStateVec[side] = tickStateVec # New tick state vector
        self.encTPrev[side] = t[-1] # New latest update time


    def writeBufferToFile(self):
        matrix = list(map(list, list(zip(*[self.encTimeRec[LEFT], self.encValRec[LEFT], self.encPWMRec[LEFT], self.encNNewRec[LEFT], \
                                 self.encTimeRec[RIGHT], self.encValRec[RIGHT], self.encPWMRec[RIGHT], self.encNNewRec[RIGHT]]))))
        s = [[str(e) for e in row] for row in matrix]
        lens = [len(max(col, key=len)) for col in zip(*s)]
        fmt = '\t'.join('{{:{}}}'.format(x) for x in lens)
        table = [fmt.format(*row) for row in s]
        f = open('output.txt','w')
        f.write('\n'.join(table))
        f.close()
        print("Wrote buffer to output.txt")


class encoderRead(threading.Thread):
    """The encoderRead Class"""

    # === Class Properties ===
    # Parameters

    # === Class Methods ===
    # Constructor
    def __init__(self,encPin=('P9_39', 'P9_37')):

        # Initialize thread
        threading.Thread.__init__(self)

        # Set properties
        self.encPin = encPin

    # Methods
    def run(self):
        global RUN_FLAG

        self.t0 = time.time()
        while RUN_FLAG:
            global ENC_IND
            global ENC_TIME
            global ENC_VAL

            for side in range(0,2):
                ENC_TIME[side][ENC_IND[side]] = time.time() - self.t0
                ADC_LOCK.acquire()
                ENC_VAL[side][ENC_IND[side]] = ADC.read_raw(self.encPin[side])
                time.sleep(ADCTIME)
                ADC_LOCK.release()
                ENC_IND[side] = (ENC_IND[side] + 1) % ENC_BUF_SIZE


def operatingPoint(uStar, uStarThreshold):
    """ This function returns the steady state tick velocity given some PWM input.

    uStar: PWM input.
    uStarThreshold: Threshold on the minimum magnitude of a PWM input value

    returns: omegaStar - steady state tick velocity
    """
    # Matlab code to find beta values
    # X = [40; 80; 100]; % Air Test
    # Y = [0.85; 2.144; 3.5];
    #
    # r = 0.0325; % Wheel radius
    # c = 2*pi*r;
    # X = [  70;   70;   70;   75;   75;   75;   80;   80;   80; 85;     85;   85;   90;   90;   90]; % Ground Test
    # Z = [4.25; 3.95; 4.23; 3.67; 3.53; 3.48; 3.19; 3.08; 2.93; 2.52; 2.59; 2.56; 1.99; 2.02; 2.04]; % Time to go 1 m
    # Y = 1./(Z*c);
    # H = [X ones(size(X))];
    # beta = H \ Y
    # beta = [0.0425, -0.9504] # Air Test Results
    beta = [0.0606, -3.1475] # Ground Test Results

    if np.abs(uStar) <= uStarThreshold:
        omegaStar = 0.0
    elif uStar > 0:
        omegaStar = beta[0]*uStar + beta[1]
    else:
        omegaStar = -1.0*(beta[0]*np.abs(uStar) + beta[1])

    return omegaStar


def kalman(x, P, Phi, H, W, V, z):
    """This function returns an optimal expected value of the state and covariance
    error matrix given an update and system parameters.

    x:   Estimate of staet at time t-1.
    P:   Estimate of error covariance matrix at time t-1.
    Phi: Discrete time state tranistion matrix at time t-1.
    H:   Observation model matrix at time t.
    W:   Process noise covariance at time t-1.
    V:   Measurement noise covariance at time t.
    z:   Measurement at time t.

    returns: (x,P) tuple
    x: Updated estimate of state at time t.
    P: Updated estimate of error covariance matrix at time t.

    """
    x_p = Phi*x # Prediction of setimated state vector
    P_p = Phi*P*Phi + W # Prediction of error covariance matrix
    S = H*P_p*H + V # Sum of error variances
    S_inv = 1/S # Inverse of sum of error variances
    K = P_p*H*S_inv # Kalman gain
    r = z - H*x_p # Prediction residual
    w = -K*r # Process error
    x = x_p - w # Update estimated state vector
    v = z - H*x # Measurement error
    if np.isnan(K*V):
        P = P_p
    else:
        P = (1 - K*H)*P_p*(1 - K*H) + K*V*K # Updated error covariance matrix

    return (x, P)

def tic():
    global TICTOC_START
    TICTOC_START = time.time()

def toc(tictocName='toc', printFlag=True):
    global TICTOC_START
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    tictocTime = time.time() - TICTOC_START
    TICTOC_COUNT = TICTOC_COUNT + 1
    TICTOC_MEAN = tictocTime / TICTOC_COUNT + TICTOC_MEAN * (TICTOC_COUNT-1) / TICTOC_COUNT
    TICTOC_MAX = max(TICTOC_MAX,tictocTime)
    TICTOC_MIN = min(TICTOC_MIN,tictocTime)

    if printFlag:
        print(tictocName + " time: " + str(tictocTime))

def tictocPrint():
    global TICTOC_COUNT
    global TICTOC_MEAN
    global TICTOC_MAX
    global TICTOC_MIN

    print("Tic Toc Stats:")
    print("Count = " + str(TICTOC_COUNT))
    print("Mean = " + str(TICTOC_MEAN))
    print("Max = " + str(TICTOC_MAX))
    print("Min = " + str(TICTOC_MIN))


